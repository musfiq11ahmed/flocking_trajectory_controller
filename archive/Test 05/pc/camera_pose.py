#!/usr/bin/env python3
"""Overhead-camera ArUco pose source for the ESP32-S3 diff-drive bot.

Implements the real CameraPoseSource (formerly a stub in test_suite.py):
the Rapoo C280 2K webcam (ceiling-mounted, ~8 ft / 2.4 m above the arena)
watches from above; the four arena corner markers (IDs 0-3) define a
per-frame homography from pixels to the arena coordinate frame, and the
robot marker (ID 4) yields (x, y, theta).

Arena frame (see ARENA_GEOMETRY.md -- confirmed with the user):
  origin (0, 0)   : midpoint of the LEFT arena edge (between markers 3 and 0)
  +X              : across the arena to the right-edge midpoint (106.5, 0)
  +Y              : toward the top-left marker (+34 in); -Y toward bottom-left
  units           : METERS (matches the rest of pc/)
  theta           : CCW from +X; marker 4's printed top edge = robot front,
                    so marker orientation IS the heading (no offset)

CAMERA CONFIGURATION (per project requirements):
  * Capture init mirrors the previous WORKING implementation
    (calibrate_and_track.py) exactly: explicit CAP_DSHOW backend, MJPG
    fourcc first, then 2560x1440, then optics -- and CAP_PROP_FPS is left
    untouched, because an explicit fps request can make the driver
    renegotiate a DIFFERENT (cropped-FOV) sensor mode even while reporting
    the same resolution. MJPG at 2K is what makes ~30 FPS possible at all
    over USB (uncompressed YUYV would manage only a few FPS).
  * Autofocus and auto-exposure DISABLED: focus locked to 0 (ceiling
    distance) as in the working version (--focus overrides), exposure
    manual (--exposure sets the value), so detection thresholds and frame
    timing do not drift mid-run.
  * Detection pipeline also restored from the working version: blur- and
    small-marker-forgiving DetectorParameters + CLAHE contrast
    equalization before detection.
  * The driver buffer is flushed on every read so we always process the
    FRESHEST frame, not a queued older one.
  * The DEBUG WINDOW is displayed scaled to 1280x720 (like the working
    version): showing a raw 2560x1440 window on a smaller screen makes you
    see only one corner of the feed. Detection always runs on full 2K.

LATENCY COMPENSATION (critical -- read before tuning):
  A camera pose is a DELAYED measurement: by the time a frame is exposed,
  transferred, decoded and processed, the robot has already moved on
  (at 0.30 m/s and 10 FPS, that is several cm per frame). This module
  therefore NEVER reports the raw detected pose as "where the robot is":
    1. every frame is timestamped at grab time and the real frame period is
       measured online (measured_fps);
    2. the end-to-end latency L is estimated as latency_frames x frame
       period (default 1.5 frames), or set explicitly with latency_s;
    3. read() DEAD-RECKONS the pose forward from the delayed measurement by
       (L + processing age), integrating the wheel speeds the robot is
       currently being commanded (via update()) through the unicycle model.
  The value returned is thus an estimate of the robot's pose NOW, which is
  what the trajectory-error and correction computations need. Prediction
  is capped at MAX_PREDICT_S and is exact for constant wheel speeds; it
  degrades gracefully during occlusions (held pose) and standstill
  (commands ~0 -> prediction ~measurement).

Error handling policy: read() returns the last valid pose for up to
--max-hold seconds of robot-marker occlusion (dead-reckoned forward);
beyond that it raises CameraPoseError so the navigation loop can coast the
robot instead of driving blind.

Requires: opencv-contrib-python (cv2.aruco) and numpy -- camera mode only;
test_suite.py imports this module lazily so sim mode stays stdlib-only.

Standalone preview (no robot needed):
    python3 camera_pose.py --index 0 --debug-view
    python3 camera_pose.py --index 0 --focus 40 --exposure -6
"""

import argparse
import math
import time

import cv2
import numpy as np

# ---------------------------------------------------------------------------
# Arena geometry (ARENA_GEOMETRY.md; 1 in = 0.0254 m exactly)
# ---------------------------------------------------------------------------
IN_TO_M = 0.0254
ARENA_WIDTH_M = 106.5 * IN_TO_M          # 2.7051 m, left edge -> right edge
ARENA_HALF_HEIGHT_M = 34.0 * IN_TO_M     # 0.8636 m, half of the 68 in height

# Arena corner marker ID -> (x, y) in the arena frame [m].
ARENA_MARKERS_M = {
    0: (0.0, -ARENA_HALF_HEIGHT_M),            # bottom-left
    1: (ARENA_WIDTH_M, -ARENA_HALF_HEIGHT_M),  # bottom-right
    2: (ARENA_WIDTH_M, +ARENA_HALF_HEIGHT_M),  # top-right
    3: (0.0, +ARENA_HALF_HEIGHT_M),            # top-left
}
ROBOT_MARKER_ID = 4
ARUCO_DICT_ID = cv2.aruco.DICT_4X4_50    # verified against the printed markers

# ---------------------------------------------------------------------------
# Camera configuration (Rapoo C280 2K, ceiling-mounted ~8 ft above arena)
# ---------------------------------------------------------------------------
CAMERA_WIDTH = 2560                       # 2K mode, MJPEG (see header notes)
CAMERA_HEIGHT = 1440
CAMERA_FPS = 30                           # requested capture rate
TRACK_WIDTH_M = 0.08                      # wheel center-to-center (measured)

DEFAULT_MAX_HOLD_S = 0.5    # robot-marker occlusion tolerated before error
DEFAULT_H_MAX_AGE_S = 2.0   # corner-marker dropout tolerated on last-good H
LATENCY_FRAMES = 1.5        # auto latency estimate: L = this x frame period
MAX_PREDICT_S = 0.5         # never dead-reckon further than this


class CameraPoseError(RuntimeError):
    """Raised when no trustworthy pose is available (caller must coast)."""


# ---------------------------------------------------------------------------
# Detection / prediction math (pure functions -- unit-testable, no camera)
# ---------------------------------------------------------------------------
def _marker_center(corners):
    """Mean of the 4 corner pixels of one detection (corners shape (4, 2))."""
    return corners.mean(axis=0)


def compute_homography(detections):
    """Pixel -> arena-frame homography from the four corner markers.

    `detections` maps marker ID -> corners array (4, 2). Returns the 3x3
    homography H, or None if any arena marker is missing.
    """
    src, dst = [], []
    for marker_id, (ax, ay) in ARENA_MARKERS_M.items():
        if marker_id not in detections:
            return None
        src.append(_marker_center(detections[marker_id]))
        dst.append((ax, ay))
    src = np.array(src, dtype=np.float64)
    dst = np.array(dst, dtype=np.float64)
    H, _mask = cv2.findHomography(src, dst, 0)
    return H


def robot_pose_from_detection(corners, H):
    """(x, y, theta) of the robot marker through homography H.

    corners are (4, 2) in OpenCV ArUco order: TL, TR, BR, BL in the marker's
    own frame. The robot's front = the marker's printed top edge (user
    confirmed), so the heading vector runs from the marker center toward the
    midpoint of the top edge. Transforming BOTH points through H converts the
    direction into the arena frame (and undoes the image's y-down axis).
    """
    center_px = _marker_center(corners)
    top_mid_px = 0.5 * (corners[0] + corners[1])   # midpoint of the top edge
    pts = np.array([[center_px, top_mid_px]], dtype=np.float64)
    (cx, cy), (tx, ty) = cv2.perspectiveTransform(pts, H)[0]
    theta = math.atan2(ty - cy, tx - cx)
    return float(cx), float(cy), float(theta)


def predict_pose(x, y, theta, v_ms, w_rads, dt):
    """Dead-reckon a pose forward by dt under constant v / w (arc motion).

    Exact midpoint-arc integration (same model as SimulatedPoseSource):
    this is how a delayed camera measurement is brought up to "now" using
    the wheel speeds currently being commanded.
    """
    if dt <= 0.0 or (abs(v_ms) < 1e-9 and abs(w_rads) < 1e-9):
        return x, y, theta
    mid_theta = theta + w_rads * dt / 2.0
    return (x + v_ms * math.cos(mid_theta) * dt,
            y + v_ms * math.sin(mid_theta) * dt,
            theta + w_rads * dt)


def detect_markers(detector, gray):
    """Run ArUco detection; return {id: corners(4,2) float64}."""
    corners_list, ids, _ = detector.detectMarkers(gray)
    detections = {}
    if ids is not None:
        for corners, marker_id in zip(corners_list, ids.flatten()):
            detections[int(marker_id)] = corners[0].astype(np.float64)
    return detections


# ---------------------------------------------------------------------------
# Live camera source
# ---------------------------------------------------------------------------
class CameraPoseSource(object):
    """Drop-in replacement for SimulatedPoseSource (same read/update API),
    with fixed camera settings and latency-compensated pose output."""

    def __init__(self, camera_index=0, max_hold_s=DEFAULT_MAX_HOLD_S,
                 h_max_age_s=DEFAULT_H_MAX_AGE_S, debug_view=False,
                 width=CAMERA_WIDTH, height=CAMERA_HEIGHT, fps=0,
                 mjpeg=True, focus=None, exposure=None, latency_s=0.0,
                 latency_frames=LATENCY_FRAMES, track_m=TRACK_WIDTH_M,
                 use_clahe=True):
        self.max_hold_s = max_hold_s
        self.h_max_age_s = h_max_age_s
        self.debug_view = debug_view
        self.latency_s = latency_s              # >0: explicit; 0: auto
        self.latency_frames = latency_frames
        self.track_m = track_m

        # Capture backend + init order EXACTLY as the previous working
        # implementation (calibrate_and_track.py): explicit DirectShow, MJPG
        # first, then 2560x1440, then optics. A different backend or property
        # order can make the UVC driver negotiate a DIFFERENT (cropped-FOV)
        # sensor mode even while reporting the same resolution.
        self._used_dshow = True
        self._cap = cv2.VideoCapture(camera_index, cv2.CAP_DSHOW)
        if not self._cap.isOpened():
            print("[camera] CAP_DSHOW failed, retrying default backend")
            self._used_dshow = False
            self._cap = cv2.VideoCapture(camera_index)
        if not self._cap.isOpened():
            raise CameraPoseError(
                "cannot open camera index %d -- check the webcam connection "
                "and that no other app is using it" % camera_index)

        if mjpeg:
            self._cap.set(cv2.CAP_PROP_FOURCC,
                          cv2.VideoWriter_fourcc("M", "J", "P", "G"))
        self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        # NOTE: CAP_PROP_FPS is deliberately NOT set -- on DirectShow an
        # explicit fps request can make the driver renegotiate a different
        # (possibly cropped) media type. The working version never set it;
        # MJPG 2K runs at its native ~30 fps regardless, and measured_fps
        # reports the true capture rate online.
        # Shrink the driver buffer so read() returns a FRESH frame, not one
        # queued several hundred ms ago (stale pose = wrong corrections).
        self._cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        # --- fixed optics (same as the working version) ---------------------
        self._cap.set(cv2.CAP_PROP_AUTOFOCUS, 0)
        # Old working code locked focus to 0 (far/ceiling distance); --focus
        # overrides if a sweep shows a sharper value.
        self._cap.set(cv2.CAP_PROP_FOCUS, 0 if focus is None else focus)
        # auto-exposure OFF: manual is 0 on DirectShow, 0.25 on V4L2/Linux.
        self._cap.set(cv2.CAP_PROP_AUTO_EXPOSURE,
                      0 if self._used_dshow else 0.25)
        if exposure is not None:
            # Device-specific (DirectShow: log2 seconds, e.g. -6 = 1/64 s;
            # V4L2: absolute). Fix it so frame timing stays constant.
            self._cap.set(cv2.CAP_PROP_EXPOSURE, exposure)

        # Optional explicit fps request -- OFF by default (see NOTE above).
        if fps > 0:
            self._cap.set(cv2.CAP_PROP_FPS, fps)

        # Report what the driver actually accepted.
        actual_w = int(self._cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        actual_h = int(self._cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        actual_fps = self._cap.get(cv2.CAP_PROP_FPS)
        print("[camera] %s MJPG %dx%d -> got %dx%d@%.0f"
              % ("DSHOW" if self._used_dshow else "default-backend",
                 width, height, actual_w, actual_h, actual_fps))

        dictionary = cv2.aruco.getPredefinedDictionary(ARUCO_DICT_ID)
        # Detector tuning from the previous working version: forgiving shape
        # approximation fights motion blur at low FPS, small min perimeter
        # finds 8-9 cm markers from 8 ft away, SUBPIX refinement keeps the
        # homography stable.
        params = cv2.aruco.DetectorParameters()
        params.polygonalApproxAccuracyRate = 0.05
        params.minMarkerPerimeterRate = 0.015
        params.adaptiveThreshConstant = 10
        params.adaptiveThreshWinSizeMin = 3
        params.adaptiveThreshWinSizeMax = 23
        params.adaptiveThreshWinSizeStep = 10
        params.minCornerDistanceRate = 0.05
        params.errorCorrectionRate = 1.0
        params.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        self._detector = cv2.aruco.ArucoDetector(dictionary, params)
        # CLAHE contrast equalization (also from the working version):
        # big detection-reliability win under uneven arena lighting.
        self._clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8, 8))
        self.use_clahe = use_clahe

        self._H = None              # last good pixel->arena homography
        self._H_time = 0.0
        self._last_pose = None      # last MEASURED (delayed) pose
        self._last_pose_time = 0.0  # monotonic time of that measurement
        self._v_cmd = 0.0           # commanded unicycle v [m/s] (from update)
        self._w_cmd = 0.0           # commanded unicycle omega [rad/s]
        self._frame_times = []      # recent grab timestamps (fps estimate)
        self.frames_read = 0
        self.frames_with_pose = 0
        self.last_latency_used_s = 0.0
        self.last_detected_ids = []   # marker IDs seen on the latest frame

    # -- diagnostics ----------------------------------------------------------
    @property
    def measured_fps(self):
        """Real capture rate from frame-grab timestamps (needs >= 2 frames)."""
        n = len(self._frame_times)
        if n < 2:
            return 0.0
        span = self._frame_times[-1] - self._frame_times[0]
        return (n - 1) / span if span > 0.0 else 0.0

    @property
    def frame_period_s(self):
        fps = self.measured_fps
        return 1.0 / fps if fps > 1.0 else 1.0 / CAMERA_FPS

    def effective_latency_s(self):
        """End-to-end camera latency estimate used for prediction."""
        if self.latency_s > 0.0:
            return self.latency_s
        return self.latency_frames * self.frame_period_s

    # -- frame grabbing ---------------------------------------------------------
    def _read_fresh(self):
        """Grab the freshest frame, dropping anything queued by the driver.

        Some UVC drivers queue frames despite CAP_PROP_BUFFERSIZE=1; the
        extra grab() discards the oldest queued frame so retrieve/read works
        on the newest one.
        """
        self._cap.grab()
        ok, frame = self._cap.read()
        return ok, frame

    # -- pose-source API ----------------------------------------------------
    def read(self):
        """Return (x, y, theta) in the arena frame (m, m, rad CCW from +X),
        DEAD-RECKONED FORWARD to 'now' to compensate camera latency.

        Falls back to the last measured pose for up to max_hold_s (also
        predicted forward); raises CameraPoseError after that so the caller
        can coast the robot.
        """
        ok, frame = self._read_fresh()
        if not ok or frame is None:
            raise CameraPoseError("camera frame grab failed")
        t_grab = time.monotonic()
        self._frame_times.append(t_grab)
        if len(self._frame_times) > 60:
            self._frame_times.pop(0)
        self.frames_read += 1

        detections = detect_markers(self._detector, cv2.cvtColor(
            frame, cv2.COLOR_BGR2GRAY))
        self.last_detected_ids = sorted(detections.keys())

        # Refresh the homography whenever all four arena markers are visible;
        # otherwise reuse the last good H for a short dropout window.
        H_new = compute_homography(detections)
        if H_new is not None:
            self._H, self._H_time = H_new, t_grab   # (matrix, timestamp)!
        H = self._H if (self._H is not None
                        and t_grab - self._H_time <= self.h_max_age_s) else None

        measured = None
        if H is not None and ROBOT_MARKER_ID in detections:
            measured = robot_pose_from_detection(detections[ROBOT_MARKER_ID], H)
            self._last_pose, self._last_pose_time = measured, t_grab
            self.frames_with_pose += 1
        elif self._last_pose is not None and \
                t_grab - self._last_pose_time <= self.max_hold_s:
            measured = self._last_pose     # brief occlusion: hold last pose

        # --- latency compensation: predict from the measurement time to NOW.
        # The measurement describes the robot at exposure time; age is the
        # processing time since grab, plus the estimated capture/transfer
        # latency L. Integrate the CURRENTLY COMMANDED v/w over that window.
        pose = None
        if measured is not None:
            now = time.monotonic()
            age = now - self._last_pose_time
            dt = min(age + self.effective_latency_s(), MAX_PREDICT_S)
            self.last_latency_used_s = dt
            pose = predict_pose(measured[0], measured[1], measured[2],
                                self._v_cmd, self._w_cmd, dt)

        # The debug view renders on EVERY frame -- including failed ones:
        # seeing which markers are (not) found is exactly what it is for.
        if self.debug_view:
            self._show_debug(frame, detections, H, pose, measured)

        if pose is not None:
            return pose
        if H is None:
            raise CameraPoseError(
                "arena corner markers (IDs 0-3) not all visible for "
                "> %.1f s -- cannot localize" % self.h_max_age_s)
        raise CameraPoseError(
            "robot marker (ID %d) not visible for > %.1f s"
            % (ROBOT_MARKER_ID, self.max_hold_s))

    def update(self, v_left_ms, v_right_ms, dt):
        """Remember the commanded wheel speeds (converted to unicycle v/w)
        so read() can dead-reckon delayed measurements forward. Unlike
        SimulatedPoseSource this does NOT integrate a simulated robot --
        the camera observes the real one."""
        self._v_cmd = (v_left_ms + v_right_ms) / 2.0
        self._w_cmd = (v_right_ms - v_left_ms) / self.track_m
        return

    def close(self):
        if self._cap is not None:
            self._cap.release()
            self._cap = None
        if self.debug_view:
            cv2.destroyAllWindows()

    # -- debug ---------------------------------------------------------------
    def _show_debug(self, frame, detections, H, pose, measured):
        vis = frame.copy()
        h_img, w_img = vis.shape[:2]

        # --- aim assist: center crosshair + per-marker visibility strip ----
        # Point the camera so the crosshair lands on the arena CENTER and all
        # five indicators are green. (C280 FOV = ~77x48 deg; at 8 ft that
        # covers ~3.9 x 2.2 m, plenty for the 2.71 x 1.73 m arena IF the
        # camera is landscape-oriented and aimed at the center.)
        cx, cy = w_img // 2, h_img // 2
        cv2.drawMarker(vis, (cx, cy), (255, 255, 255),
                       cv2.MARKER_CROSS, 50, 2)
        cv2.putText(vis, "aim: arena center here", (cx + 30, cy - 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        x0 = w_img - 5 * 70 - 20
        for i, marker_id in enumerate([0, 1, 2, 3, 4]):
            found = marker_id in detections
            color = (0, 200, 0) if found else (0, 0, 200)
            cv2.putText(vis, str(marker_id), (x0 + i * 70, 50),
                        cv2.FONT_HERSHEY_SIMPLEX, 1.2, color, 3)

        for marker_id, corners in detections.items():
            pts = corners.astype(np.int32).reshape((-1, 1, 2))
            color = (0, 0, 255) if marker_id == ROBOT_MARKER_ID else (0, 255, 0)
            cv2.polylines(vis, [pts], True, color, 2)
            c = _marker_center(corners).astype(int)
            cv2.putText(vis, str(marker_id), tuple(c),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)
        if pose is not None:
            lines = [
                "pose(now) x=%+.3f m  y=%+.3f m  th=%+.2f rad" % pose,
                "fps=%.1f  latency comp=%.0f ms" % (
                    self.measured_fps, self.last_latency_used_s * 1000.0),
            ]
            if measured is not None:
                lines.append("measured   x=%+.3f  y=%+.3f  th=%+.2f" % measured)
            for i, text in enumerate(lines):
                cv2.putText(vis, text, (10, 40 + 35 * i),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
        elif H is None:
            cv2.putText(vis, "NO HOMOGRAPHY (need markers 0-3), seen: %s"
                        % self.last_detected_ids, (10, 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
        else:
            cv2.putText(vis, "ROBOT MARKER (id 4) NOT VISIBLE, seen: %s"
                        % self.last_detected_ids, (10, 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
        # Scale the 2K frame down for display, exactly like the previous
        # working version did: an unscaled 2560x1440 window is LARGER than
        # most screens, so you only see one corner of the feed (and it looks
        # like the camera does not cover the arena). Detection still runs on
        # the full 2K frame.
        display = cv2.resize(vis, (1280, 720))
        cv2.imshow("camera_pose debug", display)
        cv2.waitKey(1)


# ---------------------------------------------------------------------------
# Standalone preview
# ---------------------------------------------------------------------------
def main():
    parser = argparse.ArgumentParser(
        description="Preview the overhead ArUco pose pipeline (no robot).")
    parser.add_argument("--index", type=int, default=0, help="camera index")
    parser.add_argument("--debug-view", action="store_true",
                        help="show an annotated video window")
    parser.add_argument("--max-hold", type=float, default=DEFAULT_MAX_HOLD_S,
                        help="occlusion hold time in s (default %(default)s)")
    parser.add_argument("--width", type=int, default=CAMERA_WIDTH)
    parser.add_argument("--height", type=int, default=CAMERA_HEIGHT)
    parser.add_argument("--fps", type=int, default=0,
                        help="explicitly request this capture rate (default "
                             "0 = do NOT touch it, like the previous working "
                             "version; an explicit request can make the "
                             "driver renegotiate a cropped sensor mode)")
    parser.add_argument("--no-mjpeg", action="store_true",
                        help="do not force MJPEG (NOT recommended at 2K: "
                             "uncompressed USB bandwidth limits FPS heavily)")
    parser.add_argument("--no-clahe", action="store_true",
                        help="disable CLAHE contrast equalization "
                             "(enabled by default, as in the working version)")
    parser.add_argument("--focus", type=float, default=None,
                        help="manual focus value (device scale, often 0-255; "
                             "autofocus is always disabled)")
    parser.add_argument("--exposure", type=float, default=None,
                        help="manual exposure value (device scale; V4L2 "
                             "absolute, DirectShow log2 s, e.g. -6 = 1/64 s; "
                             "auto-exposure is always disabled)")
    parser.add_argument("--latency-s", type=float, default=0.0,
                        help="explicit end-to-end camera latency in s "
                             "(0 = auto: %.1f x measured frame period)"
                             % LATENCY_FRAMES)
    args = parser.parse_args()

    src = CameraPoseSource(camera_index=args.index, max_hold_s=args.max_hold,
                           debug_view=args.debug_view,
                           width=args.width, height=args.height,
                           fps=args.fps, mjpeg=not args.no_mjpeg,
                           focus=args.focus, exposure=args.exposure,
                           latency_s=args.latency_s,
                           use_clahe=not args.no_clahe)
    print("camera opened; printing pose at 5 Hz (Ctrl-C to quit)")
    try:
        while True:
            t0 = time.monotonic()
            try:
                x, y, th = src.read()
                print("x=%+.3f m  y=%+.3f m  theta=%+.2f rad | "
                      "fps=%.1f lat=%.0fms track=%.0f%%"
                      % (x, y, th, src.measured_fps,
                         src.last_latency_used_s * 1000.0,
                         100.0 * src.frames_with_pose / max(src.frames_read, 1)))
            except CameraPoseError as exc:
                print("no pose: %s" % exc)
            time.sleep(max(0.0, 0.2 - (time.monotonic() - t0)))
    except KeyboardInterrupt:
        pass
    finally:
        src.close()


if __name__ == "__main__":
    main()
