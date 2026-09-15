#!/usr/bin/env python3
"""Overhead-camera ArUco pose source for the ESP32-S3 diff-drive bot.

Implements the real CameraPoseSource (formerly a stub in test_suite.py):
the Rapoo C280 2K webcam watches the arena from above; the four arena
corner markers (IDs 0-3) define a per-frame homography from pixels to the
arena coordinate frame, and the robot marker (ID 4) yields (x, y, theta).

Arena frame (see ARENA_GEOMETRY.md -- confirmed with the user):
  origin (0, 0)   : midpoint of the LEFT arena edge (between markers 3 and 0)
  +X              : across the arena to the right-edge midpoint (106.5, 0)
  +Y              : toward the top-left marker (+34 in); -Y toward bottom-left
  units           : METERS (matches the rest of pc/)
  theta           : CCW from +X; marker 4's printed top edge = robot front,
                    so marker orientation IS the heading (no offset)

Drop-in for SimulatedPoseSource in run_waypoint_navigation():
  pose = source.read()            # -> (x_m, y_m, theta_rad)
  source.update(v_l, v_r, dt)     # no-op: the camera observes, it does not
                                  # integrate commands

Error handling policy: read() returns the last valid pose for up to
--max-hold seconds of robot-marker occlusion (smooths single-frame dropouts);
beyond that it raises CameraPoseError so the navigation loop can coast the
robot instead of driving blind.

Requires: opencv-contrib-python (cv2.aruco) and numpy -- camera mode only;
test_suite.py imports this module lazily so sim mode stays stdlib-only.

Standalone preview (no robot needed):
    python3 camera_pose.py --index 0 --debug-view
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

CAMERA_WIDTH = 2560                       # Rapoo C280 "2K" native mode
CAMERA_HEIGHT = 1440

DEFAULT_MAX_HOLD_S = 0.5    # robot-marker occlusion tolerated before error
DEFAULT_H_MAX_AGE_S = 2.0   # corner-marker dropout tolerated on last-good H


class CameraPoseError(RuntimeError):
    """Raised when no trustworthy pose is available (caller must coast)."""


# ---------------------------------------------------------------------------
# Detection math (pure functions -- unit-testable without a camera)
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
    """Drop-in replacement for SimulatedPoseSource (same read/update API)."""

    def __init__(self, camera_index=0, max_hold_s=DEFAULT_MAX_HOLD_S,
                 h_max_age_s=DEFAULT_H_MAX_AGE_S, debug_view=False,
                 width=CAMERA_WIDTH, height=CAMERA_HEIGHT):
        self.max_hold_s = max_hold_s
        self.h_max_age_s = h_max_age_s
        self.debug_view = debug_view

        self._cap = cv2.VideoCapture(camera_index)
        if not self._cap.isOpened():
            raise CameraPoseError(
                "cannot open camera index %d -- check the webcam connection "
                "and that no other app is using it" % camera_index)
        self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        # Shrink the driver buffer so read() returns a FRESH frame, not one
        # queued several hundred ms ago (stale pose = wrong corrections).
        self._cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        dictionary = cv2.aruco.getPredefinedDictionary(ARUCO_DICT_ID)
        self._detector = cv2.aruco.ArucoDetector(
            dictionary, cv2.aruco.DetectorParameters())

        self._H = None              # last good pixel->arena homography
        self._H_time = 0.0
        self._last_pose = None      # (x, y, theta)
        self._last_pose_time = 0.0
        self.frames_read = 0
        self.frames_with_pose = 0

    # -- pose-source API ----------------------------------------------------
    def read(self):
        """Return (x, y, theta) in the arena frame (m, m, rad CCW from +X).

        Falls back to the last valid pose for up to max_hold_s; raises
        CameraPoseError after that so the caller can coast the robot.
        """
        ok, frame = self._cap.read()
        if not ok or frame is None:
            raise CameraPoseError("camera frame grab failed")
        self.frames_read += 1

        detections = detect_markers(self._detector, cv2.cvtColor(
            frame, cv2.COLOR_BGR2GRAY))
        now = time.monotonic()

        # Refresh the homography whenever all four arena markers are visible;
        # otherwise reuse the last good H for a short dropout window.
        H_new = compute_homography(detections)
        if H_new is not None:
            self._H, self._H_time = H_new, now
        H = self._H if (self._H is not None
                        and now - self._H_time <= self.h_max_age_s) else None

        pose = None
        if H is not None and ROBOT_MARKER_ID in detections:
            pose = robot_pose_from_detection(detections[ROBOT_MARKER_ID], H)
            self._last_pose, self._last_pose_time = pose, now
            self.frames_with_pose += 1

        if self.debug_view:
            self._show_debug(frame, detections, H, pose)

        if pose is not None:
            return pose
        if self._last_pose is not None and \
                now - self._last_pose_time <= self.max_hold_s:
            return self._last_pose        # brief occlusion: hold last pose
        if H is None:
            raise CameraPoseError(
                "arena corner markers (IDs 0-3) not all visible for "
                "> %.1f s -- cannot localize" % self.h_max_age_s)
        raise CameraPoseError(
            "robot marker (ID %d) not visible for > %.1f s"
            % (ROBOT_MARKER_ID, self.max_hold_s))

    def update(self, v_left_ms, v_right_ms, dt):
        """No-op: the camera observes the real robot; it does not integrate
        commanded speeds. Present for API compatibility with
        SimulatedPoseSource."""
        return

    def close(self):
        if self._cap is not None:
            self._cap.release()
            self._cap = None
        if self.debug_view:
            cv2.destroyAllWindows()

    # -- debug ---------------------------------------------------------------
    def _show_debug(self, frame, detections, H, pose):
        vis = frame.copy()
        for marker_id, corners in detections.items():
            pts = corners.astype(np.int32).reshape((-1, 1, 2))
            color = (0, 0, 255) if marker_id == ROBOT_MARKER_ID else (0, 255, 0)
            cv2.polylines(vis, [pts], True, color, 2)
            c = _marker_center(corners).astype(int)
            cv2.putText(vis, str(marker_id), tuple(c),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2)
        if pose is not None:
            cv2.putText(vis,
                        "pose x=%+.3f m  y=%+.3f m  th=%+.2f rad"
                        % pose, (10, 40), cv2.FONT_HERSHEY_SIMPLEX, 0.9,
                        (255, 255, 255), 2)
        elif H is None:
            cv2.putText(vis, "NO HOMOGRAPHY (need markers 0-3)", (10, 40),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 2)
        cv2.imshow("camera_pose debug", vis)
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
    args = parser.parse_args()

    src = CameraPoseSource(camera_index=args.index, max_hold_s=args.max_hold,
                           debug_view=args.debug_view)
    print("camera opened; printing pose at 5 Hz (Ctrl-C to quit)")
    try:
        while True:
            t0 = time.monotonic()
            try:
                x, y, th = src.read()
                print("x=%+.3f m  y=%+.3f m  theta=%+.2f rad  "
                      "(track rate %.0f%%)"
                      % (x, y, th,
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
