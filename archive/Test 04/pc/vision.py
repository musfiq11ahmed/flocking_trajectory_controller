"""
vision.py — ArUco detection, arena calibration (homography), and robot localisation.

Workflow
--------
1. Detect four fixed ArUco markers at known arena corners → compute homography.
2. Detect the robot's ArUco marker → transform its centre and orientation
   through the homography to obtain (x, y, θ) in world metres.
3. Provide an overlay for the live OpenCV window.
"""

import math
import time
import cv2
import numpy as np

from config import (
    ARUCO_DICT_ID, ARENA_MARKER_IDS, ROBOT_MARKER_ID,
    ARENA_MARKERS, CAMERA_INDEX, CAMERA_WIDTH, CAMERA_HEIGHT,
    EMA_ALPHA_POS, EMA_ALPHA_THETA,
)
from utils import normalize_angle


# ─────────────────────────────────────────────────────────────
#  ArUco helpers
# ─────────────────────────────────────────────────────────────

def _get_aruco_dict():
    """Return the OpenCV ArUco dictionary object."""
    name = getattr(cv2.aruco, ARUCO_DICT_ID)
    return cv2.aruco.getPredefinedDictionary(name)


def _get_detector():
    """Create an ArUco detector with the configured dictionary."""
    aruco_dict = _get_aruco_dict()
    params = cv2.aruco.DetectorParameters()
    # Improve detection at distance / low resolution
    params.adaptiveThreshWinSizeMin  = 3
    params.adaptiveThreshWinSizeMax  = 23
    params.adaptiveThreshWinSizeStep = 4
    params.cornerRefinementMethod    = cv2.aruco.CORNER_REFINE_SUBPIX
    return cv2.aruco.ArucoDetector(aruco_dict, params)


# ─────────────────────────────────────────────────────────────
#  Arena Localiser
# ─────────────────────────────────────────────────────────────

class ArenaLocalizer:
    """Detects ArUco markers and converts pixel coords → world coords."""

    def __init__(self):
        self.detector = _get_detector()
        self.homography = None          # 3×3 pixel → world
        self.arena_calibrated = False

        # Smoothed robot state
        self._robot_x     = None
        self._robot_y     = None
        self._robot_theta = None
        self._last_detect = 0.0         # timestamp of last successful detection

    # ── camera ────────────────────────────────────────────────

    @staticmethod
    def open_camera():
        """Open the webcam and set resolution."""
        cap = cv2.VideoCapture(CAMERA_INDEX, cv2.CAP_DSHOW)
        if not cap.isOpened():
            # Try without DirectShow backend
            cap = cv2.VideoCapture(CAMERA_INDEX)
        if not cap.isOpened():
            raise RuntimeError(f"Cannot open camera index {CAMERA_INDEX}")
        cap.set(cv2.CAP_PROP_FRAME_WIDTH,  CAMERA_WIDTH)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT)
        cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)   # reduce latency
        return cap

    # ── calibration ───────────────────────────────────────────

    def calibrate(self, frame) -> bool:
        """Try to detect the four arena-corner markers and compute the
        homography.  Returns True once calibration succeeds."""
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners_list, ids, _ = self.detector.detectMarkers(gray)

        if ids is None:
            return False

        ids_flat = ids.flatten().tolist()
        src_pts = []   # pixel centres (ordered by ARENA_MARKER_IDS)
        dst_pts = []   # world positions

        for mid in ARENA_MARKER_IDS:
            if mid not in ids_flat:
                return False
            idx = ids_flat.index(mid)
            centre = corners_list[idx][0].mean(axis=0)  # mean of 4 corners
            src_pts.append(centre)
            dst_pts.append(ARENA_MARKERS[mid])

        src = np.array(src_pts, dtype=np.float32)
        dst = np.array(dst_pts, dtype=np.float32)

        H, status = cv2.findHomography(src, dst)
        if H is not None:
            self.homography = H
            self.arena_calibrated = True
        return self.arena_calibrated

    # ── robot tracking ────────────────────────────────────────

    def detect_robot(self, frame):
        """Return (x, y, θ) of the robot in world coordinates, or None.

        Heading is derived from the ArUco marker's orientation:
        the direction from the bottom-edge midpoint toward the top-edge
        midpoint (corners 0-1) is taken as the robot's forward axis.

        Mount the marker so that the *top edge* (side between corner 0
        and corner 1) faces the **front** of the robot.
        """
        if not self.arena_calibrated:
            return None

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners_list, ids, _ = self.detector.detectMarkers(gray)

        if ids is None:
            return self._smoothed_state()

        ids_flat = ids.flatten().tolist()
        if ROBOT_MARKER_ID not in ids_flat:
            return self._smoothed_state()

        idx = ids_flat.index(ROBOT_MARKER_ID)
        c = corners_list[idx][0]          # shape (4, 2)

        # Pixel centre
        centre_px = c.mean(axis=0)

        # Pixel "front" midpoint  (top edge = corners 0→1)
        front_px = (c[0] + c[1]) / 2.0

        # Transform both points to world coords via homography
        centre_world = self._px_to_world(centre_px)
        front_world  = self._px_to_world(front_px)

        if centre_world is None or front_world is None:
            return self._smoothed_state()

        x = centre_world[0]
        y = centre_world[1]
        theta = math.atan2(front_world[1] - y, front_world[0] - x)

        # Apply exponential-moving-average smoothing
        if self._robot_x is None:
            self._robot_x     = x
            self._robot_y     = y
            self._robot_theta = theta
        else:
            a_p = EMA_ALPHA_POS
            a_t = EMA_ALPHA_THETA
            self._robot_x     = a_p * x + (1 - a_p) * self._robot_x
            self._robot_y     = a_p * y + (1 - a_p) * self._robot_y
            # Smooth theta while handling wrap-around
            diff = normalize_angle(theta - self._robot_theta)
            self._robot_theta = normalize_angle(self._robot_theta + a_t * diff)

        self._last_detect = time.time()
        return (self._robot_x, self._robot_y, self._robot_theta)

    def time_since_detection(self) -> float:
        """Seconds since the robot marker was last seen."""
        if self._last_detect == 0:
            return float("inf")
        return time.time() - self._last_detect

    # ── internal ──────────────────────────────────────────────

    def _px_to_world(self, px_point):
        """Project a single pixel point through the homography → world (x, y)."""
        pt = np.array([[[px_point[0], px_point[1]]]], dtype=np.float32)
        world = cv2.perspectiveTransform(pt, self.homography)
        return world[0, 0]                # (x, y) numpy array

    def _smoothed_state(self):
        """Return the last-known smoothed state (or None if never seen)."""
        if self._robot_x is not None:
            return (self._robot_x, self._robot_y, self._robot_theta)
        return None

    # ── visualisation ─────────────────────────────────────────

    def draw_overlay(self, frame, corners_list=None, ids=None,
                     robot_pose=None, trajectory=None, target_idx=None,
                     lookahead_pt=None, state_text=""):
        """Draw a rich debug overlay on *frame* (modified in-place)."""
        vis = frame.copy()
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Re-detect markers for drawing (cheap)
        cl, det_ids, _ = self.detector.detectMarkers(gray)
        if det_ids is not None:
            cv2.aruco.drawDetectedMarkers(vis, cl, det_ids)

        # Arena boundary (if calibrated)
        if self.arena_calibrated:
            arena_px = []
            for mid in ARENA_MARKER_IDS:
                wp = np.array(ARENA_MARKERS[mid], dtype=np.float32)
                pp = self._world_to_px(wp)
                if pp is not None:
                    arena_px.append(pp.astype(int))
            if len(arena_px) == 4:
                pts = np.array(arena_px).reshape(-1, 1, 2)
                cv2.polylines(vis, [pts], True, (0, 255, 0), 2)

        # Trajectory path
        if trajectory is not None and self.arena_calibrated:
            path_pts = []
            for wp in trajectory:
                pp = self._world_to_px(np.array([wp[0], wp[1]], dtype=np.float32))
                if pp is not None:
                    path_pts.append(pp.astype(int))
            if len(path_pts) > 1:
                cv2.polylines(vis, [np.array(path_pts)], False, (255, 180, 0), 2)

        # Target waypoint
        if target_idx is not None and trajectory is not None:
            wp = trajectory[target_idx]
            pp = self._world_to_px(np.array([wp[0], wp[1]], dtype=np.float32))
            if pp is not None:
                cv2.circle(vis, tuple(pp.astype(int)), 8, (0, 255, 255), -1)

        # Lookahead point
        if lookahead_pt is not None:
            pp = self._world_to_px(np.array(lookahead_pt[:2], dtype=np.float32))
            if pp is not None:
                cv2.circle(vis, tuple(pp.astype(int)), 6, (0, 200, 255), 2)

        # Robot position + heading arrow
        if robot_pose is not None:
            rx, ry, rt = robot_pose
            pp = self._world_to_px(np.array([rx, ry], dtype=np.float32))
            if pp is not None:
                pt = tuple(pp.astype(int))
                cv2.circle(vis, pt, 10, (0, 0, 255), -1)
                # heading arrow (40 px long)
                tip = self._world_to_px(
                    np.array([rx + 0.06 * math.cos(rt),
                              ry + 0.06 * math.sin(rt)], dtype=np.float32))
                if tip is not None:
                    cv2.arrowedLine(vis, pt, tuple(tip.astype(int)),
                                   (0, 0, 255), 2, tipLength=0.3)

        # Status text block (top-left)
        y0 = 30
        lines = state_text.split("\n") if state_text else []
        for i, line in enumerate(lines):
            cv2.putText(vis, line, (10, y0 + i * 24),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
            cv2.putText(vis, line, (10, y0 + i * 24),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 1)

        return vis

    def _world_to_px(self, world_pt):
        """Inverse homography: world (x,y) → pixel (u,v)."""
        if self.homography is None:
            return None
        H_inv = np.linalg.inv(self.homography)
        pt = np.array([[[world_pt[0], world_pt[1]]]], dtype=np.float32)
        px = cv2.perspectiveTransform(pt, H_inv)
        return px[0, 0]
