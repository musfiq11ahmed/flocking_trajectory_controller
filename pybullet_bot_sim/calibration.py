"""Homography calibration: anchor pixels -> arena coordinates (meters).

The calibrator is fed the pixel centroids of anchors 0..3 and their known
arena-frame coordinates (SPEC: origin = left-edge midpoint, +X across the
arena, +Y toward the top-left anchor) and computes a cv2 homography.

Lock semantics (SPEC safety section):
* locking requires ALL FOUR anchors detected in one frame and a sane convex
  quad (order 0 -> 1 -> 2 -> 3);
* once locked the homography is frozen -- anchor dropouts afterwards must
  never invalidate or refresh it.
"""

import cv2
import numpy as np

import config as C


class CalibrationError(Exception):
    pass


# The homography is locked on the anchor plates (top face ~1 cm above the
# floor), but the bot marker sits ~9 cm up.  Under the pinhole camera this
# causes a radial parallax shift about the camera axis that grows with
# distance from the arena center (up to ~2.5 cm at the corners -- too big to
# ignore).  For a pinhole camera the correction is an exact radial rescale.
_CAMERA_CENTER_ARENA = np.array(C.world_to_arena_xy(0.0, 0.0))
_BOT_MARKER_Z = C.BOT_CHASSIS_SIZE[2] + C.PLATE_THICK   # plate top face
_ANCHOR_Z = C.ANCHOR_PLATE_THICK
_PARALLAX_SCALE = ((C.CAM_Z - _BOT_MARKER_Z) / (C.CAM_Z - _ANCHOR_Z))


def _plate_plane_correct(pts_arena):
    """Map anchor-plane homography output to the bot-marker plane."""
    return (_CAMERA_CENTER_ARENA
            + (pts_arena - _CAMERA_CENTER_ARENA) * _PARALLAX_SCALE)


def _is_convex_quad(pts):
    """True if pts (4,2) in order form a convex quad with consistent winding."""
    signs = []
    n = len(pts)
    for i in range(n):
        a = pts[(i + 1) % n] - pts[i]
        b = pts[(i + 2) % n] - pts[(i + 1) % n]
        cross = a[0] * b[1] - a[1] * b[0]
        if abs(cross) < 1e-9:
            return False
        signs.append(np.sign(cross))
    return all(s == signs[0] for s in signs)


class HomographyCalibrator:
    def __init__(self, arena_points=None):
        # anchor id -> (x, y) in arena meters
        self.arena_points = dict(arena_points or C.ARENA_ANCHORS)
        self._H = None
        self.reprojection_err_px = None

    # ------------------------------------------------------------------
    @property
    def homography_locked(self):
        return self._H is not None

    @property
    def H(self):
        return self._H

    # ------------------------------------------------------------------
    def lock(self, anchor_pixels):
        """Compute and freeze H from {anchor_id: pixel(u,v)}.

        Raises CalibrationError unless all four anchors are present and form
        a sane convex quad.
        """
        missing = [a for a in C.ANCHOR_IDS if a not in anchor_pixels]
        if missing:
            raise CalibrationError(
                "cannot lock: anchors %s not all visible in one frame" % missing)
        px = np.array([anchor_pixels[a] for a in C.ANCHOR_IDS], dtype=np.float64)
        if not _is_convex_quad(px):
            raise CalibrationError(
                "cannot lock: anchor pixels do not form a convex quad")
        # Cheap sanity: quad area must be a reasonable fraction of the frame.
        area = 0.5 * abs(np.dot(px[:, 0], np.roll(px[:, 1], -1))
                         - np.dot(px[:, 1], np.roll(px[:, 0], -1)))
        if area < 100.0:
            raise CalibrationError("cannot lock: anchor quad suspiciously small")

        dst = np.array([self.arena_points[a] for a in C.ANCHOR_IDS],
                       dtype=np.float64)
        H, _ = cv2.findHomography(px.reshape(-1, 1, 2), dst.reshape(-1, 1, 2))
        if H is None:
            raise CalibrationError("cv2.findHomography failed")
        self._H = H
        back = self.pixels_to_arena(px)
        self.reprojection_err_px = float(
            np.max(np.linalg.norm(back - dst, axis=1)))
        return H

    def unlock(self):
        self._H = None
        self.reprojection_err_px = None

    # ------------------------------------------------------------------
    def pixel_to_arena(self, uv):
        """(u,v) pixel -> (x,y) arena meters. Requires a locked homography."""
        return self.pixels_to_arena(np.asarray(uv, dtype=np.float64).reshape(1, 2))[0]

    def pixels_to_arena(self, uv_points):
        """(N,2) pixels -> (N,2) arena meters.

        NOTE: always transform POINTS through H (never subtract pixel vectors):
        the image v axis points down while arena +Y is up, and the homography
        absorbs that flip plus the metric scale.
        """
        if self._H is None:
            raise CalibrationError("homography not locked")
        pts = np.asarray(uv_points, dtype=np.float64).reshape(-1, 1, 2)
        out = cv2.perspectiveTransform(pts, self._H)
        return out.reshape(-1, 2)

    # ------------------------------------------------------------------
    def bot_pose_from_marker(self, marker):
        """Camera-only bot pose in the arena frame.

        marker = output of vision.detect_bot_marker().
        Returns ((x, y), theta) where (x, y) is the estimated CHASSIS CENTER
        (the known body-frame offset of the visible plate centroid is
        removed) and theta is the body heading.

        The heading comes from the bot silhouette's PCA major axis
        (disambiguated by the nose stripe in vision.py).  BOTH pixel points
        (axis base and heading tip) go through the full homography point
        transform -- never subtract pixel vectors directly, because the
        image v axis points down while arena +Y points up.
        """
        pts = _plate_plane_correct(self.pixels_to_arena(
            [marker["center_px"], marker["axis_base_px"],
             marker["heading_tip_px"]]))
        center_arena, tail_arena, tip_arena = pts
        d = tip_arena - tail_arena
        theta = float(np.arctan2(d[1], d[0]))
        # Remove the fixed body-frame tilt of the visible plate region's PCA
        # axis (the stripe notch skews it slightly off the body +X axis).
        theta -= C.PLATE_PCA_TILT_RAD
        # Remove the known visible-plate-centroid offset to get base origin.
        rot = np.array([[np.cos(theta), -np.sin(theta)],
                        [np.sin(theta), np.cos(theta)]])
        base_arena = center_arena - rot @ C.PLATE_VISIBLE_CENTROID_BODY
        return (float(base_arena[0]), float(base_arena[1])), theta
