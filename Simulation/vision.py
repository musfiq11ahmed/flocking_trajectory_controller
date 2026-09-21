"""Overhead camera rendering and color-segmentation detection.

The simulated "camera" is a PyBullet overhead RGB render.  Detection is pure
color segmentation in HSV space (cv2):

* anchors 0..3  -> saturated colored plates (red / green / blue / yellow)
* marker 4 (bot)-> white top plate centroid (position reference) + black nose
                   stripe centroid (heading reference)

All detections are returned as PIXEL coordinates; conversion to meters is the
job of ``calibration.HomographyCalibrator``.  Note that image v points DOWN
while world/arena +Y is up, so direction vectors must be formed by
transforming both pixel points through the homography -- never by naive pixel
subtraction.
"""

import cv2
import numpy as np
import pybullet as p

import config as C

# HSV thresholds (OpenCV H in [0,179], S,V in [0,255]).
# Anchor hues are verified against the actual renderer in tests/scratch checks.
_ANCHOR_HSV = {
    0: [((0, 120, 100), (8, 255, 255)), ((172, 120, 100), (179, 255, 255))],  # red
    1: [((45, 100, 80), (80, 255, 255))],                                      # green
    2: [((95, 100, 80), (130, 255, 255))],                                     # blue
    3: [((18, 100, 120), (40, 255, 255))],                                     # yellow
}
_WHITE_SOFT_LO = 180.0          # soft whiteness ramp: min-channel 180..239
_WHITE_SOFT_HI = 239.0
_BLACK_V_MAX = 80.0
_BORDER = 4                     # ignore a 4-px frame border (render artifact)

_MIN_ANCHOR_AREA_PX = 60
_MIN_PLATE_AREA_PX = 40
_MIN_STRIPE_AREA_PX = 6


def rgba_to_rgb(rgba, h, w):
    """p.getCameraImage returns RGBA (a tuple/list); convert to HxWx3 uint8."""
    arr = np.asarray(rgba, dtype=np.uint8).reshape(h, w, 4)
    return arr[:, :, :3].copy()


class VisionSystem:
    """Renders the overhead camera and detects anchors / bot marker pixels."""

    def __init__(self, client, img_w=C.CAM_IMG_W, img_h=C.CAM_IMG_H,
                 cam_z=C.CAM_Z, fov_deg=C.CAM_FOV_DEG):
        self.client = client
        self.img_w = img_w
        self.img_h = img_h
        self.cam_z = cam_z
        self.proj_fov_deg = fov_deg
        self.view = p.computeViewMatrix(
            cameraEyePosition=[0.0, 0.0, cam_z],
            cameraTargetPosition=list(C.CAM_TARGET),
            cameraUpVector=list(C.CAM_UP),
            physicsClientId=client)
        self.proj = p.computeProjectionMatrixFOV(
            fov_deg, img_w / float(img_h), 0.5 * cam_z, 1.5 * cam_z)
        self.frames_grabbed = 0

    # ------------------------------------------------------------------
    # Camera
    # ------------------------------------------------------------------
    def render(self):
        """Grab one fresh frame -> HxWx3 uint8 RGB image."""
        w, h = self.img_w, self.img_h
        res = p.getCameraImage(w, h, self.view, self.proj,
                               renderer=p.ER_TINY_RENDERER,
                               physicsClientId=self.client)
        frame = rgba_to_rgb(res[2], h, w)
        self.frames_grabbed += 1
        return frame

    def expected_pixel(self, world_x, world_y):
        """Project a world (x,y) point to the expected pixel (u,v).

        Only used for sanity checks/tests, never by the control path.
        """
        import math
        half = math.tan(math.radians(self.proj_fov_deg) / 2.0) * self.cam_z
        u = 0.5 * self.img_w * (1.0 + world_x / half)
        v = 0.5 * self.img_h * (1.0 - world_y / half)   # v points down
        return np.array([u, v])

    # ------------------------------------------------------------------
    # Detection helpers
    # ------------------------------------------------------------------
    @staticmethod
    def _centroid(mask):
        """Centroid (u,v) and area IN PIXELS of the mask foreground."""
        m = cv2.moments(mask)
        if m["m00"] <= 0:
            return None, 0.0
        # m00 sums intensities (255 per foreground px) -> divide by 255.
        return (np.array([m["m10"] / m["m00"], m["m01"] / m["m00"]]),
                m["m00"] / 255.0)

    @staticmethod
    def _largest_blob(mask):
        """Keep only the largest connected component of a binary mask."""
        n, labels, stats, _ = cv2.connectedComponentsWithStats(mask)
        if n <= 1:
            return mask
        best = 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))
        return np.where(labels == best, 255, 0).astype(np.uint8)

    def _mask_hsv_ranges(self, hsv, ranges):
        mask = None
        for lo, hi in ranges:
            part = cv2.inRange(hsv, np.array(lo, np.uint8), np.array(hi, np.uint8))
            mask = part if mask is None else cv2.bitwise_or(mask, part)
        return mask

    # ------------------------------------------------------------------
    # Anchors (ids 0..3)
    # ------------------------------------------------------------------
    def detect_anchors(self, frame):
        """-> {anchor_id: centroid_pixel(u,v)} for every anchor found."""
        hsv = cv2.cvtColor(frame, cv2.COLOR_RGB2HSV)
        out = {}
        for aid, ranges in _ANCHOR_HSV.items():
            mask = self._mask_hsv_ranges(hsv, ranges)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN,
                                    np.ones((3, 3), np.uint8))
            mask = self._largest_blob(mask)
            c, area = self._centroid(mask)
            if c is not None and area >= _MIN_ANCHOR_AREA_PX:
                out[aid] = c
        return out

    # ------------------------------------------------------------------
    # Bot marker (marker 4)
    # ------------------------------------------------------------------
    def detect_bot_marker(self, frame):
        """-> dict with pixel-space marker geometry, or None if not visible.

        center_px      = soft (intensity-weighted) centroid of the visible
                         white plate region -> bot position reference
        nose_px        = soft darkness-weighted centroid of the nose stripe
        axis_base_px   = same as center_px (PCA reference point)
        heading_tip_px = center_px + 30 px along the plate PCA major axis,
                         sign-disambiguated toward the nose stripe.
                         axis_base/heading_tip must BOTH go through the full
                         homography point transform to form a world heading
                         (pixel v points down, arena +Y points up).
        plate_area     = hard white area in pixels

        Soft weights (instead of binary masks) make the centroid/PCA nearly
        alias-free, which keeps the heading bias ~1 deg.
        """
        minc = frame.min(axis=2).astype(np.float64)
        maxc = frame.max(axis=2).astype(np.float64)
        # "whiteness": high only for the plate (floor/anchors/chassis/stripe
        # all have a low minimum channel or low intensity).
        whiteness = np.clip((minc - _WHITE_SOFT_LO) / (_WHITE_SOFT_HI - _WHITE_SOFT_LO),
                            0.0, 1.0)
        whiteness[:_BORDER, :] = 0.0
        whiteness[-_BORDER:, :] = 0.0
        whiteness[:, :_BORDER] = 0.0
        whiteness[:, -_BORDER:] = 0.0

        # Hard mask just to LOCALIZE the plate blob.
        hard = np.where(whiteness > 0.5, 255, 0).astype(np.uint8)
        hard = cv2.morphologyEx(hard, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8))
        hard = self._largest_blob(hard)
        _c, area = self._centroid(hard)
        if _c is None or area < _MIN_PLATE_AREA_PX:
            return None

        # Padded ROI around the plate.
        ys, xs = np.nonzero(hard)
        pad = 12
        xa, xb = max(0, xs.min() - pad), min(frame.shape[1], xs.max() + pad + 1)
        ya, yb = max(0, ys.min() - pad), min(frame.shape[0], ys.max() + pad + 1)
        W = whiteness[ya:yb, xa:xb]
        yy, xx = np.mgrid[ya:yb, xa:xb].astype(np.float64)
        ws = W.sum()
        if ws < _MIN_PLATE_AREA_PX * 0.25:
            return None
        cx = float((xx * W).sum() / ws)
        cy = float((yy * W).sum() / ws)
        dx, dy = xx - cx, yy - cy
        cov = np.array([[(W * dx * dx).sum(), (W * dx * dy).sum()],
                        [(W * dx * dy).sum(), (W * dy * dy).sum()]]) / ws
        eigvals, eigvecs = np.linalg.eigh(cov)
        axis = eigvecs[:, int(np.argmax(eigvals))]  # major axis, sign ambiguous

        # Nose stripe: soft darkness inside the plate ROI.
        darkness = np.clip((_BLACK_V_MAX - maxc[ya:yb, xa:xb]) / _BLACK_V_MAX,
                           0.0, 1.0)
        darkness *= (W < 0.5)          # stripe is the dark hole in the plate
        ds = float(darkness.sum())
        if ds < _MIN_STRIPE_AREA_PX * 0.25:
            return None
        nx = float((xx * darkness).sum() / ds)
        ny = float((yy * darkness).sum() / ds)

        # Disambiguate the axis sign: stripe sits at the NOSE (+heading) end.
        if (nx - cx) * axis[0] + (ny - cy) * axis[1] < 0.0:
            axis = -axis
        center = np.array([cx, cy])
        return {
            "center_px": center,
            "nose_px": np.array([nx, ny]),
            "axis_base_px": center,
            "heading_tip_px": center + 30.0 * axis,
            "plate_area": area,
        }

    # ------------------------------------------------------------------
    # Convenience
    # ------------------------------------------------------------------
    def grab_detections(self):
        """Grab one frame and return (frame, anchors, bot_marker)."""
        frame = self.render()
        return frame, self.detect_anchors(frame), self.detect_bot_marker(frame)
