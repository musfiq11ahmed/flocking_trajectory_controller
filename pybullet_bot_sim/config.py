"""Shared geometry / tuning constants for the arena calibrate-align-move sim.

This module is the single source of truth for layout numbers that must agree
between ``scripts/generate_urdf.py`` (which writes the URDF files) and the
runtime modules (``vision.py``, ``calibration.py``, ``control.py``).

Frames
------
* World frame: PyBullet world.  The four anchor plates sit at (+-1.5, +-1.5).
* Arena frame (SPEC): origin at the midpoint of the arena's left edge,
  +X across the arena, +Y toward the top-left anchor.  With the anchors at
  world (+-1.5, +-1.5) the mapping is simply::

      arena_x = world_x + 1.5
      arena_y = world_y

  so the anchors land at arena (0, +-1.5) and (3.0, +-1.5).
"""

import numpy as np

# ---------------------------------------------------------------------------
# Arena layout (world frame, meters)
# ---------------------------------------------------------------------------
ANCHOR_HALF = 1.5                      # anchors at (+-ANCHOR_HALF, +-ANCHOR_HALF)
FLOOR_HALF = 2.3                       # light-gray floor half extent
FLOOR_THICK = 0.02
ANCHOR_PLATE_SIZE = 0.30               # square colored plate side length
ANCHOR_PLATE_THICK = 0.01

# Anchor ids -> (world_x, world_y, urdf_rgb).
#   0 = top-left (red), 1 = top-right (green),
#   2 = bottom-right (blue), 3 = bottom-left (yellow)
# Quad order 0 -> 1 -> 2 -> 3 is convex.
ANCHORS = {
    0: (-ANCHOR_HALF, +ANCHOR_HALF, (0.90, 0.05, 0.05)),   # red
    1: (+ANCHOR_HALF, +ANCHOR_HALF, (0.05, 0.75, 0.10)),   # green
    2: (+ANCHOR_HALF, -ANCHOR_HALF, (0.10, 0.25, 0.95)),   # blue
    3: (-ANCHOR_HALF, -ANCHOR_HALF, (0.95, 0.85, 0.05)),   # yellow
}
ANCHOR_IDS = (0, 1, 2, 3)
BOT_MARKER_ID = 4                       # the bot is "marker 4" per SPEC

FLOOR_RGB = (0.70, 0.70, 0.70)          # light gray, low saturation

# Arena-frame anchor coordinates (SPEC: origin left-edge midpoint,
# +X across arena, +Y toward top-left).
def world_to_arena_xy(x, y):
    return (x + ANCHOR_HALF, y)

ARENA_ANCHORS = {
    aid: world_to_arena_xy(wx, wy) for aid, (wx, wy, _rgb) in ANCHORS.items()
}

# ---------------------------------------------------------------------------
# Bot geometry (body frame of the chassis base link, meters)
# ---------------------------------------------------------------------------
BOT_CHASSIS_SIZE = (0.30, 0.20, 0.08)   # x (forward), y, z
BOT_CHASSIS_RGB = (0.45, 0.45, 0.45)    # neutral gray, does not collide with masks

# Asymmetric white top plate: extends further toward the rear (-X) and toward
# +Y so its centroid is offset from the base origin (heading is unambiguous).
PLATE_X_MIN, PLATE_X_MAX = -0.15, 0.10
PLATE_Y_MIN, PLATE_Y_MAX = -0.05, 0.08
PLATE_THICK = 0.01
PLATE_RGB = (1.0, 1.0, 1.0)

# Black nose stripe near the +X (front) edge of the plate, INSET from all
# plate edges so it is surrounded by white (robust segmentation).
STRIPE_X_MIN, STRIPE_X_MAX = 0.05, 0.09
STRIPE_Y_MIN, STRIPE_Y_MAX = -0.035, 0.035
STRIPE_THICK = 0.006
STRIPE_RGB = (0.02, 0.02, 0.02)

# Centroids in the bot body frame (known constants, used to convert the
# detected plate centroid into the true base position).
PLATE_CENTROID_BODY = np.array([
    0.5 * (PLATE_X_MIN + PLATE_X_MAX),
    0.5 * (PLATE_Y_MIN + PLATE_Y_MAX),
])
STRIPE_CENTROID_BODY = np.array([
    0.5 * (STRIPE_X_MIN + STRIPE_X_MAX),
    0.5 * (STRIPE_Y_MIN + STRIPE_Y_MAX),
])
# The stripe covers part of the plate, so the VISIBLE white region's centroid
# is plate-minus-stripe.  That is what the camera actually segments.
_PLATE_AREA = (PLATE_X_MAX - PLATE_X_MIN) * (PLATE_Y_MAX - PLATE_Y_MIN)
_STRIPE_AREA = (STRIPE_X_MAX - STRIPE_X_MIN) * (STRIPE_Y_MAX - STRIPE_Y_MIN)
PLATE_VISIBLE_CENTROID_BODY = (
    (PLATE_CENTROID_BODY * _PLATE_AREA - STRIPE_CENTROID_BODY * _STRIPE_AREA)
    / (_PLATE_AREA - _STRIPE_AREA))


def _plate_pca_tilt():
    """Body-frame tilt of the visible plate region's PCA major axis.

    The stripe notch makes the visible white region (plate minus stripe)
    slightly skewed, so its PCA major axis is not exactly the body +X axis.
    The tilt is a fixed geometric constant: compute it once on a fine grid.
    """
    g = 0.0005
    xs = np.arange(PLATE_X_MIN, PLATE_X_MAX, g)
    ys = np.arange(PLATE_Y_MIN, PLATE_Y_MAX, g)
    X, Y = np.meshgrid(xs, ys)
    vis = ~((X >= STRIPE_X_MIN) & (X <= STRIPE_X_MAX)
            & (Y >= STRIPE_Y_MIN) & (Y <= STRIPE_Y_MAX))
    px, py = X[vis], Y[vis]
    dx, dy = px - px.mean(), py - py.mean()
    cov = np.array([[np.mean(dx * dx), np.mean(dx * dy)],
                    [np.mean(dx * dy), np.mean(dy * dy)]])
    eigvals, eigvecs = np.linalg.eigh(cov)
    axis = eigvecs[:, int(np.argmax(eigvals))]
    tilt = float(np.arctan2(axis[1], axis[0]))
    # The axis is sign-ambiguous: wrap the tilt into (-pi/2, pi/2].
    return float(np.remainder(tilt + np.pi / 2, np.pi) - np.pi / 2)


PLATE_PCA_TILT_RAD = _plate_pca_tilt()   # ~+0.045 rad; subtract from estimate

BOT_MASS = 1.0
BOT_SPAWN_XY = (-1.0, 1.0)              # random respawn range (keep clear of anchors)

# ---------------------------------------------------------------------------
# Camera
# ---------------------------------------------------------------------------
CAM_IMG_W = 320
CAM_IMG_H = 320
# Quasi-orthographic overhead camera: high + narrow FOV keeps perspective
# distortion (parallax, visible chassis side faces) negligible while staying
# compatible with TinyRenderer (which only supports projective cameras).
CAM_Z = 12.0
CAM_HALF_EXTENT = 2.33                  # world half-extent covered by the image
CAM_FOV_DEG = float(np.degrees(2.0 * np.arctan(CAM_HALF_EXTENT / CAM_Z)))
CAM_TARGET = (0.0, 0.0, 0.0)
CAM_UP = (0.0, 1.0, 0.0)                # image top = world +Y

# ---------------------------------------------------------------------------
# Physics / control timing (SPEC control parameters)
# ---------------------------------------------------------------------------
PHYSICS_DT = 1.0 / 240.0
CMD_PERIOD = 0.05                       # 20 Hz command rate
STEPS_PER_CMD = int(round(CMD_PERIOD / PHYSICS_DT))

ALIGN_TOL_RAD = 0.10
ALIGN_TIMEOUT_S = 20.0
MOVE_TOL_ALONG_M = 0.05
MOVE_TOL_LATERAL_M = 0.06
MOVE_TIMEOUT_S = 30.0

MAX_LIN_SPEED = 0.35                    # m/s (clamped)
MAX_ANG_SPEED = 1.50                    # rad/s (clamped)
MIN_LIN_SPEED = 0.06                    # anti-stall floor (SPEC: clamp, then floor)
MIN_ANG_SPEED = 0.25

EMA_ALPHA = 0.4                         # pose EMA smoothing factor

MOVE_DISTANCE_M = 1.0
TARGET_HEADING = 0.0                    # pivot to theta = 0 rad (+X)
