# Arena Calibrate, Align & Move 1 m (PyBullet sim)

A PyBullet simulation of the "calibrate, align, move 1 m" workflow: an
overhead camera watches a light-gray arena with four colored anchor plates;
the operator locks a pixel→meters homography, spawns a bot at a random pose,
and the bot pivots to heading θ = 0 (+X) and drives forward 1 m using **only
camera-derived feedback** (never ground truth).

## Install

```bash
pip install pybullet numpy opencv-python-headless
python3 scripts/generate_urdf.py   # regenerate urdf/ (already committed)
```

## Run

```bash
python3 main.py --headless      # DIRECT physics; auto-drives when stdin is closed
printf 'c\n' | python3 main.py --headless   # lock via the real key path, then auto-run
```

Interactive terminal keys:

| key   | action                                                        |
|-------|---------------------------------------------------------------|
| `c`   | lock homography (needs all 4 anchors visible in one frame)    |
| `r`   | (re)spawn the bot at a random pose                            |
| Enter/`s` | start: report detected pose, align to θ=0, drive 1 m      |
| `q`   | coast (0,0) and quit (also aborts align/move mid-sequence)    |

## Test

```bash
python3 tests/test_sim.py    # N=5 random-pose trials, 5 cm / 5 deg, < 60 s
```

## How it works

* **`config.py`** — single source of truth for arena/bot geometry, camera,
  and SPEC control parameters (20 Hz command rate, 0.10 rad align tolerance,
  0.05 m / 0.06 m move tolerances, 20 s / 30 s timeouts, speed clamps +
  anti-stall floors).
* **`scripts/generate_urdf.py`** — writes `urdf/arena.urdf` (floor + 4
  saturated anchor plates at world (±1.5, ±1.5); ids 0=red TL, 1=green TR,
  2=blue BR, 3=yellow BL) and `urdf/bot.urdf` (gray chassis + asymmetric
  white top plate + black nose stripe = "marker 4").
* **`vision.py`** — quasi-overhead camera (high camera + narrow FOV to keep
  perspective effects negligible; TinyRenderer is perspective-only).
  Anchors are HSV color-segmented.  The bot marker uses *soft*
  intensity-weighted statistics: a whiteness-weighted centroid (position)
  and whiteness-weighted PCA major axis (heading axis), sign-disambiguated
  by a darkness-weighted stripe centroid.  Soft weights keep the heading
  bias ~1° (binary masks had 8-24° rasterization bias).
* **`calibration.py`** — `HomographyCalibrator`: `cv2.findHomography` from
  anchor pixels to arena meters (origin = left-edge midpoint, +X across the
  arena, +Y toward top-left; anchors map to (0,±1.5) and (3.0,±1.5)).
  Locking requires all four anchors forming a convex quad; while locked the
  homography is frozen (anchor dropouts cannot invalidate it).
  `bot_pose_from_marker()` pushes BOTH pixel points of the heading vector
  through the homography (pixel v points down, arena +Y up — never subtract
  pixel vectors), removes the fixed plate-PCA tilt constant and the known
  body-frame offset of the visible plate centroid, and corrects the small
  residual perspective parallax (radial rescale about the camera axis).
* **`control.py`** — `CameraPoseSource` chains render → detect → homography
  → EMA (circular for heading).  `BotController.align_to_heading()` pivots
  with P control on the atan2-wrapped heading error;
  `move_forward_distance()` tracks `start + (1 m, 0)` with along-track P
  control plus heading/lateral correction.  Speeds are clamped then
  anti-stall-floored (SPEC shaping); velocity is applied every physics step
  along the current *camera-estimated* heading (zero lateral component by
  construction).  Every exit path (success, abort, timeout, pose loss)
  coasts: command (0,0) for ≥ 0.5 s.
* **`main.py`** — terminal state machine (`WAIT_ANCHORS → AWAIT_BOT →
  RUNNING → DONE`) with non-blocking stdin keys (POSIX termios/select,
  Windows msvcrt).  `App.handle_key(ch)` is public for tests.  In
  `--headless` mode with stdin closed it auto-drives the whole sequence
  once and exits 0.

## Notes / deviations from SPEC

* SPEC describes the real robot (`pc/camera_pose.py` ArUco pipeline,
  firmware RPM commands).  This repo is the PyBullet adaptation per the
  project mission: ArUco markers ↔ colored plates (ids 0-3) + plate/stripe
  bot marker (id 4); wheel RPM commands ↔ base velocity commands; the
  workflow, lock semantics, command rate, tolerances, timeouts, and
  coast-on-abort safety contract are carried over unchanged.
* The move controller accepts convergence at 70 % of the SPEC tolerances to
  leave margin for camera-estimation bias; reported/allowed tolerances are
  the SPEC values (0.05 m along-track, 0.06 m lateral).
