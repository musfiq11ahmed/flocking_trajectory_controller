# Arena Geometry & Vision Setup — Confirmed Spec

Single source of truth for the camera pose pipeline (to be implemented later,
plugs into the `CameraPoseSource` stub in `pc/test_suite.py`, Test 4).

## ArUco markers (verified by decoding the uploaded images)

- Dictionary: **DICT_4X4_50** (all five decode cleanly; IDs match filenames)

| ID | Location                | Role                          |
|----|-------------------------|-------------------------------|
| 0  | Arena bottom-left corner | Homography reference         |
| 1  | Arena bottom-right corner| Homography reference         |
| 2  | Arena top-right corner   | Homography reference         |
| 3  | Arena top-left corner    | Homography reference         |
| 4  | On the robot             | Pose (position + heading)    |

## Coordinate system (arena frame)

- **Origin (0, 0):** midpoint of the LEFT arena edge — midway between marker 3
  (top-left) and marker 0 (bottom-left).
- **+Y:** toward the top-left marker; **−Y:** toward the bottom-left marker.
- **+X:** straight across the arena toward the RIGHT edge; the point
  **(106.5, 0)** is the midpoint between marker 2 (top-right) and marker 1
  (bottom-right).
- Units: inches measured on the arena; software works in **meters**
  (consistent with the rest of `pc/` code).

## Dimensions & marker coordinates (marker-center to marker-center)

- Width (TL→TR / left edge→right edge): **106.5 in = 2.7051 m**
- Height (TL→BL): **68.0 in = 1.7272 m** (half-height 34 in = 0.8636 m)

| ID | (x, y) in        | (x, y) m             |
|----|------------------|----------------------|
| 3  | (0, +34.0)       | (0, +0.8636)         |
| 0  | (0, −34.0)       | (0, −0.8636)         |
| 2  | (106.5, +34.0)   | (2.7051, +0.8636)    |
| 1  | (106.5, −34.0)   | (2.7051, −0.8636)    |

## Robot heading convention

- Marker 4 is mounted with the printed marker's **top edge pointing toward
  the robot's front** (drive direction) → heading θ = marker orientation
  directly, **no mounting offset**.
- θ measured **CCW from the +X axis** (standard math convention, matches
  `wrap_pi()` in `test_suite.py`).

## Camera

- **Rapoo C280 2K webcam (2560×1440)**, mounted above the arena, aimed at the
  arena center → near-planar top-down view.
- Model: per-frame **homography** H (pixel → arena plane) from the four corner
  marker centers (IDs 0–3); then marker 4's center → (x, y), marker 4's
  orientation → θ. Optional full intrinsic (checkerboard) calibration can be
  added later for edge-distortion accuracy.

## Pipeline (planned)

frame → detect markers (DICT_4X4_50) → H from IDs 0–3 → marker 4 → (x, y, θ)
→ EMA filter (α = 0.5, already in test_suite.py) → unicycle controller
→ 20 Hz UDP commands to the ESP32-S3.
