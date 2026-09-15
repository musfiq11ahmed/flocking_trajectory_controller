# Vision-Guided S-Curve Navigation — Usage

Closed-loop trajectory following: the overhead Rapoo C280 webcam tracks the
robot's real pose (ArUco markers 0–3 = arena reference, marker 4 = robot),
and the PC continuously corrects toward the **next** waypoint of
`scurve_trajectory.csv`. The CSV is never modified.

## How error compensation works (A → B+2 → C)

Every 50 ms the control law computes wheel speeds from the **measured** pose
toward the **current target waypoint**. So if the robot lands at B+2 instead
of B, the very next command is already (B+2) → C — the offset is absorbed by
the next leg automatically. Two mechanisms complete the behavior:

- **Arrival tolerance** (0.10 m): when close enough to B, the target advances
  to C and any residual error is carried into the B→C leg.
- **`--wp-timeout S`** (recommended ~8 s): if a waypoint is never reached
  exactly (slip, obstacle, systematic offset), the run does not stall — it
  advances anyway, logs the residual error, and the miss is compensated on
  the way to the next point.

## One-time setup

```bash
pip install opencv-contrib-python numpy   # camera mode only; sim is stdlib-only
```

1. Print markers 0–3 (DICT_4X4_50) at the arena corners per their filenames,
   marker 4 on the robot with its printed top edge facing the FRONT.
2. Robot runs the normal `robot_firmware.py` as `main.py` (PID firmware),
   PC and robot on the same 2.4 GHz network.

## Step 1 — validate the camera alone (no robot needed)

```bash
python3 camera_pose.py --index 0 --debug-view
```
Prints the live pose at 5 Hz plus a tracking-rate percentage. Move the robot
by hand and sanity-check coordinates against a tape measure. `--debug-view`
opens an annotated window (green = arena markers, red = robot marker).

## Step 2 — dry-run the full pipeline in simulation

```bash
python3 test_suite.py --test 4 --ip <robot_ip>            # --pose sim (default)
```
Unchanged from before; the robot (if connected) follows simulated commands.

## Step 3 — real camera-guided run

```bash
python3 test_suite.py --test 4 --pose camera --ip <robot_ip> \
    --wp-timeout 8 --debug-view
```

What happens:
1. `scurve_trajectory.csv` loads; waypoints shift x += 1.35255 m into the
   arena frame (in-memory only — the file on disk is untouched), with an
   out-of-bounds warning if anything falls outside the arena.
2. **HOMING (default)**: wherever the robot was left in the arena, it first
   drives to the trajectory origin (waypoint #0) in two stages —
   (a) *approach* until within 0.15 m, then (b) *align*: stop translating and
   pivot in place to the origin heading (the 0.15 m/s anti-stall floor means
   it can't park dead-on, so position and heading are handled separately).
   Homing is never skipped by `--wp-timeout`; only `--home-timeout S`
   (~20 s recommended) aborts + coasts if the origin is unreachable.
   Disable with `--no-home-first` (legacy behavior: waypoint #0 is just the
   first target).
3. **PATH**: 20 Hz loop starting at waypoint #1: camera pose → EMA filter →
   unicycle control law → friction compensation → 0.30 m/s clamp → RPM → UDP.
4. Each waypoint: advance when within 0.10 m, or on `--wp-timeout` (logs the
   residual). At the end the robot is commanded to coast.

Safety: if the robot marker is lost for > 0.5 s, or the arena markers for
> 2 s, navigation ABORTS and the robot is explicitly coasted (the firmware's
500 ms failsafe is the backup).

## Files

| File | Role |
|---|---|
| `camera_pose.py` | CameraPoseSource: ArUco detection + per-frame homography + pose; standalone preview CLI |
| `test_suite.py`  | Test 4 now accepts `--pose sim\|camera`, `--wp-timeout`, `--camera-index`, `--debug-view`, `--no-frame-offset` |
| `../ARENA_GEOMETRY.md` | Coordinate system, marker map, dimensions (confirmed spec) |

Validation performed: synthetic overhead-view images with markers rendered
at known ground-truth poses recovered position to < 1.1 mm and heading to
< 0.9°; timeout-advance logic verified with an unreachable waypoint.
