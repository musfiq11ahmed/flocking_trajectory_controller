# Plan: Run motor_test over WiFi instead of USB

## Goal
`motor_test.py` currently runs ON the ESP32-S3 via `mpremote` over USB serial.
User wants the same open-loop duty-sweep diagnostic triggered/controlled over WiFi.

## Key design decision
Keep the timing-critical part (PWM duty steps + encoder edge counting) ON the
ESP32 — sending per-step PWM commands over UDP would be jittery and would
distort the diagnostic. WiFi is used only to (a) trigger the test and
(b) stream results back. This mirrors the project's existing UDP-over-WiFi
architecture (port 4210 protocol in robot_firmware.py / udp_protocol.py).

## Stage 1 — ESP32-side script: `motor_test_wifi.py` (MicroPython)
- Connects to WiFi (same credential style as robot_firmware.py)
- UDP listener on port 4211 (separate from the 4210 control protocol)
- Commands from PC: `PING` (link check), `RUN` (full duty sweep, identical
  logic to motor_test.py), `STOP` (immediate coast / abort)
- Each result line of the sweep is sent back to the PC as a UDP text datagram
- Self-terminating sweep + STOP handler = failsafe
- Deployed as main.py *temporarily* (it must own the PWM pins exclusively);
  robot_firmware.py is copied back afterwards

## Stage 2 — PC-side script: `pc/motor_test_wifi.py` (stdlib only)
- argparse CLI matching existing tools (`--ip`), plus `--port`, `--timeout`
- Sends PING → RUN, receives and prints result lines with a receive timeout
- Prints the same PASS/FAIL reading guide as the USB version

## Stage 3 — Validation & docs
- Syntax-check both files (py_compile for PC, ast parse; careful MicroPython
  review for firmware side)
- Usage instructions: one-time USB deploy of motor_test_wifi.py as main.py
  (or Thonny), then everything over WiFi; how to switch back to
  robot_firmware.py; note that test_suite.py Test 1 is the closed-loop
  WiFi equivalent that needs no firmware swap

---

# Plan (Stage 2): Vision-guided S-curve navigation with per-waypoint error compensation

## Requirements analysis
User flow: follow scurve_trajectory.csv; camera tracks real pose via arena
markers 0-3 + robot marker 4; deviations compensated ONLY for the next
waypoint; CSV never modified.

Findings from existing code:
- `run_waypoint_navigation()` (test_suite.py) ALREADY compensates this way:
  control law runs from the MEASURED pose to the CURRENT target waypoint
  every 50 ms, so a B+2 arrival yields a (B+2)->C command automatically.
- GAP 1: `CameraPoseSource` is a NotImplementedError stub -> implement for
  real (new file `pc/camera_pose.py`), per ARENA_GEOMETRY.md spec.
- GAP 2: scurve_trajectory.csv is in the OLD center-origin frame
  (x in [-0.6, +0.6]); the arena frame has origin at the LEFT-edge midpoint.
  Fix at load time with x += 1.35255 m (arena half width); CSV file stays
  byte-identical on disk.
- GAP 3: arrival requires dist < 0.10 m; a systematic offset would stall on
  one waypoint forever. Add optional per-waypoint timeout advance
  (--wp-timeout): on timeout, advance anyway, log residual error, error
  rolls into the next waypoint (exactly the user's A->B+2->C requirement).

## Stage 2.1 — pc/camera_pose.py (new, Mode B per vibecoding-general-swarm)
- OpenCV (opencv-contrib-python) ArUco DICT_4X4_50, cv2.VideoCapture,
  2560x1440 (Rapoo C280), low buffer latency
- Per frame: detect markers; if all of IDs 0-3 visible -> recompute
  homography H (pixel -> arena meters); keep last-good H for brief dropouts
- Robot pose: marker 4 center + top-edge direction through H ->
  (x, y, theta CCW from +X); marker top edge = robot front (confirmed)
- read()/update(v_l, v_r, dt) signatures matching SimulatedPoseSource
  (update = no-op for camera); hold last pose <= 0.5 s, then raise
  CameraPoseError so navigation aborts + robot coasts (never drive blind)
- Standalone preview CLI + optional debug view (annotated frame)

## Stage 2.2 — test_suite.py integration (copy into output, patch)
- `--pose {sim,camera}` (default sim: old behavior untouched)
- Frame offset x += 1.35255 m applied at load for Test 4 (--no-frame-offset
  to disable); waypoint bounds check vs arena (0..2.7051, +/-0.8636 m)
- `--wp-timeout S`: advance on timeout with residual-error log
- `--camera-index`, `--debug-view`
- Lazy import of camera_pose (stdlib-only promise kept for sim mode)

## Stage 2.3 — Validation
- Synthetic vision test: render the 5 markers into a perspective-warped
  synthetic arena image at KNOWN poses, run camera_pose detection math,
  verify recovered (x, y, theta) within tolerance
- py_compile all; sim-mode Test 4 unchanged (default path)

## Stage 2.4 — Docs
- Usage: preview mode, camera nav run, wp-timeout semantics, new dependency
  (opencv-contrib-python + numpy, camera mode only)
