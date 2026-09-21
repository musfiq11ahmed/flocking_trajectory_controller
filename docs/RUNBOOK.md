# RUNBOOK — ESP32-S3 Diff-Drive Bot: From Zero to Vision-Guided S-Curve

Complete step-by-step procedure. Use these canonical files (ignore older
duplicates elsewhere in the folder):

```
firmware/robot_firmware.py      -> deploy to ESP32 as main.py (PID + UDP)
pc/udp_protocol.py              -> shared protocol (no need to run)
pc/test_suite.py                -> tests 0-5 + camera navigation
pc/camera_pose.py               -> overhead ArUco pose source
pc/find_bot.py                  -> discover the bot IP over WiFi
pc/arena_calibrate_move.py      -> CMD workflow: lock arena, align +X, move 1 m
```

Note: older notes may mention `pc/motor_test_pc.py` or `firmware/motor_test_esp32.py`.
Those WiFi motor-diagnostic files are NOT part of this cleaned package. Use
`pc/find_bot.py` for discovery and `pc/test_suite.py` tests 0-5 for link/motor
checks with the normal PID firmware.

---

## PART A — One-time setup

### A1. PC software
```bash
pip install esptool mpremote
pip install opencv-contrib-python numpy     # camera mode only
```
(Python 3.9+ recommended. Everything else is stdlib.)

### A2. Flash MicroPython on the ESP32-S3 (once per board)
Download the `.bin` from https://micropython.org/download/ESP32_GENERIC_S3/
then, with the bot connected by USB:
```bash
esptool.py --chip esp32s3 --port COM13 erase_flash
esptool.py --chip esp32s3 --port COM13 write_flash -z 0x0 ESP32_GENERIC_S3-<version>.bin
```
(Use your actual port: COMx on Windows, /dev/ttyUSB0 or /dev/ttyACM0 on Linux.)

### A3. Wiring check
Verify against README.md section 1: DRV8833 VM from the 5 V buck (NOT from
3V3), STBY hardwired to 3V3, common GND everywhere, encoders on 3V3, and
GPIO 0/3/43/44/46 untouched.

### A4. Deploy the robot firmware
`robot_firmware.py` already contains your WiFi credentials
(`WIFI_SSID`/`WIFI_PASSWORD` at the top — edit if they change).
```bash
mpremote connect COM13 fs cp firmware/robot_firmware.py :main.py
mpremote connect COM13 reset
```
Then watch the boot log and **write down the IP address**:
```bash
mpremote connect COM13 repl
# ... [WiFi] connected, IP = 192.168.x.x   (Ctrl-] to exit)
```

---

## PART B — Verify the hardware (motors + WiFi link)

Do these once after any wiring/firmware change. Lift the wheels off the
ground for B1–B3.

### B1. Discovery + open-loop motor sanity (normal PID firmware)
Discover the bot IP without a cable:
```bash
python pc/find_bot.py
python pc/find_bot.py 192.168.50.255   # try this if general broadcast finds nothing
```
Then lift the wheels and run the open-loop sanity test through the normal PID firmware:
```bash
python pc/test_suite.py --test 1 --ip 192.168.x.x
```

PASS = RPM rises clearly with duty (≈40 RPM @ 0.40 → 100+ @ 1.00).

### B2. WiFi control-loop tests (PID firmware running)
```bash
python pc/test_suite.py --test 0 --ip 192.168.x.x   # link check (usually 192.168.50.106)
python pc/test_suite.py --test 1 --ip 192.168.x.x   # open-loop sanity
python pc/test_suite.py --test 2 --ip 192.168.x.x   # step response (logs CSV)
python pc/test_suite.py --test 3 --ip 192.168.x.x   # straight + turn
python pc/test_suite.py --test 5 --ip 192.168.x.x   # failsafe (IMPORTANT)
```
Do NOT drive untethered until Test 5 passes (500 ms command-loss → coast).

---

## PART C — Camera & arena setup (once)

### C1. Place the ArUco markers
- Print markers 0–3 (DICT_4X4_50) and fix them at the arena corners
  **matching their filenames**: 0 = bottom-left, 1 = bottom-right,
  2 = top-right, 3 = top-left. Flat, unoccluded, with a white quiet zone
  around each.
- Fix marker 4 flat on top of the robot, **printed top edge pointing to the
  robot's FRONT**.
- The 106.5 in × 68 in dimensions are measured marker-CENTER to center.

### C2. Mount the camera
Rapoo C280 on the ceiling (~8 ft / 2.4 m above the arena), aimed at the
arena center, ALL FOUR corner markers comfortably in frame at all times.
Fix it rigidly. The software forces 2560×1440 MJPEG, disables autofocus and
auto-exposure, and compensates camera latency in the control loop — but a
stable mount and constant lighting are still essential.

### C3. Validate the vision pipeline (no robot needed)
```bash
python pc/camera_pose.py --index 1 --debug-view
```
(If you have a built-in webcam, try `--index 1`.)
- Check the startup line: it should report `got 2560x1440@30` (MJPEG).
- Green boxes on markers 0–3, red on marker 4; pose prints at 5 Hz.
- Move the robot by hand: x should run 0 → 2.705 m left-to-right,
  y −0.864 → +0.864 m bottom-to-top; rotate it and watch theta.
- Watch `fps=` (should be high and STABLE) and `track=` (~100 %).
- Lock in focus/exposure for the 8 ft distance (fixed thereafter):
  `python pc/camera_pose.py --index 1 --debug-view --focus 40 --exposure -6`
  (sweep --focus until markers are sharpest; note both values for step D2)
- Note the `lat=` latency estimate; if you later see systematic overshoot
  along the travel direction, set it explicitly with `--latency-s`.

---

## PART D — Run the S-curve trajectory

### D1. (Optional) Dry-run in simulation first
```bash
python pc/test_suite.py --test 4 --ip 192.168.x.x
```
(`--pose sim` is the default; a connected robot follows simulated commands —
lift the wheels if you don't want it driving yet.)

### D2. Real vision-guided run
1. Robot powered, PID firmware running (from A4/B1 restore), on the arena
   floor — **any position, any orientation**.
2. PC and robot on the same 2.4 GHz network; webcam plugged in.
3. Run (add the --focus/--exposure/--latency-s values you tuned in C3):
```bash
python pc/test_suite.py --test 4 --pose camera --ip 192.168.x.x \
    --wp-timeout 8 --home-timeout 20 --focus 40 --debug-view
```

### SKIP TEST 4 AND RUN:
python pc\arena_calibrate_move.py --camera-index 1 --debug-view
python pc\arena_calibrate_move.py --camera-index 1 --width 1280 --height 720 --no-mjpeg --debug-view
python pc\arena_calibrate_move.py --camera-index 1 --width 1280 --height 720 --debug-view
python pc\arena_calibrate_move.py --camera-index 1 --debug-view

python pc\arena_calibrate_move.py --camera-index 1 --debug-view
python pc\arena_calibrate_move.py --camera-index 1 --debug-view --speed 0.30 --align-wheel-speed 0.06 --align-stable 4

What you will see:
1. CSV loads, waypoints shift into the arena frame (file on disk untouched).
2. **HOMING**: the robot drives from wherever it sits to the trajectory
   origin (0.753, −0.450), then pivots to face 0.00 rad.
3. **PATH**: follows all 250 waypoints at 20 Hz; each leg is computed from
   the camera-measured pose, so any error at one waypoint is compensated on
   the way to the next (the CSV is never modified). A waypoint not reached
   within 8 s is skipped and its error rolls into the next leg.
4. The robot coasts at the end; PASS requires final error ≤ 0.10 m.

Abort any time with **Ctrl-C** (the firmware failsafe coasts the robot
within 500 ms of command loss; camera loss also aborts + coasts).

---

## Quick reference: daily operation (after one-time setup)

```bash
# 1. power the robot (robot_firmware.py auto-runs), put it anywhere in the arena
# 2. check vision (optional):
python pc/camera_pose.py --index 1 --debug-view
# 3. run:
python pc/test_suite.py --test 4 --pose camera --ip 192.168.x.x \
    --wp-timeout 8 --home-timeout 20 --debug-view
```

## Troubleshooting

| Symptom | First thing to check |
|---|---|
| `cannot open camera index 1` | Wrong index (try 1), or another app owns the webcam |
| no telemetry / bot not found | Run `python pc/find_bot.py`; check same 2.4 GHz network, robot IP, and that `robot_firmware.py` is running |
| No telemetry in test 0 | Robot IP changed? Check DHCP reservation / REPL boot log |
| Wheels dead but camera fine | Run B1 diagnostic — DRV8833 VM power path |
| `arena corner markers not all visible` | Marker placement/lighting; check with --debug-view |
| Robot hunts near origin | Normal: 0.15 m/s anti-stall floor; homing aligns in stage 1b |
| Waypoints outside arena warning | CSV scale/frame — see ARENA_GEOMETRY.md |
