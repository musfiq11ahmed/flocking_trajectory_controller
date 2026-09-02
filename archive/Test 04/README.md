# Autonomous Trajectory-Following Robot

A complete system for an ESP32-S3 differential-drive robot that follows CSV
trajectories using overhead webcam localisation with ArUco markers.

---

## System Architecture

```
┌──────────────── PC  (Python 3.10+) ────────────────────┐
│                                                         │
│  Webcam ──► ArUco Detection ──► Homography Transform   │
│                                      │                  │
│             CSV Trajectory ──► Pure Pursuit Controller  │
│                                      │                  │
│                              (v_left, v_right) ticks/s  │
│                                      │                  │
│                               USB Serial TX ────────────┼──┐
└─────────────────────────────────────────────────────────┘  │
                                                             │
┌──────────────── ESP32-S3 (MicroPython) ─────────────────┐  │
│                                                          │  │
│  USB Serial RX ◄─────────────────────────────────────────┼──┘
│       │                                                  │
│  Target speeds ──► PID Left  ──► DRV8833 ──► Left Motor  │
│                    PID Right ──► DRV8833 ──► Right Motor  │
│                                                          │
│  Encoder Left  ──► Speed measurement ──► PID feedback    │
│  Encoder Right ──► Speed measurement ──► PID feedback    │
└──────────────────────────────────────────────────────────┘
```

---

## Hardware Specifications

| Component | Detail |
|-----------|--------|
| MCU | ESP32-S3 DevKit |
| Motors | N20 DC gear motors, 6 V, 500 RPM, with quadrature encoders |
| Motor driver | DRV8833 dual H-bridge |
| Wheels | Pololu, Ø 42 mm (1.65") |
| Wheelbase | 7.5 cm (centre-to-centre) |
| Camera | Rapoo C280 2K webcam (ceiling-mounted) |
| Arena | 9 ft × 77 in  (2.74 m × 1.96 m) |
| ArUco markers | DICT_4X4_50;  IDs 0–3 = arena corners,  ID 4 = robot |

---

## Wiring Diagram

```
 ┌────────────────────────────────────────────────────────────┐
 │                      ESP32-S3 DevKit                       │
 │                                                            │
 │   GPIO 4  ─────── AIN1 ┐                                  │
 │   GPIO 5  ─────── AIN2 ├─ DRV8833 channel A → Left motor  │
 │                         │                                  │
 │   GPIO 6  ─────── BIN1 ┐                                  │
 │   GPIO 7  ─────── BIN2 ├─ DRV8833 channel B → Right motor │
 │                         │                                  │
 │   3.3 V   ─────── STBY (Wake DRV8833 from sleep)          │
 │                                                            │
 │   GPIO 1  ─────── Encoder L, channel A                     │
 │   GPIO 2  ─────── Encoder L, channel B                     │
 │   GPIO 9  ─────── Encoder R, channel A                     │
 │   GPIO 10 ─────── Encoder R, channel B                     │
 │                                                            │
 │   3.3 V   ─────── Encoder VCC (both left & right)          │
 │   GND     ─────── Common GND (DD38LOSA VOUT−)              │
 │   5V/VBUS ─────── DD38LOSA VOUT+ (5V power from reg)       │
 │   USB     ─────── PC (serial communication)                │
 └────────────────────────────────────────────────────────────┘

 Power Distribution & DRV8833:
   Battery (7.4V–8.4V) +  → SPDT Switch → DD38LOSA VIN+
   Battery (7.4V–8.4V) −  → DD38LOSA VIN−
   100 µF Capacitor       → Across DD38LOSA VOUT+ and VOUT−
   
   DD38LOSA VOUT+ (5V)    → ESP32-S3 5V (or VBUS) & DRV8833 VM
   DD38LOSA VOUT− (GND)   → ESP32-S3 GND & DRV8833 GND
   
   AO1 / AO2  → Left  motor terminals (M2 / M1)
   BO1 / BO2  → Right motor terminals (M1 / M2)
```

### ⚠️ Important Notes

- **5V Power Rail**: Ensure the DD38LOSA is properly regulating 5V before connecting it to the ESP32-S3 or the DRV8833 to prevent damage.
- **Common ground** is established through the DD38LOSA VOUT− pin which connects to both the ESP32 and DRV8833.
- **Logic Levels**: The encoders are powered from the ESP32's 3.3V pin, meaning their signal outputs (Phase A/B) are safely at 3.3V logic levels suitable for the ESP32-S3 GPIO pins.

---

## Software Setup

### 1. PC — Install Python dependencies

```bash
pip install opencv-contrib-python numpy pyserial
```

### 2. ESP32-S3 — Flash MicroPython firmware

1. Download the latest **MicroPython for ESP32-S3** from
   <https://micropython.org/download/ESP32_GENERIC_S3/>
2. Flash with `esptool`:
   ```bash
   pip install esptool
   python -m esptool --chip esp32s3 --port COM3 erase-flash
   python -m esptool --chip esp32s3 --port COM3 write_flash -z 0x0 ESP32_GENERIC_S3-*.bin
   ```
3. Verify: open Thonny → select MicroPython (ESP32) → you should see the REPL.

### 3. Upload ESP32 code

Using **Thonny** (easiest) or **mpremote**:

```bash
# With mpremote:
pip install mpremote
mpremote connect COM3 cp esp32/pid.py :pid.py
mpremote connect COM3 cp esp32/motor.py :motor.py
mpremote connect COM3 cp esp32/robot.py :robot.py
mpremote connect COM3 cp esp32/main.py :main.py
```

After uploading, **reset** the ESP32.  Its `main.py` will run automatically
and print `READY` over USB serial.

### 4. Print ArUco markers

```bash
cd "d:\NIRO\flocking test\calibration"
python generate_aruco_markers.py
```

Print the 5 images on **matte** paper (no gloss!).  Recommended sizes:
- Arena corners (IDs 0–3): **8–10 cm**
- Robot marker (ID 4): **5–8 cm**

---

## Arena Setup

### Coordinate Convention

```
            +Y (up)
             ↑
             │
  ID 3       │         ID 2
 (−W/2,H/2)  │      (W/2,H/2)
             │
 ────────────O────────────→ +X (right)
             │
  ID 0       │         ID 1
 (−W/2,−H/2) │      (W/2,−H/2)
             │
```

- **Origin** = centre of the arena
- **θ = 0** → facing +X (right)
- **θ = π/2** → facing +Y (up)
- θ increases **counter-clockwise**
- Trajectory CSV values for θ are in **radians**

### Marker Placement

1. Place marker **ID 0** at the bottom-left corner of your arena.
2. Place marker **ID 1** at bottom-right.
3. Place marker **ID 2** at top-right.
4. Place marker **ID 3** at top-left.
5. Mount marker **ID 4** flat on top of the robot — **the top edge of the
   marker (between corner 0 and corner 1) must face the robot's front**.

### Camera Mounting

Mount the Rapoo C280 on the ceiling, pointing straight down, centred over
the arena.  The camera's 85° FOV at ~2.5 m height covers ≈ 4.6 m, which
is enough for the 2.74 m arena width.

---

## Running the System

### Quick Start

```bash
cd "d:\NIRO\flocking test\pc"
python main.py --port COM3 --trajectory ../trajectories/sample_trajectory.csv
```

### Vision-Only Debug (no ESP32 needed)

```bash
python main.py --no-serial --trajectory ../trajectories/sample_trajectory.csv
```

### Keyboard Controls (OpenCV window)

| Key | Action |
|-----|--------|
| `SPACE` | Start / resume trajectory tracking |
| `E` | Emergency stop |
| `R` | Reset (re-calibrate arena) |
| `Q` | Quit |

### What You'll See

The OpenCV window shows:
- Green rectangle: arena boundary
- Blue polyline: trajectory path
- Red circle + arrow: robot position + heading
- Yellow circle: current target waypoint
- Orange circle: lookahead point
- Text overlay: state, position, heading, commands

---

## Calibration & Tuning

### Step 1 — Encoder Calibration (TICKS_PER_REV)

The default `TICKS_PER_REV = 420` assumes 7 PPR encoder, 2× decoding,
30:1 gear ratio.  **Measure yours**:

1. Open Thonny, connect to ESP32, run in REPL:
   ```python
   from robot import Encoder
   enc = Encoder(9, 10)
   print(enc.count)   # should be 0
   ```
2. Manually rotate the left wheel **exactly one full revolution**.
3. Read `enc.count`.  That number = your `TICKS_PER_REV`.
4. Update `TICKS_PER_REV` in `pc/config.py` and `GEAR_RATIO` if needed.

### Step 2 — Motor Direction

If a motor spins the wrong way:
- Swap the motor wires, **OR**
- Set `LEFT_INVERTED = True` or `RIGHT_INVERTED = True` in `esp32/robot.py`.

### Step 3 — PID Tuning

Start with the defaults (`Kp=2.0, Ki=0.5, Kd=0.05`) and adjust:

1. Set `Ki = 0, Kd = 0`.
2. Increase `Kp` until the motor responds quickly but doesn't oscillate.
3. Add `Ki` (small increments) to eliminate steady-state speed error.
4. Add `Kd` to dampen any overshoot.

You can observe the PID behaviour by watching the `FB:` feedback lines
in a serial monitor while sending constant-speed commands.

### Step 4 — Controller Tuning

In `pc/config.py`:

| Parameter | Effect |
|-----------|--------|
| `LOOKAHEAD_DISTANCE` | Larger → smoother turns, may cut corners. Smaller → tighter tracking, may oscillate. |
| `CRUISE_SPEED` | Linear speed (m/s). Start low (0.08), increase when tracking is stable. |
| `K_HEADING` | Heading correction gain. Increase if robot doesn't align to θ. |

---

## Trajectory CSV Format

```csv
x,y,theta
-0.600000,-0.450000,0.000000
-0.595181,-0.449957,0.017998
...
```

- **x, y**: position in metres (arena world frame, origin = centre)
- **theta**: heading in **radians** (0 = facing +X, counter-clockwise positive)
- Whitespace or comma delimiters are both accepted.
- First row is a header and is skipped.

---

## Troubleshooting

| Symptom | Possible Cause | Fix |
|---------|---------------|-----|
| "Cannot open camera" | Camera in use / wrong index | Close other apps; try `--camera 1` |
| ArUco markers not detected | Glare / too small / too far | Use matte print; increase marker size |
| Homography never calibrates | Not all 4 corners visible | Adjust camera position / angle |
| Robot heading is 90° off | Marker mounted sideways | Rotate marker so top edge faces forward |
| Motors don't respond | Serial port wrong / REPL active | Check COM port; reset ESP32 |
| Robot oscillates | PID gains too high | Reduce Kp; check encoder wiring |
| Robot drifts / circles | One motor inverted | Swap terminals or set `*_INVERTED = True` |
| "Timeout waiting for READY" | ESP32 main.py not running | Upload files; reset board |
| "Failed to connect to ESP32" | Board didn't enter bootloader mode | Hold BOOT button, press EN/RESET, release BOOT, then run esptool again |

---

## Mathematical Reference

### Differential Drive Inverse Kinematics
```
v_left  = v − ω · L / 2
v_right = v + ω · L / 2
```
where `v` = linear velocity, `ω` = angular velocity, `L` = wheelbase.

### Pure Pursuit Curvature
```
α = atan2(target_y − robot_y, target_x − robot_x) − robot_θ
κ = 2 · sin(α) / L_d
ω = v · κ
```
where `L_d` = lookahead distance.

### Angle Normalisation
```python
angle = angle % (2π)
if angle > π:
    angle -= 2π
```

### Encoder Speed Measurement
```
speed_tps = Δticks / Δt      # ticks per second
speed_mps = speed_tps × (π · D) / TICKS_PER_REV
```
where `D` = wheel diameter.

---

## File Structure

```
d:\NIRO\flocking test\
├── pc/
│   ├── main.py                  # Main PC application
│   ├── vision.py                # ArUco detection + homography
│   ├── trajectory_controller.py # Pure Pursuit controller
│   ├── serial_comm.py           # Serial communication
│   ├── config.py                # All tunable parameters
│   └── utils.py                 # Math helpers
├── esp32/
│   ├── main.py                  # MicroPython entry point
│   ├── robot.py                 # Robot controller (PID + serial + encoders)
│   ├── motor.py                 # DRV8833 motor driver
│   └── pid.py                   # PID controller
├── trajectories/
│   └── sample_trajectory.csv    # Example trajectory
├── calibration/
│   ├── generate_aruco_markers.py  # Print ArUco markers
│   └── calibrate_camera.py      # Optional camera calibration
└── README.md                    # This file
```
