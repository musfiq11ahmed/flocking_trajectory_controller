# SPEC — Differential-Drive Bot Test Code Suite

## 1. Hardware Context (authoritative, from user's pin diagram)

- MCU: **ESP32-S3** (Arduino framework, dual core)
- Motor driver: **DRV8833** (shared 5V rail from DD38LOSA buck; STBY tied to 3V3 — i.e. driver is always awake; firmware may still drive STBY on a GPIO is NOT allowed — STBY is hardwired to 3V3)
- Motors: 2× N20 gearmotor with magnetic quadrature encoders
- Power: 5V rail shared between ESP32-S3 (5V/VBUS) and DRV8833 VM; encoders powered from 3V3

### Pin map (MUST match exactly)
| Function | ESP32-S3 GPIO |
|---|---|
| Left motor forward (DRV8833 AIN1) | GPIO 4 |
| Left motor reverse (DRV8833 AIN2) | GPIO 5 |
| Right motor forward (DRV8833 BIN1) | GPIO 6 |
| Right motor reverse (DRV8833 BIN2) | GPIO 7 |
| Left encoder Phase A (C1) | GPIO 1 |
| Left encoder Phase B (C2) | GPIO 2 |
| Right encoder Phase A (C1) | GPIO 9 |
| Right encoder Phase B (C2) | GPIO 10 |

**Forbidden pins (never use): 0, 3, 43, 44, 46.**

### DRV8833 drive mode
Use **fast-decay (coast) IN/IN mode**: PWM on one IN pin while the other is held LOW for forward; swap for reverse; both LOW = coast/stop. Implement via ESP32 LEDC PWM channels (one channel per IN pin, 4 channels total, ~20 kHz to stay ultrasonic, 8-bit or 10-bit resolution).

## 2. System Architecture (from user's diagram)

- Layer 1/2 (PC): vision + controller compute target left/right wheel **RPM**, sent over **UDP** (2.4 GHz WiFi) to the robot.
- Layer 3 (ESP32-S3 firmware):
  - **Core 0**: Network listener task — receives UDP RPM target packets.
  - **Core 1**: Hardware PID controller task — 50 ms loop, reads encoders, adjusts PWM to DRV8833.
- Firmware also streams **telemetry over UDP** back to the PC (encoder counts + measured RPM) so the test harness can validate behavior.

## 3. Deliverables (all under `/mnt/agents/output/bot_test_code/`)

```
bot_test_code/
├── firmware/
│   └── esp32_bot_firmware.ino      # single-file Arduino sketch for ESP32-S3
├── pc/
│   ├── udp_protocol.py             # shared packet encode/decode helpers
│   ├── test_suite.py               # interactive test harness (tests 0–5 below)
│   ├── generate_scurve_csv.py      # writes scurve_trajectory.csv (250 waypoints)
│   └── requirements.txt            # pure stdlib preferred; numpy allowed
├── README.md                       # wiring checklist + run instructions
```

### 3.1 Firmware: `esp32_bot_firmware.ino`

Requirements:
- Arduino core for ESP32-S3. Single `.ino` file, well-commented. No external libraries beyond ESP32 Arduino core (WiFi.h, WiFiUdp.h, driver/ledc or ledcAttach API — pick the API matching Arduino-ESP32 v3.x; note v2.x alternative in a comment).
- WiFi: connect to a configurable SSID/password (#define at top). Static IP optional (#define). Print assigned IP over Serial (115200).
- **UDP protocol (port 4210, little-endian floats, plain struct — must match pc/udp_protocol.py exactly):**
  - Command packet PC→ESP32, 8 bytes: `float target_left_rpm, float target_right_rpm`.
  - Telemetry packet ESP32→PC, 24 bytes @ 20 Hz: `float left_rpm_measured, float right_rpm_measured, int32 left_ticks, int32 right_ticks, float left_pwm_duty (0..1), float right_pwm_duty (0..1)`. Send to the last IP/port that sent a command.
- **Core 0 task** `networkTask`: blocking UDP receive with timeout; parses command packet; writes targets into shared volatile struct; sends telemetry every 50 ms; heartbeat LED optional (use a safe pin like GPIO 21 or skip).
- **Core 1 task** `pidTask`: fixed 50 ms period (vTaskDelayUntil):
  - Read encoder tick deltas (quadrature via interrupt-driven counting on GPIO 1/2/9/10, 4x decoding or at least 2x; document choice).
  - Convert ticks → RPM using `#define ENCODER_TICKS_PER_REV` (default 12 ticks/motor-rev × gear ratio — expose as configurable; default: N20 1:150 gear, 12 CPR motor → 1800 ticks per output rev; put in a clearly-marked tunables block).
  - PI(D) controller per wheel with anti-windup; output −1..1 duty.
  - Convert duty → DRV8833 IN/IN signals via LEDC (4 channels).
  - **Failsafe**: if no UDP command received for >500 ms, zero targets (matches the "dead reckoning/failsafe" intent).
  - Emergency stop: command of exactly (0.0, 0.0) = coast.
- Serial debug prints at boot and on WiFi events only (keep loop clean).

### 3.2 PC: `udp_protocol.py`
- `pack_command(left_rpm, right_rpm) -> bytes`, `unpack_telemetry(bytes) -> dict`, constants for port/period. Must mirror firmware structs byte-for-byte (document with comments).

### 3.3 PC: `test_suite.py`
Interactive CLI (numbered menu + `--test N --ip X.X.X.X` non-interactive mode). Tests:
- **Test 0 — Link check**: send zero command, receive telemetry for 2 s, report packet rate & latency feel.
- **Test 1 — Open-loop sanity**: prompt user to lift wheels off ground; command +30 RPM left only, then right only, then both forward, both reverse (2 s each, 1 s coast between). Verify encoder ticks change in the expected direction and report measured RPM vs target. PASS/FAIL criteria printed.
- **Test 2 — Closed-loop step response**: command 0→60 RPM on both wheels, log telemetry to CSV (`logs/step_response_<ts>.csv`), print settling stats (time to ±10% of target, overshoot, steady-state error).
- **Test 3 — Differential maneuvers**: straight (both +40 RPM, 3 s), turn in place (left +30 / right −30, 2 s). Report tick ratios to check symmetry.
- **Test 4 — Waypoint navigation simulator** (implements the 10 robot tasks from the spec, but with **simulated pose feedback** — no camera needed): loads `scurve_trajectory.csv`, homes to first waypoint, then iterates: simulated pose + optional noise with EMA filter, Euclidean distance + shortest-path heading error, unicycle control law (v = kρ·dist, ω = kα·heading_error, with tunable gains), split to left/right wheel speeds (wheel radius 0.03 m, track width 0.12 m as defaults, configurable), friction compensation (scale up so min wheel speed ≥ 0.15 m/s while preserving ratio), clamp at 0.30 m/s, convert m/s → RPM, stream over UDP at 20 Hz, advance when simulated error < 0.10 m. On a real bot this same loop can later be fed by the ArUco pipeline — structure it so the pose source is a pluggable class (`SimulatedPoseSource` now, `CameraPoseSource` stub for later).
- **Test 5 — Failsafe check**: command motion, then stop sending packets; verify robot coasts within ~1 s (telemetry RPM → 0).

### 3.4 PC: `generate_scurve_csv.py`
- Generates a smooth parametric S-curve (two arcs joined, or a cubic S in the plane) with **exactly 250 waypoints**, CSV columns `x,y,theta` (meters, radians), theta = path tangent. Default arena scale ~1.5 m × 1.5 m. `--out` argument.

### 3.5 README.md
- Wiring checklist reproducing the pin tables (power, control, motors, encoders, forbidden pins).
- Arduino IDE / arduino-cli setup for ESP32-S3, tunables block explanation.
- How to run each test, what PASS looks like, safety notes (lift wheels, current limits).

## 4. Quality bar
- All pin numbers exactly as Section 1; forbidden pins unused.
- UDP structs byte-identical between firmware and Python (use `struct` with `<ff` and `<ffllff`... — careful: `int32` not `long`; use `<ff i i ff`).
- Python: stdlib only (socket, struct, time, csv, argparse, math); numpy NOT required.
- No blocking delays in firmware PID task other than vTaskDelayUntil.
- Code compiles conceptually for Arduino-ESP32 v3.x (use `ledcAttach(pin, freq, res)` / `ledcWrite(pin, duty)` API), with v2.x equivalents in comments.
