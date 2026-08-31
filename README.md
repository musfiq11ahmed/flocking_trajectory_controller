# Bot Test Code — ESP32-S3 Differential-Drive Robot

Test firmware + PC harness for a differential-drive robot:

- **MCU:** ESP32-S3 running **MicroPython** (threads share one VM core — the
  network listener runs in a `_thread`, the 50 ms PID loop on the main thread)
- **Driver:** DRV8833 (STBY hardwired to 3V3 — always awake, fast-decay IN/IN mode)
- **Motors:** 2× N20 gearmotor (1:150) with magnetic quadrature encoders
- **Link:** UDP over 2.4 GHz WiFi (port 4210); PC sends wheel RPM targets,
  robot runs a 50 ms PID loop and streams telemetry back at 20 Hz.

```
bot_test_code/
├── firmware/
│   └── robot_firmware.py           # single-file MicroPython firmware — copy
│                                   # to the ESP32-S3 as main.py (no boot.py needed)
├── pc/
│   ├── udp_protocol.py             # shared packet encode/decode helpers
│   ├── test_suite.py               # interactive test harness (tests 0–5)
│   ├── generate_scurve_csv.py      # writes scurve_trajectory.csv (250 waypoints)
│   └── requirements.txt            # (empty — pure Python stdlib)
└── README.md
```

---

## 1. Wiring checklist

### 1.1 Battery / buck converter

| From | To | Notes |
|---|---|---|
| Battery Pack Negative (−) | DD38LOSA VIN− / GND | |
| 100 µF capacitor | Across DD38LOSA VOUT+ and VOUT− | striped (negative) leg → VOUT− |

### 1.2 5V power distribution (shared rail)

| Source pin | Destination pin | Purpose |
|---|---|---|
| DD38LOSA VOUT+ (5V) | ESP32-S3 5V (or VBUS) | Microcontroller power |
| DD38LOSA VOUT− (GND) | ESP32-S3 GND | Common ground |
| DD38LOSA VOUT+ (5V) | DRV8833 VM | Motor driver power |
| DD38LOSA VOUT− (GND) | DRV8833 GND | Common ground |

### 1.3 ESP32-S3 → DRV8833 (control)

| ESP32-S3 pin | DRV8833 pin | Function |
|---|---|---|
| 3V3 | STBY | Wake from sleep mode (hardwired — firmware never drives STBY) |
| GPIO 4 | AIN1 | Left motor forward |
| GPIO 5 | AIN2 | Left motor reverse |
| GPIO 6 | BIN1 | Right motor forward |
| GPIO 7 | BIN2 | Right motor reverse |

### 1.4 DRV8833 → N20 motors (output)

| DRV8833 pin | Motor pin | Side |
|---|---|---|
| AO1 | Left N20 motor — M2 | Left |
| AO2 | Left N20 motor — M1 | Left |
| BO1 | Right N20 motor — M1 | Right |
| BO2 | Right N20 motor — M2 | Right |

### 1.5 N20 encoders → ESP32-S3 (PID feedback)

Encoders are powered from the ESP32's 3.3V regulator so logic levels stay
GPIO-safe.

| Encoder pin | ESP32-S3 pin | Encoder / phase |
|---|---|---|
| VCC | 3V3 | Left encoder power |
| GND | GND | Left encoder ground |
| C1 (Phase A) | GPIO 1 | Left encoder |
| C2 (Phase B) | GPIO 2 | Left encoder |
| VCC | 3V3 | Right encoder power |
| GND | GND | Right encoder ground |
| C1 (Phase A) | GPIO 9 | Right encoder |
| C2 (Phase B) | GPIO 10 | Right encoder |

### 1.6 Forbidden pins

**Never use GPIO 0, 3, 43, 44, 46** (strapping/USB/UART pins on the ESP32-S3).
The firmware and test scripts do not reference them.

---

## 2. Firmware setup (MicroPython on ESP32-S3)

All robot-side code is Python. No Arduino IDE, no C++ toolchain.

1. **Flash MicroPython** (once per board). Download the ESP32-S3 firmware
   `.bin` from <https://micropython.org/download/ESP32_GENERIC_S3/>, then:

   ```bash
   pip install esptool mpremote
   esptool.py --chip esp32s3 --port /dev/ttyUSB0 erase_flash
   esptool.py --chip esp32s3 --port /dev/ttyUSB0 write_flash -z 0x0 ESP32_GENERIC_S3-<version>.bin
   ```

   (Use the actual port — `COMx` on Windows, `/dev/ttyACM0` or `/dev/ttyUSB0`
   on Linux/macOS.)

2. **Edit WiFi credentials**: open `firmware/robot_firmware.py` and set
   `WIFI_SSID` / `WIFI_PASSWORD` in the TUNABLES block at the top.

3. **Copy the firmware to the board as `main.py`** (it auto-runs at boot; no
   `boot.py` is required):

   ```bash
   mpremote connect /dev/ttyUSB0 fs cp firmware/robot_firmware.py :main.py
   mpremote connect /dev/ttyUSB0 reset
   ```

   …or use **Thonny**: open `robot_firmware.py`, *Save as…* → *MicroPython
   device* → `main.py`, then press the reset button.

4. **Note the IP address printed at boot** (open the REPL/serial with
   `mpremote connect /dev/ttyUSB0 repl` or Thonny's shell) — you need it for
   every PC test. On connection failure the firmware prints a retry message
   and keeps retrying.

### Tunables (top of `firmware/robot_firmware.py`)

| Constant | Default | Meaning |
|---|---|---|
| `WIFI_SSID` / `WIFI_PASSWORD` | placeholders | your 2.4 GHz WiFi credentials |
| `WIFI_CONNECT_TIMEOUT_MS` | `15000` | per-attempt WiFi timeout before retry |
| `UDP_PORT` | `4210` | command + telemetry port |
| `TELEMETRY_PERIOD_MS` | `50` | telemetry rate (20 Hz) |
| `FAILSAFE_TIMEOUT_MS` | `500` | command-loss timeout → coast |
| `ENCODER_TICKS_PER_REV` | `1800.0` | 12 CPR × 150 gear ratio, **1× decoding** (Phase A rising edge only). Scale with your actual gearbox (e.g. 1:100 → 1200). |
| `PID_PERIOD_MS` | `50` | control-loop period |
| `PID_KP` / `PID_KI` | 0.02 / 0.04 | per-wheel PI gains (anti-windup via `PID_INTEGRAL_LIMIT` + conditional integration) |
| `PWM_FREQ_HZ` | `20000` | `machine.PWM` frequency — 20 kHz ultrasonic; duty set via `duty_u16` |
| `WHEEL_RADIUS_M` / `TRACK_WIDTH_M` | 0.03 / 0.12 m | chassis geometry (reference / future on-board kinematics) |
| `MAX_RPM` | `120.0` | clamp on commanded RPM |

### MicroPython limitations (read before tuning)

- **Soft IRQ encoders:** Python IRQ handlers (`Pin.irq(..., hard=False)`) are
  scheduled by the VM, not run as true hardware ISRs, so they **can miss ticks**
  at high edge rates. The firmware therefore uses **1× decoding** (Phase A
  rising edge, Phase B for direction) which keeps the edge rate at ~30 × RPM
  edges/s — roughly 3.6 kHz at the 120 RPM clamp, near the practical ceiling.
  Expect a usable ceiling of **a few kHz** per encoder; do not raise `MAX_RPM`
  or switch to 2×/4× decoding without re-validating with Test 1.
- **Shared-core threading:** MicroPython threads share one VM (GIL) on a single
  core — the `_thread` network listener and the main-thread PID loop mirror the
  Core 0 / Core 1 split logically, but there is **no true dual-core pinning**
  in mainline MicroPython. Heavy GC or WiFi bursts can add jitter to the 50 ms
  loop; the loop schedule itself is drift-free (`ticks_add`/`ticks_diff`).

### UDP protocol (must match `pc/udp_protocol.py` byte-for-byte)

| Packet | Direction | Size | Layout |
|---|---|---|---|
| Command | PC → ESP32 | 8 bytes | `<ff` : `float target_left_rpm, float target_right_rpm` |
| Telemetry | ESP32 → PC | 24 bytes @ 20 Hz | `<ffiiff` : `float left_rpm, float right_rpm, int32 left_ticks, int32 right_ticks, float left_pwm_duty, float right_pwm_duty` |

Telemetry is sent to whichever IP:port last sent a command. A command of
exactly `(0.0, 0.0)` means **coast**; losing commands for >500 ms also coasts
(failsafe).

---

## 3. PC tools

No dependencies beyond the Python 3 standard library (`requirements.txt` is a
comment-only placeholder).

```bash
cd pc
python3 generate_scurve_csv.py           # writes scurve_trajectory.csv (250 waypoints)
python3 test_suite.py                    # interactive menu (asks for robot IP)
python3 test_suite.py --test 2 --ip 192.168.1.50     # non-interactive
```

`test_suite.py` options: `--test {0..5}`, `--ip A.B.C.D`,
`--csv PATH` (test 4 waypoints), `--noise M` (test 4 simulated pose noise, 0 = off).

### Test 0 — Link check
Sends zero (coast) commands for 2 s and counts telemetry.
**PASS:** packets arrive at ≥ 15 Hz with ~50 ms gaps.

### Test 1 — Open-loop sanity
Prompts you to **lift the wheels off the ground**, then commands: left +30 RPM,
right +30 RPM, both +30, both −30 (2 s each, 1 s coast between).
**PASS:** encoder ticks move in the commanded direction and measured RPM is
within 35 % (or 8 RPM) of target on each driven wheel.

### Test 2 — Closed-loop step response
0 → 60 RPM step on both wheels for 4 s; telemetry logged to
`pc/logs/step_response_<timestamp>.csv`.
**PASS:** settling time to ±10 % ≤ 1.5 s, overshoot ≤ 25 %,
|steady-state error| ≤ 10 % of target. Tune `PID_KP`/`PID_KI` if it fails.

### Test 3 — Differential maneuvers
Straight line (both +40 RPM, 3 s) then turn in place (left +30 / right −30, 2 s).
**PASS:** straight right/left tick ratio in [0.7, 1.3]; turn left/|right| ratio
in [0.6, 1.4]. Persistent asymmetry → check wiring, motor health, or per-wheel
PID tuning.

### Test 4 — Waypoint navigation simulator
Runs the full 10-task navigation pipeline (load S-curve CSV → home to first
waypoint → per-waypoint: EMA-filtered pose, Euclidean distance + shortest-path
heading error, unicycle control law, friction compensation ≥ 0.15 m/s
preserving ratio, 0.30 m/s clamp, m/s → RPM, UDP @ 20 Hz, advance when error
< 0.10 m) using a `SimulatedPoseSource` instead of the camera — no vision
hardware needed. A `CameraPoseSource` stub class marks where the real ArUco +
homography pipeline plugs in later. If a physical robot is at `--ip`, its wheels
follow the simulated commands live.
**PASS:** all 250 waypoints traversed, final simulated error ≤ 0.10 m.

### Test 5 — Failsafe check
Commands +40 RPM for 1.5 s, then stops all transmissions and watches telemetry.
**PASS:** measured RPM reaches ~0 within 1.0 s of the last command (firmware
failsafe timeout is 500 ms).

---

## 4. Safety notes

- **Lift the wheels off the ground** for Tests 1, 2 and 5 (and Test 4 if a real
  robot is attached and you don't want it driving). Test 3 needs ground contact
  to be meaningful but works on a bench.
- The 5V rail is shared between the ESP32-S3 and the DRV8833 VM — check the
  buck converter output (5.0 V) and the 100 µF capacitor polarity before
  powering up.
- N20 stall current can exceed 1 A per motor; keep hands, cables and tools
  clear of the wheels, and don't hold a stalled motor under power.
- GPIO 0, 3, 43, 44, 46 are off-limits (boot strapping / USB / UART0).
- DRV8833 STBY is hardwired to 3V3; the driver is always awake. The robot
  relies on the firmware failsafe (500 ms command loss → coast) — verify it
  with Test 5 before driving untethered.
- Keep the PC and robot on the same 2.4 GHz network; UDP has no delivery
  guarantee, so the failsafe is your last line of defense.
