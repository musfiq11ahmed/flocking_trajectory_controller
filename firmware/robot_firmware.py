"""
robot_firmware.py -- MicroPython firmware for the ESP32-S3 differential-drive bot.
====================================================================================

Deploy: copy this file to the ESP32-S3 as ``main.py`` (mpremote or Thonny, see
README.md). It runs automatically at boot. No ``boot.py`` is needed.

Hardware:
  - MCU          : ESP32-S3
  - Motor driver : DRV8833 (STBY hardwired to 3V3 -> always awake; firmware
                   must NEVER drive STBY. Fast-decay IN/IN drive mode: PWM on
                   one IN pin with the other held at duty 0; both 0 = coast)
  - Motors       : 2x N20 gearmotor (1:150) with magnetic quadrature encoders

Architecture (mirrors the user's Core 0 / Core 1 diagram, using threads):
  - _thread worker  : network listener -- UDP socket on port 4210, receives
                      8-byte RPM command packets, streams 24-byte telemetry at
                      20 Hz back to the last command sender.
  - main thread     : 50 ms PI control loop (drift-free ticks_us scheduling),
                      reads encoders, drives the DRV8833 via machine.PWM,
                      enforces the 500 ms command-loss failsafe (coast).

  NOTE: mainline MicroPython runs all threads inside a single VM guarded by a
  GIL on ONE core (on ESP32-S3 there is only one core anyway), so this mirrors
  the *logic* of the Core 0 / Core 1 split but cannot pin tasks to separate
  physical cores the way the FreeRTOS Arduino firmware could.

UDP protocol (little-endian, packed, MUST match pc/udp_protocol.py):
  Command   PC->ESP32 (8 bytes)  : '<ff'     float target_left_rpm,
                                             float target_right_rpm
  Telemetry ESP32->PC (24 bytes) : '<ffiiff' float left_rpm_measured,
                                             float right_rpm_measured,
                                             int32 left_ticks,
                                             int32 right_ticks,
                                             float left_pwm_duty  (-1..+1),
                                             float right_pwm_duty (-1..+1)

Encoder decoding -- 1x (single channel, single edge):
  Python IRQ handlers on ESP32 are SOFT-scheduled (hard=False): they run as
  callbacks inside the VM, not as true hardware ISRs, so they can and WILL
  miss edges at full 4x quadrature rates. Practical ceiling is on the order
  of a few kHz of edges per second (worst-case drops under WiFi/GC load).
  Therefore this firmware counts only the RISING edge of Phase A and samples
  Phase B for direction (1x decoding). At ENCODER_TICKS_PER_REV = 1800 per
  output-shaft rev (12 CPR motor shaft x 150:1 gearbox, 1x), the edge rate is
    edges/s = (RPM / 60) * 12 * 150 = 30 * RPM
  i.e. ~3.6 kHz at the 120 RPM clamp -- already near the practical limit, so
  do NOT raise MAX_RPM or move to 2x/4x decoding without verifying counting
  accuracy (Test 1) first.

PIN MAP (authoritative -- matches SPEC.md / wiring guide; do NOT change):
  GPIO 4  -> DRV8833 AIN1  (left motor forward)
  GPIO 5  -> DRV8833 AIN2  (left motor reverse)
  GPIO 6  -> DRV8833 BIN1  (right motor forward)
  GPIO 7  -> DRV8833 BIN2  (right motor reverse)
  GPIO 1  <- left encoder  C1 (Phase A)
  GPIO 2  <- left encoder  C2 (Phase B)
  GPIO 9  <- right encoder C1 (Phase A)
  GPIO 10 <- right encoder C2 (Phase B)
  FORBIDDEN PINS (never used here): 0, 3, 43, 44, 46
  DRV8833 STBY is hardwired to 3V3 -- this firmware must NOT drive it.
"""

import time

import _thread
import network
import usocket as socket
import ustruct as struct
from machine import PWM, Pin

# ============================ TUNABLES (edit me) ============================

# --- WiFi credentials --------------------------------------------------------
WIFI_SSID = "YOUR_WIFI_SSID"
WIFI_PASSWORD = "YOUR_WIFI_PASSWORD"
WIFI_CONNECT_TIMEOUT_MS = 15000   # per-attempt timeout before printing a retry

# --- UDP protocol ------------------------------------------------------------
UDP_PORT = 4210                   # command listener + telemetry source port
TELEMETRY_PERIOD_MS = 50          # 20 Hz telemetry stream
FAILSAFE_TIMEOUT_MS = 500         # command-loss timeout -> coast

# --- Encoders ----------------------------------------------------------------
# N20 magnetic encoder: 12 CPR (cycles per motor-shaft rev) per channel.
# This firmware uses 1x decoding (Phase A rising edge only), so:
#   ticks per output-shaft rev = 12 * 150 (gear ratio) = 1800.
# GEAR-RATIO ASSUMPTION: N20 1:150 gearbox. If your motors have a different
# ratio, scale accordingly (e.g. 1:100 -> 1200, 1:298 -> 3576). If you change
# decoding to 2x, double this value -- but see the tick-rate warning above.
ENCODER_TICKS_PER_REV = 1800.0

# --- PI controller (per wheel, output = signed duty -1..+1) ------------------
PID_PERIOD_MS = 50                # control-loop period (main thread)
PID_KP = 0.020                    # duty per RPM of error
PID_KI = 0.040                    # duty per (RPM * s) of integrated error
PID_INTEGRAL_LIMIT = 1.0          # anti-windup clamp on the integral term

# --- PWM (machine.PWM) -------------------------------------------------------
PWM_FREQ_HZ = 20000               # 20 kHz = ultrasonic, fast-decay IN/IN
# Duty is set with duty_u16() (0..65535); no resolution constant needed.

# --- Chassis geometry (kept for reference / future on-board kinematics) ------
WHEEL_RADIUS_M = 0.03             # N20 wheel radius
TRACK_WIDTH_M = 0.12              # wheel-to-wheel distance

# --- Safety ------------------------------------------------------------------
MAX_RPM = 120.0                   # clamp on commanded target RPM

# ========================== END OF TUNABLES ==================================


# ------------------------------- Pin map -------------------------------------
PIN_L_FWD = 4     # GPIO 4  -> DRV8833 AIN1 (left forward)
PIN_L_REV = 5     # GPIO 5  -> DRV8833 AIN2 (left reverse)
PIN_R_FWD = 6     # GPIO 6  -> DRV8833 BIN1 (right forward)
PIN_R_REV = 7     # GPIO 7  -> DRV8833 BIN2 (right reverse)
PIN_ENC_L_A = 1   # GPIO 1  <- left encoder  phase A
PIN_ENC_L_B = 2   # GPIO 2  <- left encoder  phase B
PIN_ENC_R_A = 9   # GPIO 9  <- right encoder phase A
PIN_ENC_R_B = 10  # GPIO 10 <- right encoder phase B
# Forbidden pins 0, 3, 43, 44, 46 are intentionally not referenced anywhere.

# --- UDP packet layouts (byte-identical to pc/udp_protocol.py) ---------------
COMMAND_FORMAT = "<ff"
COMMAND_SIZE = struct.calcsize(COMMAND_FORMAT)        # 8 bytes
TELEMETRY_FORMAT = "<ffiiff"
TELEMETRY_SIZE = struct.calcsize(TELEMETRY_FORMAT)    # 24 bytes

# ---------------------- Shared state (thread <-> thread) ---------------------
# MicroPython threads share one VM (GIL), so plain attribute/int assignment is
# atomic enough here -- no locks are needed for these single-word fields.
shared = {
    "target_l": 0.0,        # target RPM, written by network thread
    "target_r": 0.0,
    "last_cmd_ms": 0,       # ticks_ms() of last valid command
    "ticks_l": 0,           # encoder totals, written by IRQ handlers
    "ticks_r": 0,
    "meas_l": 0.0,          # measured RPM, written by PID loop
    "meas_r": 0.0,
    "duty_l": 0.0,          # signed duty actually applied (-1..1)
    "duty_r": 0.0,
}


def _to_int32(v):
    """Wrap an unbounded Python int into signed int32 for the '<ffiiff' packet."""
    v &= 0xFFFFFFFF
    return v - 0x100000000 if v >= 0x80000000 else v


def _clamp(v, lo, hi):
    return lo if v < lo else (hi if v > hi else v)


# ------------------------------ DRV8833 motor --------------------------------
class Motor:
    """One DRV8833 half (two IN pins) in fast-decay IN/IN mode.

    duty > 0 : PWM on the forward pin, reverse pin held at duty 0
    duty < 0 : PWM on the reverse pin, forward pin held at duty 0
    duty = 0 : both pins at duty 0 -> coast/stop
    """

    def __init__(self, pin_fwd, pin_rev):
        self.pwm_fwd = PWM(Pin(pin_fwd, Pin.OUT), freq=PWM_FREQ_HZ, duty_u16=0)
        self.pwm_rev = PWM(Pin(pin_rev, Pin.OUT), freq=PWM_FREQ_HZ, duty_u16=0)

    def set(self, duty):
        duty = _clamp(duty, -1.0, 1.0)
        d = int(abs(duty) * 65535 + 0.5)
        if duty > 0.0:
            self.pwm_fwd.duty_u16(d)
            self.pwm_rev.duty_u16(0)
        elif duty < 0.0:
            self.pwm_fwd.duty_u16(0)
            self.pwm_rev.duty_u16(d)
        else:
            self.pwm_fwd.duty_u16(0)   # coast
            self.pwm_rev.duty_u16(0)

    def coast(self):
        self.set(0.0)


# ------------------------------- Encoders ------------------------------------
class Encoder:
    """1x quadrature counting: Phase A RISING edge, Phase B gives direction.

    The handler is intentionally minimal (read B, increment/decrement one
    counter) because MicroPython IRQ callbacks are soft-scheduled and every
    extra microsecond raises the chance of missing edges at speed.
    Direction convention: A rising while B is LOW = forward (+1). If your
    motors count backwards relative to the drive direction, swap the A/B
    wiring (or the two GPIO numbers) for that side.
    """

    def __init__(self, pin_a, pin_b, count_key):
        self._phase_b = Pin(pin_b, Pin.IN)
        self._key = count_key
        phase_a = Pin(pin_a, Pin.IN)
        phase_a.irq(handler=self._on_a_rising,
                    trigger=Pin.IRQ_RISING,
                    hard=False)   # soft IRQ: VM callback (see header warning)

    def _on_a_rising(self, pin):
        if self._phase_b.value():
            shared[self._key] -= 1
        else:
            shared[self._key] += 1


# Hardware objects exist at module level so the try/finally in main() can
# always coast the motors even if boot fails partway through.
motor_left = Motor(PIN_L_FWD, PIN_L_REV)
motor_right = Motor(PIN_R_FWD, PIN_R_REV)
enc_left = Encoder(PIN_ENC_L_A, PIN_ENC_L_B, "ticks_l")
enc_right = Encoder(PIN_ENC_R_A, PIN_ENC_R_B, "ticks_r")


# ------------------------------- WiFi ----------------------------------------
def wifi_connect():
    """Station-mode connect with timeout + retry; prints the IP on success."""
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)
    attempt = 0
    while True:
        attempt += 1
        print('[WiFi] connecting to "%s" (attempt %d)' % (WIFI_SSID, attempt))
        wlan.connect(WIFI_SSID, WIFI_PASSWORD)
        t0 = time.ticks_ms()
        while not wlan.isconnected():
            if time.ticks_diff(time.ticks_ms(), t0) > WIFI_CONNECT_TIMEOUT_MS:
                print("[WiFi] connect timed out, retrying...")
                break
            time.sleep_ms(250)
        else:
            break
    print("[WiFi] connected, IP =", wlan.ifconfig()[0])
    print("[UDP ] listening on port", UDP_PORT)
    return wlan


# ------------------------ Network thread (Core 0 role) -----------------------
def network_thread():
    """UDP command listener + 20 Hz telemetry streamer.

    Mirrors the Arduino core-0 networkTask: blocking-ish receive with a short
    timeout (so telemetry keeps its cadence on the same thread), parse 8-byte
    '<ff' commands, remember the sender, stream 24-byte '<ffiiff' telemetry
    back to it every TELEMETRY_PERIOD_MS.
    """
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind(("0.0.0.0", UDP_PORT))
    s.settimeout(0.005)

    remote = None
    last_telemetry = time.ticks_ms()

    while True:
        # --- receive ---------------------------------------------------------
        try:
            data, addr = s.recvfrom(COMMAND_SIZE)
        except OSError:
            data, addr = None, None  # timeout: no packet this slice

        if data is not None and len(data) == COMMAND_SIZE:
            left, right = struct.unpack(COMMAND_FORMAT, data)
            shared["target_l"] = _clamp(left, -MAX_RPM, MAX_RPM)
            shared["target_r"] = _clamp(right, -MAX_RPM, MAX_RPM)
            shared["last_cmd_ms"] = time.ticks_ms()
            remote = addr          # telemetry goes to the last command sender
        # malformed-size datagrams are silently discarded (UDP has no framing)

        # --- telemetry @ 20 Hz ------------------------------------------------
        now = time.ticks_ms()
        if remote is not None and \
                time.ticks_diff(now, last_telemetry) >= TELEMETRY_PERIOD_MS:
            last_telemetry = now
            pkt = struct.pack(
                TELEMETRY_FORMAT,
                shared["meas_l"],
                shared["meas_r"],
                _to_int32(shared["ticks_l"]),
                _to_int32(shared["ticks_r"]),
                shared["duty_l"],
                shared["duty_r"],
            )
            try:
                s.sendto(pkt, remote)
            except OSError:
                pass                 # WiFi hiccup: drop one frame, keep going


# ---------------------------- PID loop (Core 1 role) -------------------------
def pid_loop():
    """50 ms per-wheel PI controller. Runs on the main thread.

    Scheduled with ticks_add/ticks_diff off an absolute deadline so the period
    does not drift (the MicroPython equivalent of vTaskDelayUntil).
    """
    dt = PID_PERIOD_MS / 1000.0
    # ticks -> RPM:  delta * (control periods per minute) / ticks per rev
    ticks_to_rpm = (60000.0 / PID_PERIOD_MS) / ENCODER_TICKS_PER_REV

    prev_ticks_l = shared["ticks_l"]
    prev_ticks_r = shared["ticks_r"]
    integ_l = 0.0
    integ_r = 0.0

    period_us = PID_PERIOD_MS * 1000
    next_tick = time.ticks_add(time.ticks_us(), period_us)

    while True:
        # --- failsafe: no valid command for >500 ms -> zero targets ---------
        # Zero targets are handled below as "coast", so a lost link stops the
        # motors within one PID period of the timeout.
        if time.ticks_diff(time.ticks_ms(), shared["last_cmd_ms"]) > \
                FAILSAFE_TIMEOUT_MS:
            shared["target_l"] = 0.0
            shared["target_r"] = 0.0

        # --- measure ----------------------------------------------------------
        ticks_l = shared["ticks_l"]
        ticks_r = shared["ticks_r"]
        rpm_l = (ticks_l - prev_ticks_l) * ticks_to_rpm
        rpm_r = (ticks_r - prev_ticks_r) * ticks_to_rpm
        prev_ticks_l = ticks_l
        prev_ticks_r = ticks_r

        tgt_l = shared["target_l"]
        tgt_r = shared["target_r"]

        if tgt_l == 0.0 and tgt_r == 0.0:
            # Emergency stop / failsafe: command of exactly (0.0, 0.0) = coast.
            # Both IN pins at duty 0 -> DRV8833 outputs float; reset state.
            motor_left.coast()
            motor_right.coast()
            integ_l = 0.0
            integ_r = 0.0
            shared["duty_l"] = 0.0
            shared["duty_r"] = 0.0
        else:
            # One PI controller per wheel with anti-windup.
            err_l = tgt_l - rpm_l
            err_r = tgt_r - rpm_r

            integ_l += PID_KI * err_l * dt
            integ_r += PID_KI * err_r * dt
            # Anti-windup, part 1: clamp the integral term itself...
            integ_l = _clamp(integ_l, -PID_INTEGRAL_LIMIT, PID_INTEGRAL_LIMIT)
            integ_r = _clamp(integ_r, -PID_INTEGRAL_LIMIT, PID_INTEGRAL_LIMIT)

            out_l = PID_KP * err_l + integ_l
            out_r = PID_KP * err_r + integ_r

            # ...part 2: conditional integration -- if the output saturated in
            # the same direction as the error, roll back this cycle's integral.
            if (out_l > 1.0 and err_l > 0.0) or (out_l < -1.0 and err_l < 0.0):
                integ_l -= PID_KI * err_l * dt
                out_l = PID_KP * err_l + integ_l
            if (out_r > 1.0 and err_r > 0.0) or (out_r < -1.0 and err_r < 0.0):
                integ_r -= PID_KI * err_r * dt
                out_r = PID_KP * err_r + integ_r

            out_l = _clamp(out_l, -1.0, 1.0)
            out_r = _clamp(out_r, -1.0, 1.0)

            motor_left.set(out_l)
            motor_right.set(out_r)
            shared["duty_l"] = out_l
            shared["duty_r"] = out_r

        shared["meas_l"] = rpm_l
        shared["meas_r"] = rpm_r

        # --- drift-free 50 ms schedule ---------------------------------------
        remaining = time.ticks_diff(next_tick, time.ticks_us())
        if remaining > 0:
            time.sleep_us(remaining)
        next_tick = time.ticks_add(next_tick, period_us)


# --------------------------------- Boot --------------------------------------
def main():
    print("=== ESP32-S3 differential-drive bot firmware (MicroPython) ===")
    motor_left.coast()       # start coasted, whatever the reset state was
    motor_right.coast()
    wifi_connect()
    shared["last_cmd_ms"] = time.ticks_ms()

    _thread.stack_size(8 * 1024)   # network thread stack (bytes)
    _thread.start_new_thread(network_thread, ())

    pid_loop()               # main thread = 50 ms PI controller (never returns)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("[STOP] KeyboardInterrupt -- coasting motors")
    except Exception as exc:
        print("[FAULT]", exc, "-- coasting motors")
    finally:
        # Any exit path (exception, Ctrl-C at the REPL, watchdog-driven REPL
        # return) leaves the DRV8833 in coast: both IN pins at duty 0.
        motor_left.coast()
        motor_right.coast()
