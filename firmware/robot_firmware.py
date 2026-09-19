"""
robot_firmware_v2.py -- MicroPython firmware for the ESP32-S3 differential-drive bot.
====================================================================================
DIAGNOSTICS BUILD. The UDP wire protocol is BYTE-IDENTICAL to v1
(command '<ff' 8 bytes, telemetry '<ffiiff' 24 bytes), so pc/udp_protocol.py
does not need any changes. Deploy as main.py exactly like v1.

What v2 adds (debuggability only -- no behavior change):
  1. [NET] prints: first-command announcement, rx_ok/rx_bad counters and current
     targets/duties every 2 s. You can now SEE whether commands arrive.
  2. [FAILSAFE] prints when the 500 ms command-loss coast engages/clears.
  3. [NET FAULT] print if the network thread crashes (v1 died silently).
  4. SO_REUSEADDR on the UDP socket (survives re-running main.py without reset).
  5. recvfrom buffer raised 8 -> 64 bytes: an oversized/misformatted datagram is
     now counted as rx_bad instead of being silently truncated into a
     plausible-looking command.
  6. NaN guard: a PC-side format bug (wrong endianness/width) can unpack as
     NaN, which slips through _clamp and lands Motor.set() in its coast
     branch FOREVER with a clean log. NaN packets now count as rx_bad.
     (Denormal decodes remain visible via the raw-bytes print and the
     full-precision tgt values in the [NET] report.)
  7. Encoder keeps a reference to the Phase A Pin object (v1 left it as a local
     in __init__; keeping the object alive is required for the IRQ to stay
     registered).

NOTE: even with SO_REUSEADDR, never re-run main.py on a live VM
(exec/mpremote run without reset): the previous network thread stays alive
and keeps writing to the OLD shared dict while the new control loop reads
the NEW one -> commands received, motors dead. Always soft-reset first:
    mpremote connect COM13 reset

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

  NOTE: a total encoder failure CANNOT stop the wheels -- the PI loop would
  just see 0 RPM and saturate the duty at +/-1.0. If your wheels do not move
  at all, the encoders are NOT the cause; check command delivery and the
  DRV8833 power path first (run motor_test.py).

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
WIFI_SSID = "ASUS_1E_NIRO_2.4G"
WIFI_PASSWORD = "niro@2026"
WIFI_CONNECT_TIMEOUT_MS = 15000   # per-attempt timeout before printing a retry

# --- UDP protocol ------------------------------------------------------------
UDP_PORT = 4210                   # command listener + telemetry source port
TELEMETRY_PERIOD_MS = 50          # 20 Hz telemetry stream
FAILSAFE_TIMEOUT_MS = 500         # command-loss timeout -> coast
NET_REPORT_PERIOD_MS = 2000       # v2: [NET] diagnostics print cadence

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
PID_KFF = 0.0148                  # v6: feedforward, duty per RPM of TARGET.
                                  # Measured on this bot: 27.9 RPM @ 0.40 duty
                                  # and 61.0 RPM @ 0.90 duty -> 0.0148 duty/RPM.
                                  # Applies the expected duty instantly so the
                                  # PI only trims the residual; cuts the 2-3 s
                                  # stiction wind-up delay in step response.
PID_INTEGRAL_LIMIT = 1.0          # anti-windup clamp on the integral term
PID_LAUNCH_P_CAP = 0.20           # v9: cap the P term while |err| > ERR_FAR so
                                  # a step cannot saturate the output for a
                                  # cycle (one 0.98-duty cycle = +13 RPM spike)
PID_ERR_FAR_RPM = 5.0             # "far from target" threshold for the P cap
PID_STALL_OUT_CAP = 0.65          # v9: while stalled, output is capped just
                                  # above the ~0.5-0.6 breakaway threshold --
                                  # enough to start the wheel, never enough
                                  # for the 0.98-duty launch spike
PID_FF_HOLD_RPM = 2.0             # (removed in v10 -- conditional FF caused
                                  # limit-cycle oscillation; kept for reference)

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

# --- Direction correction (v5) ----------------------------------------------
# If a wheel spins physically BACKWARD for a positive duty (motor leads
# swapped at the DRV8833), set that side's MOTOR_INVERT to True.
# If a wheel spins forward but its ticks DECREASE (encoder A/B swapped),
# set that side's ENCODER_INVERT to True.
# Goal convention: positive RPM command -> wheel spins robot-forward AND
# ticks increase. Diagnose with the REPL snippet in README/RUNBOOK.
MOTOR_INVERT_L = False   # v7: motor wiring is FINE. v5 guessed motor polarity
                         # but the bot spun in place under "both forward" --
                         # proving the left motor turns forward for +duty and
                         # it is the ENCODER that reads inverted.
MOTOR_INVERT_R = False
ENCODER_INVERT_L = True  # left encoder counts backwards relative to motion
ENCODER_INVERT_R = False
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
    "remote": None,         # (ip, port) of last command sender; telemetry target
    "ticks_l": 0,           # encoder totals, written by IRQ handlers
    "ticks_r": 0,
    "meas_l": 0.0,          # measured RPM, written by PID loop
    "meas_r": 0.0,
    "duty_l": 0.0,          # signed duty actually applied (-1..1)
    "duty_r": 0.0,
}

# v2: network diagnostics counters (written by the network thread).
diag = {
    "rx_ok": 0,             # valid 8-byte command packets received
    "rx_bad": 0,            # datagrams with the wrong size (format mismatch)
    "tx": 0,                # telemetry packets actually sent
}

# v3: WLAN handle kept globally so the [NET] report can print live RSSI.
_wlan = None

# v4: UDP socket created in main() and shared: the network thread only
# receives, the PID loop sends telemetry on its precise 50 ms cadence.
_sock = None


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

    def __init__(self, pin_fwd, pin_rev, invert=False):
        self._invert = invert
        self.pwm_fwd = PWM(Pin(pin_fwd, Pin.OUT), freq=PWM_FREQ_HZ, duty_u16=0)
        self.pwm_rev = PWM(Pin(pin_rev, Pin.OUT), freq=PWM_FREQ_HZ, duty_u16=0)

    def set(self, duty):
        duty = _clamp(duty, -1.0, 1.0)
        if self._invert:
            duty = -duty
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

    def __init__(self, pin_a, pin_b, count_key, invert=False):
        self._phase_b = Pin(pin_b, Pin.IN)
        self._key = count_key
        self._step = -1 if invert else 1
        # v2 fix: keep a reference to the Phase A pin. The IRQ registration
        # lives on the Pin object; in v1 it was a local that went out of
        # scope as soon as __init__ returned.
        self._phase_a = Pin(pin_a, Pin.IN)
        self._phase_a.irq(handler=self._on_a_rising,
                          trigger=Pin.IRQ_RISING)   # soft IRQ: VM callback (see header)

    def _on_a_rising(self, pin):
        if self._phase_b.value():
            shared[self._key] -= self._step
        else:
            shared[self._key] += self._step


# Hardware objects exist at module level so the try/finally in main() can
# always coast the motors even if boot fails partway through.
motor_left = Motor(PIN_L_FWD, PIN_L_REV, invert=MOTOR_INVERT_L)
motor_right = Motor(PIN_R_FWD, PIN_R_REV, invert=MOTOR_INVERT_R)
enc_left = Encoder(PIN_ENC_L_A, PIN_ENC_L_B, "ticks_l", invert=ENCODER_INVERT_L)
enc_right = Encoder(PIN_ENC_R_A, PIN_ENC_R_B, "ticks_r", invert=ENCODER_INVERT_R)


# ------------------------------- WiFi ----------------------------------------
def wifi_connect():
    """Station-mode connect with timeout + retry; prints the IP on success."""
    wlan = network.WLAN(network.STA_IF)
    wlan.active(True)
    # Disable WiFi modem sleep. The default power-save mode lets the radio
    # doze between beacons, so outgoing UDP telemetry gets queued and
    # delivered in bursts (observed: 20 Hz stream arriving at ~2 Hz with
    # 400-900 ms gaps). PM_NONE keeps the radio awake: full telemetry rate,
    # low latency. Slightly higher power draw -- irrelevant on a robot.
    try:
        wlan.config(pm=wlan.PM_NONE)
        print("[WiFi] power-save disabled (pm=PM_NONE)")
    except (ValueError, AttributeError):
        # Older MicroPython builds lack PM_NONE; PM_PERFORMANCE (min-modem
        # sleep) is the next best.
        try:
            wlan.config(pm=wlan.PM_PERFORMANCE)
            print("[WiFi] pm=PM_PERFORMANCE (PM_NONE unsupported)")
        except (ValueError, AttributeError):
            print("[WiFi] WARNING: cannot set pm -- expect telemetry jitter")
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
    try:
        print("[WiFi] RSSI = %d dBm  (good: > -65, ok: -65..-75, bad: < -75)"
              % wlan.status("rssi"))
    except (OSError, ValueError):
        pass
    print("[UDP ] listening on port", UDP_PORT)
    return wlan


# ------------------------ Network thread (Core 0 role) -----------------------
def network_thread():
    """UDP command listener (RX only, v4).

    v4 restructure: this thread does NOTHING but blocking receive. Telemetry
    moved into the PID loop, which owns a precise 50 ms cadence -- previously
    every recvfrom timeout here directly delayed telemetry, capping the link
    at ~13 Hz. Blocking recv releases the GIL, so the PID loop runs freely;
    incoming commands are handled the instant lwIP delivers them.

    Counts valid/invalid datagrams and prints a status line every
    NET_REPORT_PERIOD_MS. Any crash is printed as [NET FAULT] instead of
    killing the thread silently.
    """
    try:
        s = _sock
        s.settimeout(0.25)     # wake 4x/s for the report even with no traffic

        last_report = time.ticks_ms()
        print("[NET ] listening on 0.0.0.0:%d -- waiting for first command"
              % UDP_PORT)

        while True:
            # --- receive -----------------------------------------------------
            # Buffer is 64 bytes, NOT 8: an oversized/misformatted datagram
            # must surface as rx_bad, not be truncated into a false-valid
            # 8-byte command.
            try:
                data, addr = s.recvfrom(64)
            except OSError:
                data, addr = None, None  # timeout: no packet this slice

            if data is not None:
                if len(data) == COMMAND_SIZE:
                    left, right = struct.unpack(COMMAND_FORMAT, data)
                    if left != left or right != right:        # NaN guard
                        # A PC-side packing bug (wrong endianness/width) can
                        # decode as NaN; NaN slips through _clamp and makes
                        # Motor.set() coast forever. Reject it loudly.
                        diag["rx_bad"] += 1
                        if diag["rx_bad"] == 1:
                            print("[NET ] NaN command, raw=", data,
                                  "-- check PC struct.pack('<ff', ...)")
                    else:
                        shared["target_l"] = _clamp(left, -MAX_RPM, MAX_RPM)
                        shared["target_r"] = _clamp(right, -MAX_RPM, MAX_RPM)
                        shared["last_cmd_ms"] = time.ticks_ms()
                        if shared["remote"] is None:
                            # Raw bytes included: kills "format mismatch?"
                            # guesswork in one look.
                            print("[NET ] first command from %s:%d raw=%s "
                                  "-> tgt=(%s, %s) RPM"
                                  % (addr[0], addr[1], data, left, right))
                        shared["remote"] = addr  # telemetry -> last sender
                        diag["rx_ok"] += 1
                else:
                    diag["rx_bad"] += 1   # wrong size -> PC format mismatch
                    if diag["rx_bad"] == 1:
                        print("[NET ] bad-size datagram (%d bytes), raw=%s "
                              "-- PC must send exactly 8 bytes '<ff'"
                              % (len(data), data))

            # --- diagnostics @ 0.5 Hz ------------------------------------------
            now = time.ticks_ms()
            if time.ticks_diff(now, last_report) >= NET_REPORT_PERIOD_MS:
                last_report = now
                remote = shared["remote"]
                rssi = "n/a"
                if _wlan is not None:
                    try:
                        rssi = "%ddBm" % _wlan.status("rssi")
                    except (OSError, ValueError):
                        pass
                print("[NET ] rx_ok=%d rx_bad=%d tx=%d rssi=%s tgt=(%.1f, %.1f) "
                      "duty=(%.2f, %.2f) remote=%s"
                      % (diag["rx_ok"], diag["rx_bad"], diag["tx"], rssi,
                         shared["target_l"], shared["target_r"],
                         shared["duty_l"], shared["duty_r"],
                         "%s:%d" % (remote[0], remote[1])
                         if remote is not None else "none"))
    except Exception as exc:
        # v1 died silently here and the robot simply never responded again.
        print("[NET FAULT]", exc, "-- network thread died; reset the ESP32")


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
    failsafe_active = False       # v2: for engage/clear prints

    period_us = PID_PERIOD_MS * 1000
    next_tick = time.ticks_add(time.ticks_us(), period_us)
    last_telemetry = time.ticks_ms()

    while True:
        # --- failsafe: no valid command for >500 ms -> zero targets ---------
        # Zero targets are handled below as "coast", so a lost link stops the
        # motors within one PID period of the timeout.
        if time.ticks_diff(time.ticks_ms(), shared["last_cmd_ms"]) > \
                FAILSAFE_TIMEOUT_MS:
            if not failsafe_active:
                failsafe_active = True
                print("[FAILSAFE] no command for >%d ms -- coasting"
                      % FAILSAFE_TIMEOUT_MS)
            shared["target_l"] = 0.0
            shared["target_r"] = 0.0
        elif failsafe_active:
            failsafe_active = False
            print("[FAILSAFE] commands received -- resuming control")

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

            # v9 anti-windup, part 0: STALL GUARD. This drivetrain needs ~0.5
            # duty to break static friction; while the wheel is stuck, the
            # error stays large and a full-rate integrator would charge to
            # the clamp, then the wheel lurches forward on breakaway.
            d_i_l = PID_KI * err_l * dt
            d_i_r = PID_KI * err_r * dt
            stalled_l = abs(rpm_l) < 2.0 and abs(tgt_l) > 2.0
            stalled_r = abs(rpm_r) < 2.0 and abs(tgt_r) > 2.0
            # v9: stalled wheels integrate at 1/4 rate -- enough to guarantee
            # breakaway even at low targets (where FF+P sits just under the
            # stiction threshold), slow enough to prevent the lurch windup.
            if stalled_l:
                d_i_l *= 0.25
            if stalled_r:
                d_i_r *= 0.25
            integ_l += d_i_l
            integ_r += d_i_r
            # Anti-windup, part 1: clamp the integral term itself...
            integ_l = _clamp(integ_l, -PID_INTEGRAL_LIMIT, PID_INTEGRAL_LIMIT)
            integ_r = _clamp(integ_r, -PID_INTEGRAL_LIMIT, PID_INTEGRAL_LIMIT)

            # v10: FEEDFORWARD IS ALWAYS ON. The v8/v9 conditional FF caused a
            # relaxation oscillation (seen in the 153154 log: every time RPM
            # crossed ~32, FF cut -> duty collapsed 0.6->0.1 -> RPM plunged ->
            # FF back -> repeat, never staying in the +/-10% band). Its original
            # job -- un-masking the brake after a launch spike -- is now done
            # by the stall output cap, which prevents the spike entirely.
            ff_l = PID_KFF * tgt_l
            ff_r = PID_KFF * tgt_r

            # v9: cap the P "kick" while far from target. A 30 RPM step puts
            # KP*err = 0.6 duty on top of FF -> output saturates at ~1.0 for
            # one 50 ms cycle -> wheel spikes to 43 RPM. The integrator is
            # NOT capped, so friction compensation is unaffected.
            p_l = PID_KP * err_l
            p_r = PID_KP * err_r
            # (cap skipped while stalled: full P is needed to break stiction)
            if abs(err_l) > PID_ERR_FAR_RPM and not stalled_l:
                p_l = _clamp(p_l, -PID_LAUNCH_P_CAP, PID_LAUNCH_P_CAP)
            if abs(err_r) > PID_ERR_FAR_RPM and not stalled_r:
                p_r = _clamp(p_r, -PID_LAUNCH_P_CAP, PID_LAUNCH_P_CAP)

            out_l = p_l + integ_l + ff_l
            out_r = p_r + integ_r + ff_r

            # ...part 2: conditional integration -- if the output saturated in
            # the same direction as the error, roll back this cycle's integral.
            if (out_l > 1.0 and err_l > 0.0) or (out_l < -1.0 and err_l < 0.0):
                integ_l -= PID_KI * err_l * dt
                out_l = p_l + integ_l + ff_l
            if (out_r > 1.0 and err_r > 0.0) or (out_r < -1.0 and err_r < 0.0):
                integ_r -= PID_KI * err_r * dt
                out_r = p_r + integ_r + ff_r

            # v9: stall output cap -- while the wheel has not started, never
            # apply more than breakaway duty (kills the launch spike).
            if stalled_l:
                out_l = _clamp(out_l, -PID_STALL_OUT_CAP, PID_STALL_OUT_CAP)
            if stalled_r:
                out_r = _clamp(out_r, -PID_STALL_OUT_CAP, PID_STALL_OUT_CAP)

            out_l = _clamp(out_l, -1.0, 1.0)
            out_r = _clamp(out_r, -1.0, 1.0)

            motor_left.set(out_l)
            motor_right.set(out_r)
            shared["duty_l"] = out_l
            shared["duty_r"] = out_r

        shared["meas_l"] = rpm_l
        shared["meas_r"] = rpm_r

        # --- telemetry @ PID cadence (v4) -------------------------------------
        # Sent from here, not the network thread: this loop owns the precise
        # 50 ms schedule, so telemetry rate no longer depends on RX timeouts.
        remote = shared["remote"]
        if remote is not None and \
                time.ticks_diff(time.ticks_ms(), last_telemetry) >= \
                TELEMETRY_PERIOD_MS:
            last_telemetry = time.ticks_ms()
            pkt = struct.pack(
                TELEMETRY_FORMAT,
                rpm_l,
                rpm_r,
                _to_int32(shared["ticks_l"]),
                _to_int32(shared["ticks_r"]),
                shared["duty_l"],
                shared["duty_r"],
            )
            try:
                _sock.sendto(pkt, remote)
                diag["tx"] += 1
            except OSError:
                pass             # WiFi hiccup: drop one frame, keep going

        # --- drift-free 50 ms schedule ---------------------------------------
        # time.sleep_us() on the ESP32 port is a BUSY-WAIT that never releases
        # the GIL: spinning ~50 ms per cycle here starved the network thread
        # and collapsed the 20 Hz UDP link to ~3 Hz. Sleep the bulk of the
        # period with sleep_ms (vTaskDelay -> releases the GIL so the network
        # thread runs), then spin only the last few ms for deadline precision.
        remaining = time.ticks_diff(next_tick, time.ticks_us())
        if remaining > 4000:
            time.sleep_ms((remaining - 3000) // 1000)
        while time.ticks_diff(next_tick, time.ticks_us()) > 0:
            pass
        next_tick = time.ticks_add(next_tick, period_us)


# --------------------------------- Boot --------------------------------------
def main():
    print("=== ESP32-S3 diff-drive bot firmware v10 (stall cap + steady FF) ===")
    print("[CFG ] MOTOR_INVERT L=%s R=%s  ENCODER_INVERT L=%s R=%s"
          % (MOTOR_INVERT_L, MOTOR_INVERT_R,
             ENCODER_INVERT_L, ENCODER_INVERT_R))
    motor_left.coast()       # start coasted, whatever the reset state was
    motor_right.coast()
    global _wlan, _sock
    _wlan = wifi_connect()

    # One shared UDP socket: network thread receives, PID loop sends
    # telemetry. (lwIP UDP sockets tolerate concurrent recv/send.)
    _sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    try:
        # Lets us re-run main.py (or rebind after a crash) without a
        # soft reset, instead of dying with EADDRINUSE.
        _sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    except (OSError, AttributeError):
        pass                     # port without SO_REUSEADDR: harmless
    _sock.bind(("0.0.0.0", UDP_PORT))

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
