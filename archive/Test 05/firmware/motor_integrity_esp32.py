"""Motor + encoder integrity check for the ESP32-S3 diff-drive bot.

Runs entirely ON the ESP32 (no WiFi, no PC-side code). For each motor:

  1. DEADBAND SWEEP -- ramps duty 0.20 -> 1.00 and reports the minimum duty
     that actually starts the wheel (stiction measure). N20 gearmotors often
     need 0.3-0.5; much higher suggests low battery, drag, or a worn motor.
  2. SIGN CHAIN -- from the first sweep step that moved: +duty must give
     +ticks (the convention the PID firmware needs).
  3. FWD/REV at 0.40 duty WITH a 0.25 s full-duty kick to break stiction --
     checks speed symmetry once the wheel is already turning.
  4. MAX-RPM probe at 0.90 duty.

Ends with the exact MOTOR_INVERT_x / ENCODER_INVERT_x flags to set in
robot_firmware.py if a sign chain is inverted.

HOW TO RUN (robot LIFTED, wheels free, motor battery ON):
    mpremote connect COM13 run firmware/motor_integrity_esp32.py

WATCH THE WHEELS during the sweep: for each side, note whether the tire
spins robot-FORWARD or robot-BACKWARD when it first starts moving -- the
script tells you the tick sign, only your eyes can tell the physical
direction, and the fix depends on the combination.

This script does NOT touch main.py -- nothing is installed or overwritten.
"""

import time
from machine import Pin, PWM

# --- Pins (must match robot_firmware.py) ------------------------------------
PIN_L_FWD = 4
PIN_L_REV = 5
PIN_R_FWD = 6
PIN_R_REV = 7
PIN_ENC_L_A = 1
PIN_ENC_L_B = 2
PIN_ENC_R_A = 9
PIN_ENC_R_B = 10

PWM_FREQ_HZ = 20000
ENCODER_TICKS_PER_REV = 1800.0    # 12 CPR x 1:150 gearbox, 1x decoding

TEST_DUTY = 0.40                  # symmetry test level (with kick)
FULL_DUTY = 0.90                  # max-RPM probe
KICK_S = 0.25                     # full-duty pulse to break stiction
PHASE_S = 2.0
SETTLE_S = 1.0
SWEEP_STEP_S = 0.7                # drive time per deadband step
SWEEP_MIN_TICKS = 80              # "it moved" threshold per sweep step
MIN_TICKS = 150                   # "motion" threshold per 2 s phase

ticks = {"L": 0, "R": 0}
_enc_b = {}


def _make_encoder(side, pin_a, pin_b):
    _enc_b[side] = Pin(pin_b, Pin.IN)

    def _irq(pin, _side=side):
        if _enc_b[_side].value():
            ticks[_side] -= 1
        else:
            ticks[_side] += 1

    a = Pin(pin_a, Pin.IN)
    a.irq(handler=_irq, trigger=Pin.IRQ_RISING)
    return a                      # keep a reference so the IRQ stays live


class Motor:
    def __init__(self, pin_fwd, pin_rev):
        self.fwd = PWM(Pin(pin_fwd, Pin.OUT), freq=PWM_FREQ_HZ, duty_u16=0)
        self.rev = PWM(Pin(pin_rev, Pin.OUT), freq=PWM_FREQ_HZ, duty_u16=0)

    def set(self, duty):
        d = int(abs(duty) * 65535 + 0.5)
        if duty > 0:
            self.fwd.duty_u16(d)
            self.rev.duty_u16(0)
        elif duty < 0:
            self.fwd.duty_u16(0)
            self.rev.duty_u16(d)
        else:
            self.fwd.duty_u16(0)
            self.rev.duty_u16(0)


motors = {"L": Motor(PIN_L_FWD, PIN_L_REV), "R": Motor(PIN_R_FWD, PIN_R_REV)}
_enc_a_l = _make_encoder("L", PIN_ENC_L_A, PIN_ENC_L_B)
_enc_a_r = _make_encoder("R", PIN_ENC_R_A, PIN_ENC_R_B)


def coast_all():
    motors["L"].set(0)
    motors["R"].set(0)


def drive_timed(side, duty, seconds, kick=False):
    """Drive one motor, return tick_delta measured over `seconds` at `duty`
    (the kick pulse itself is not measured)."""
    m = motors[side]
    if kick:
        m.set(1.0 if duty > 0 else -1.0)
        time.sleep(KICK_S)
    ticks[side] = 0
    m.set(duty)
    time.sleep(seconds)
    m.set(0)
    return ticks[side]


def rpm_of(dt, seconds):
    return dt / ENCODER_TICKS_PER_REV * (60.0 / seconds)


def sweep_deadband(side):
    """Ramp duty up until the wheel starts. Returns (duty or None, ticks)."""
    d10 = 20                       # 0.20 in tenths of a percent*10
    while d10 <= 100:
        d = d10 / 100.0
        dt = drive_timed(side, d, SWEEP_STEP_S)
        if abs(dt) >= SWEEP_MIN_TICKS:
            return d, dt
        d10 += 10
    return None, 0


def check_motor(side):
    print("\n--- Motor %s: deadband sweep (WATCH this wheel!) ---" % side)
    d_start, dt_start = sweep_deadband(side)
    if d_start is None:
        print("  no motion even at 100% duty")
        return {"motion": False, "sign": False, "symmetry": False,
                "rpm_f": 0.0, "rpm_r": 0.0, "rpm_max": 0.0,
                "sym_ratio": 0.0, "min_duty": None}
    print("  starts moving at duty %.2f (ticks %+d in %.1fs) --> %s"
          % (d_start, dt_start, SWEEP_STEP_S,
             "ticks POSITIVE (correct)" if dt_start > 0
             else "ticks NEGATIVE (inverted chain)"))
    time.sleep(SETTLE_S)

    sign_ok = dt_start > 0

    dt_f = drive_timed(side, +TEST_DUTY, PHASE_S, kick=True)
    rpm_f = rpm_of(dt_f, PHASE_S)
    print("  duty %+0.2f (kick) -> ticks %+6d, rpm %+7.1f"
          % (TEST_DUTY, dt_f, rpm_f))
    time.sleep(SETTLE_S)
    dt_r = drive_timed(side, -TEST_DUTY, PHASE_S, kick=True)
    rpm_r = rpm_of(dt_r, PHASE_S)
    print("  duty %+0.2f (kick) -> ticks %+6d, rpm %+7.1f"
          % (-TEST_DUTY, dt_r, rpm_r))
    time.sleep(SETTLE_S)
    dt_m = drive_timed(side, +FULL_DUTY, PHASE_S)
    rpm_m = rpm_of(dt_m, PHASE_S)
    print("  duty %+0.2f        -> ticks %+6d, rpm %+7.1f  (max-speed probe)"
          % (FULL_DUTY, dt_m, rpm_m))
    time.sleep(SETTLE_S)

    motion = abs(dt_f) >= MIN_TICKS and abs(dt_r) >= MIN_TICKS
    sym = abs(rpm_f) / max(abs(rpm_r), 0.1) if motion else 0.0
    return {"motion": motion, "sign": sign_ok,
            "symmetry": 0.6 <= sym <= 1.67 if motion else False,
            "rpm_f": rpm_f, "rpm_r": rpm_r, "rpm_max": rpm_m,
            "sym_ratio": sym, "min_duty": d_start}


def verdict(side, r):
    print("  verdict %s:" % side)
    if r["min_duty"] is None:
        print("    NO MOTION AT ANY DUTY. Watch the wheel during the sweep:")
        print("    - wheel SPINS but ticks ~0  -> encoder wiring/channel dead")
        print("    - wheel STILL               -> battery off? DRV8833 wiring?")
        return "dead"
    if r["min_duty"] > 0.50:
        print("    HIGH STICTION: needs %.0f%% duty to start (healthy is ~20-40%%)."
              % (r["min_duty"] * 100))
        print("    Check battery charge and mechanical drag. Low-speed PID")
        print("    tracking will be jerky; the nav code's min-speed floor helps.")
    if not r["sign"]:
        print("    SIGN CHAIN INVERTED: +duty produced -ticks.")
        print("    If the wheel spun physically BACKWARD when it started: swap")
        print("    the two motor leads at the DRV8833 (or MOTOR_INVERT_%s = True)."
              % side)
        print("    If it spun FORWARD: encoder sense is flipped, set")
        print("    ENCODER_INVERT_%s = True in robot_firmware.py." % side)
        return "inverted"
    if r["motion"] and not r["symmetry"]:
        print("    ASYMMETRIC: fwd/rev ratio %.2f (expect 0.6-1.67)."
              % r["sym_ratio"])
        print("    Check for mechanical drag or a tired motor.")
        return "asymmetric"
    if not r["motion"]:
        print("    Sign OK but weak sustained motion at %.0f%% duty." %
              (TEST_DUTY * 100))
        return "weak"
    print("    OK  (starts at %.0f%% duty, fwd %+.1f rpm, rev %+.1f rpm,"
          % (r["min_duty"] * 100, r["rpm_f"], r["rpm_r"]))
    print("         est. max %+.1f rpm)" % r["rpm_max"])
    return "ok"


def main():
    print("=" * 60)
    print("MOTOR INTEGRITY CHECK  (open-loop, no PID, no WiFi)")
    print("!! LIFT THE ROBOT -- wheels spin in 5 s !!")
    print("=" * 60)
    for i in range(5, 0, -1):
        print("  starting in %d..." % i)
        time.sleep(1)

    try:
        results = {}
        status = {}
        for side in ("L", "R"):
            results[side] = check_motor(side)
            status[side] = verdict(side, results[side])

        print("\n=== SUMMARY ===")
        print("  left : %s" % status["L"])
        print("  right: %s" % status["R"])
        if status["L"] == "ok" and status["R"] == "ok":
            match = abs(results["L"]["rpm_f"]) / max(abs(results["R"]["rpm_f"]), 0.1)
            print("  L/R speed match at duty %.2f: ratio %.2f %s"
                  % (TEST_DUTY, match,
                     "(good)" if 0.65 <= match <= 1.54
                     else "(MISMATCH -- check motors)"))
            print("  Both chains consistent: keep all INVERT flags False.")
        else:
            print("  Fix the flagged side(s), then re-run this script,")
            print("  then re-run: python pc/test_suite.py --test 1 --ip <bot>")
    finally:
        coast_all()
        print("\nMotors coasted. Done.")


main()
