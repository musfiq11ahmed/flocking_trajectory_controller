"""
motor_test.py -- standalone open-loop drive test for the ESP32-S3 diff-drive bot.
====================================================================================
Purpose: decide in about one minute whether "wheels not moving at all" is a
HARDWARE problem (DRV8833 power path) or a SOFTWARE/NETWORK problem. It
bypasses WiFi, UDP, and the PI controller entirely: each DRV8833 half is
driven forward then reverse at 40% duty for 1 second while encoder Phase-A
edges are counted.

Run it from the PC (mpremote interrupts main.py's control loop first):

    mpremote connect COM13 run motor_test.py

If you are sitting inside a REPL instead, press Ctrl-C first so the PI loop
has released the PWM pins, then paste/exec this file.

How to read the result:
  - Wheel speed rises clearly with duty (e.g. ~40 RPM at 0.40 -> ~100+ RPM at
    1.00)  -> hardware is GOOD. Your bug is upstream: the PC is not
    delivering valid nonzero RPM commands (wrong/stale IP, wrong packet
    format, all-zero commands from the controller, dead network thread).
  - Wheel does NOT turn at ANY duty  -> 100% power path: DRV8833 VM (motor
    supply) missing, STBY/nSLEEP not at 3V3, missing common GND between
    ESP32 and driver, miswired AOUT/BOUT, or a dead driver/motor.
  - SLOW and/or INTERMITTENT rotation (starts sometimes, stalls sometimes,
    barely speeds up with duty)  -> MARGINAL MOTOR SUPPLY. The DRV8833 has
    an undervoltage lockout around 2.5-2.7 V on VM: if VM sags under load
    (powered from the ESP32's 3V3 pin, weak battery, thin jumper wires,
    breadboard rails), the driver cuts out and recovers repeatedly -- that
    IS the intermittent rotation. Fix the power path: VM direct from the
    battery/5-6 V with short thick wires, a 100-470 uF cap across VM/GND at
    the driver, solid nSLEEP-to-3V3 joint, common GND everywhere, and keep
    motor power OFF the breadboard.
  - Wheel turns but edge count is ~0 -> encoder wiring/power issue. Note:
    this alone would NOT stop the wheels (the PI loop just saturates the
    duty), so fix motion first, encoders second.

Reference numbers (N20 1:150, VM = 6 V, wheels off the ground): roughly
30 encoder edges/s per RPM, so duty 1.00 should give ~100-130 RPM, i.e.
~3000-4000 edges/s. Counts may under-report slightly at high speed (soft
IRQ), which is fine -- you are looking at the TREND across duties.

REMINDER: never re-run main.py on a live VM without a reset first
(mpremote connect COM13 reset) -- a zombie network thread from a previous
run can keep the UDP socket and fight your debugging.
"""

import time

from machine import PWM, Pin

PWM_FREQ_HZ = 20000
DUTIES = (0.40, 0.70, 1.00)   # duty sweep: speed MUST rise clearly with duty
SPIN_MS = 1000
SETTLE_MS = 300

# name, forward IN pin, reverse IN pin, encoder Phase A pin
MOTORS = (
    ("LEFT ", 4, 5, 1),
    ("RIGHT", 6, 7, 9),
)

_edges = [0]


def _count(pin):
    _edges[0] += 1


def spin(label, pin_on, pin_off, pin_enc_a, duty_frac):
    pwm_on = PWM(Pin(pin_on, Pin.OUT), freq=PWM_FREQ_HZ, duty_u16=0)
    pwm_off = PWM(Pin(pin_off, Pin.OUT), freq=PWM_FREQ_HZ, duty_u16=0)
    enc = Pin(pin_enc_a, Pin.IN)
    enc.irq(handler=_count, trigger=Pin.IRQ_RISING)
    _edges[0] = 0
    pwm_on.duty_u16(int(duty_frac * 65535))
    time.sleep_ms(SPIN_MS)
    pwm_on.duty_u16(0)         # coast
    eps = _edges[0] * 1000.0 / SPIN_MS
    print("  %s duty=%.2f -> %5d edges (%5.0f edges/s ~ %3.0f RPM)"
          % (label, duty_frac, _edges[0], eps, eps / 30.0))
    enc.irq(handler=None)


print("=== motor_test v2: open-loop duty sweep, 1 s per step ===")
print("=== wheels OFF the ground; watch whether speed rises with duty ===")
for name, fwd, rev, enc_a in MOTORS:
    for d in DUTIES:
        print(name, "forward duty %.2f:" % d)
        spin("fwd", fwd, rev, enc_a, d)
        time.sleep_ms(SETTLE_MS)
    print(name, "reverse duty 1.00:")
    spin("rev", rev, fwd, enc_a, 1.00)
    time.sleep_ms(SETTLE_MS)
print("=== done -- all outputs coasted ===")
