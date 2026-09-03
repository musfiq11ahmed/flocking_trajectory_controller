"""
robot.py — High-level robot controller for ESP32-S3.

Responsibilities
----------------
* Read serial commands from the PC  (target wheel speeds in ticks/s).
* Read quadrature encoders via pin interrupts.
* Run independent PID loops for left and right wheels.
* Drive the DRV8833 motors.
* Send encoder feedback back to the PC.
* Watchdog: stop motors if no command arrives within WATCHDOG_MS.
"""

import sys
import time
import select

from machine import Pin
from pid import PID
from motor import DRV8833


# ─────────────────────────────────────────────────────────────
#  Configuration  (edit to match your wiring / motor specs)
# ─────────────────────────────────────────────────────────────

# Motor driver pins (DRV8833)
AIN1_PIN   = 4
AIN2_PIN   = 5
BIN1_PIN   = 6
BIN2_PIN   = 7
NSLEEP_PIN = None      # Tied to 3.3 V (STBY)

# Encoder pins (channel A + B per wheel)
ENC_L_A    = 1
ENC_L_B    = 2
ENC_R_A    = 9
ENC_R_B    = 10

# Motor direction — set True to swap forward/reverse for a motor
LEFT_INVERTED  = False
RIGHT_INVERTED = True

# PID gains (tuned for smooth, jitter-free N20 motor velocity control)
KP = 0.6
KI = 0.1
KD = 0.0      # Zero to prevent discrete encoder quantization chattering
KF = 0.45     # Feed-forward gain to provide instant, smooth baseline PWM

# Loop timing
LOOP_MS        = 20          # PID loop period → 50 Hz
FEEDBACK_EVERY = 5           # send feedback every N loops (≈10 Hz)
WATCHDOG_MS    = 500         # stop motors if no command for this long

# Speed deadzone — below this target, just brake
DEADZONE_TPS   = 5.0         # ticks/s


# ─────────────────────────────────────────────────────────────
#  Quadrature Encoder (interrupt-based, 2× decoding)
# ─────────────────────────────────────────────────────────────

class Encoder:
    """Interrupt-driven quadrature encoder (2× counting on ch-A edges).

    Direction is determined by comparing A and B at each A-edge.
    """

    def __init__(self, pin_a_num, pin_b_num):
        self._pin_a = Pin(pin_a_num, Pin.IN, Pin.PULL_UP)
        self._pin_b = Pin(pin_b_num, Pin.IN, Pin.PULL_UP)
        self._count = 0
        self._pin_a.irq(trigger=Pin.IRQ_RISING | Pin.IRQ_FALLING,
                        handler=self._isr)

    def _isr(self, _):
        # At each A transition: if A == B → one direction, else → other
        if self._pin_a.value() == self._pin_b.value():
            self._count += 1
        else:
            self._count -= 1

    @property
    def count(self):
        return self._count

    def reset(self):
        self._count = 0


# ─────────────────────────────────────────────────────────────
#  Robot controller
# ─────────────────────────────────────────────────────────────

class Robot:
    def __init__(self):
        # Motor driver
        self.driver = DRV8833(
            AIN1_PIN, AIN2_PIN, BIN1_PIN, BIN2_PIN,
            nsleep=NSLEEP_PIN,
            left_inv=LEFT_INVERTED,
            right_inv=RIGHT_INVERTED,
        )

        # Encoders
        self.enc_left  = Encoder(ENC_L_A, ENC_L_B)
        self.enc_right = Encoder(ENC_R_A, ENC_R_B)

        # PID controllers (PI + Feed-Forward for smooth velocity tracking)
        self.pid_left  = PID(kp=KP, ki=KI, kd=KD, kf=KF)
        self.pid_right = PID(kp=KP, ki=KI, kd=KD, kf=KF)

        # Target speeds (ticks/s, set by serial commands from PC)
        self.target_left  = 0.0
        self.target_right = 0.0

        # Measured speeds (ticks/s)
        self.speed_left  = 0.0
        self.speed_right = 0.0

        # Book-keeping
        self._prev_enc_l = 0
        self._prev_enc_r = 0
        self._last_cmd_ms = time.ticks_ms()
        self._loop_count  = 0

        # Non-blocking stdin reader
        self._poll = select.poll()
        self._poll.register(sys.stdin, select.POLLIN)
        self._rx_buf = ""

    # ── main loop ─────────────────────────────────────────────

    def run(self):
        """Enter the main control loop (never returns normally)."""
        # Signal to the PC that we're ready
        sys.stdout.write("READY\n")

        while True:
            t0 = time.ticks_ms()

            # 1. Read serial commands (non-blocking)
            self._read_serial()

            # 2. Watchdog — stop if no recent command
            if time.ticks_diff(time.ticks_ms(), self._last_cmd_ms) > WATCHDOG_MS:
                self.stop()

            # 3. Measure wheel speeds with low-pass filtering to eliminate quantization chatter
            dt_s = LOOP_MS / 1000.0
            enc_l = self.enc_left.count
            enc_r = self.enc_right.count

            delta_l = enc_l - self._prev_enc_l
            delta_r = enc_r - self._prev_enc_r
            self._prev_enc_l = enc_l
            self._prev_enc_r = enc_r

            raw_speed_l = delta_l / dt_s
            raw_speed_r = delta_r / dt_s

            # Low-pass filter (exponential moving average: 65% previous + 35% new)
            self.speed_left  = 0.65 * self.speed_left  + 0.35 * raw_speed_l
            self.speed_right = 0.65 * self.speed_right + 0.35 * raw_speed_r

            # 4. PID update → motor PWM
            if abs(self.target_left) < DEADZONE_TPS:
                pwm_l = 0
                self.pid_left.reset()
                self.driver.left.coast()
            else:
                pwm_l = self.pid_left.update(self.target_left, self.speed_left, dt_s)
                self.driver.left.set_speed(int(pwm_l))

            if abs(self.target_right) < DEADZONE_TPS:
                pwm_r = 0
                self.pid_right.reset()
                self.driver.right.coast()
            else:
                pwm_r = self.pid_right.update(self.target_right, self.speed_right, dt_s)
                self.driver.right.set_speed(int(pwm_r))

            # 5. Send feedback to PC (at reduced rate)
            self._loop_count += 1
            if self._loop_count >= FEEDBACK_EVERY:
                self._loop_count = 0
                sys.stdout.write(
                    "FB:{},{},{:.1f},{:.1f}\n".format(
                        enc_l, enc_r, self.speed_left, self.speed_right
                    )
                )

            # 6. Wait for next loop tick
            elapsed = time.ticks_diff(time.ticks_ms(), t0)
            if elapsed < LOOP_MS:
                time.sleep_ms(LOOP_MS - elapsed)

    # ── serial I/O ────────────────────────────────────────────

    def _read_serial(self):
        """Non-blocking read of commands from USB serial (stdin)."""
        while self._poll.poll(0):
            ch = sys.stdin.read(1)
            if ch is None:
                break
            if ch == "\n":
                line = self._rx_buf.strip()
                self._rx_buf = ""
                self._parse_command(line)
            else:
                self._rx_buf += ch

    def _parse_command(self, line):
        """Parse a CMD:<vl>,<vr> or STOP command."""
        if line == "STOP":
            self.stop()
            self._last_cmd_ms = time.ticks_ms()
            return
        if not line.startswith("CMD:"):
            return
        try:
            parts = line[4:].split(",")
            self.target_left  = float(parts[0])
            self.target_right = float(parts[1])
            self._last_cmd_ms = time.ticks_ms()
            if abs(self.target_left) < DEADZONE_TPS and abs(self.target_right) < DEADZONE_TPS:
                self.stop()
        except (ValueError, IndexError):
            pass   # ignore malformed commands

    # ── cleanup ───────────────────────────────────────────────

    def stop(self):
        """Emergency stop — coast both motors."""
        self.target_left  = 0.0
        self.target_right = 0.0
        self.driver.stop()
        self.pid_left.reset()
        self.pid_right.reset()
