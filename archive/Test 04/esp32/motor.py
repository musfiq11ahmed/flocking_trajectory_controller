"""
motor.py — DRV8833 dual H-bridge motor driver for MicroPython on ESP32-S3.

DRV8833 control truth-table (slow-decay PWM):
    Forward :  IN1 = PWM (duty),  IN2 = LOW  (duty 0)
    Reverse :  IN1 = LOW  (duty 0),  IN2 = PWM (duty)
    Brake   :  IN1 = HIGH, IN2 = HIGH  (both duty 1023)
    Coast   :  IN1 = LOW,  IN2 = LOW   (both duty 0)
"""

from machine import Pin, PWM


class Motor:
    """Control one channel (A or B) of a DRV8833.

    Parameters
    ----------
    in1_pin, in2_pin : int
        GPIO numbers for xIN1 and xIN2 of the driver.
    freq : int
        PWM frequency in Hz (20 kHz avoids audible whine).
    inverted : bool
        If True, swap forward / reverse direction.
    """

    def __init__(self, in1_pin, in2_pin, freq=1000, inverted=False):
        self.pwm1 = PWM(Pin(in1_pin), freq=freq, duty=0)
        self.pwm2 = PWM(Pin(in2_pin), freq=freq, duty=0)
        self.inverted = inverted

    def set_speed(self, speed):
        """Set motor speed.

        Parameters
        ----------
        speed : int
            −1023 … +1023.  Positive = forward, negative = reverse.
            (Direction sense is flipped if ``inverted=True``.)
        """
        speed = int(speed)
        speed = max(-1023, min(1023, speed))

        if self.inverted:
            speed = -speed

        if speed > 0:
            self.pwm1.duty(speed)
            self.pwm2.duty(0)
        elif speed < 0:
            self.pwm1.duty(0)
            self.pwm2.duty(-speed)
        else:
            # Coast (both LOW)
            self.pwm1.duty(0)
            self.pwm2.duty(0)

    def brake(self):
        """Active braking (both outputs HIGH)."""
        self.pwm1.duty(1023)
        self.pwm2.duty(1023)

    def coast(self):
        """Free-wheel (both outputs LOW)."""
        self.pwm1.duty(0)
        self.pwm2.duty(0)


class DRV8833:
    """Convenience wrapper around two Motor channels + the nSLEEP pin.

    Parameters
    ----------
    ain1, ain2 : int   — GPIO for motor A (left)
    bin1, bin2 : int   — GPIO for motor B (right)
    nsleep     : int | None  — GPIO for nSLEEP (optional; tie to 3.3 V otherwise)
    left_inv, right_inv : bool — swap direction for each motor
    """

    def __init__(self, ain1, ain2, bin1, bin2,
                 nsleep=None, left_inv=False, right_inv=False):
        self.left  = Motor(ain1, ain2, inverted=left_inv)
        self.right = Motor(bin1, bin2, inverted=right_inv)

        self._nsleep = None
        if nsleep is not None:
            self._nsleep = Pin(nsleep, Pin.OUT)

        self.enable()

    def enable(self):
        """Wake the driver (nSLEEP HIGH)."""
        if self._nsleep:
            self._nsleep.value(1)

    def disable(self):
        """Put the driver to sleep (nSLEEP LOW) — motors coast."""
        if self._nsleep:
            self._nsleep.value(0)

    def stop(self):
        """Coast both motors."""
        self.left.coast()
        self.right.coast()
