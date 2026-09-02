"""
pid.py — Discrete PID controller for MicroPython.

Each wheel gets its own PID instance.  The controller operates on
encoder ticks-per-second as both setpoint and measurement.
"""


class PID:
    """Incremental PID with anti-windup clamping.

    Parameters
    ----------
    kp, ki, kd : float
        Proportional, integral, and derivative gains.
    out_min, out_max : float
        Output clamp range (maps to PWM duty: −1023 … +1023).
    """

    def __init__(self, kp=2.0, ki=0.5, kd=0.05,
                 out_min=-1023, out_max=1023):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.out_min = out_min
        self.out_max = out_max

        self._integral   = 0.0
        self._prev_error = 0.0
        self._output     = 0.0

    def update(self, setpoint, measurement, dt):
        """Compute one PID step.

        Parameters
        ----------
        setpoint    : target speed (ticks/s)
        measurement : measured speed (ticks/s)
        dt          : time since last call (seconds)

        Returns
        -------
        output : float in [out_min, out_max]
        """
        if dt <= 0:
            return self._output

        error = setpoint - measurement

        # Proportional
        p_term = self.kp * error

        # Integral (with clamping to prevent windup)
        self._integral += error * dt
        # Anti-windup: clamp integral contribution
        i_term = self.ki * self._integral
        if i_term > self.out_max:
            self._integral = self.out_max / self.ki if self.ki else 0
            i_term = self.out_max
        elif i_term < self.out_min:
            self._integral = self.out_min / self.ki if self.ki else 0
            i_term = self.out_min

        # Derivative (on error)
        d_term = self.kd * (error - self._prev_error) / dt
        self._prev_error = error

        # Sum and clamp
        output = p_term + i_term + d_term
        output = max(self.out_min, min(self.out_max, output))
        self._output = output
        return output

    def reset(self):
        """Zero out all internal state."""
        self._integral   = 0.0
        self._prev_error = 0.0
        self._output     = 0.0
