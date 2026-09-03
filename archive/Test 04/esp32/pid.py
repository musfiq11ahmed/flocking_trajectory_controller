"""
pid.py — Discrete PID controller for MicroPython.

Each wheel gets its own PID instance.  The controller operates on
encoder ticks-per-second as both setpoint and measurement.
"""


class PID:
    """Incremental PID with feedforward and anti-windup clamping.

    Parameters
    ----------
    kp, ki, kd, kf : float
        Proportional, integral, derivative, and feedforward gains.
    out_min, out_max : float
        Output clamp range (maps to PWM duty: −1023 … +1023).
    """

    def __init__(self, kp=0.6, ki=0.1, kd=0.0, kf=0.5,
                 out_min=-1023, out_max=1023):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.kf = kf
        self.out_min = out_min
        self.out_max = out_max

        self._integral   = 0.0
        self._prev_error = 0.0
        self._output     = 0.0

    def update(self, setpoint, measurement, dt):
        """Compute one PID step with feedforward.

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

        # 1. Feed-forward (provides immediate baseline PWM for desired speed)
        ff_term = self.kf * setpoint

        # 2. Minimum stiction boost (overcomes static gearbox friction)
        stiction_term = 0.0
        if abs(setpoint) > 5.0:
            stiction_term = 70.0 if setpoint > 0 else -70.0

        # 3. Proportional
        p_term = self.kp * error

        # 4. Integral (with anti-windup clamping to prevent runaway)
        self._integral += error * dt
        i_max = 350.0   # Limit integral contribution
        i_term = self.ki * self._integral
        if i_term > i_max:
            self._integral = i_max / self.ki if self.ki else 0
            i_term = i_max
        elif i_term < -i_max:
            self._integral = -i_max / self.ki if self.ki else 0
            i_term = -i_max

        # 5. Derivative (damped, zero by default to prevent discrete tick jitter)
        d_term = 0.0
        if self.kd > 0 and dt > 0:
            d_term = self.kd * (error - self._prev_error) / dt
        self._prev_error = error

        # Sum and clamp to [-1023, 1023]
        output = ff_term + stiction_term + p_term + i_term + d_term
        output = max(self.out_min, min(self.out_max, output))
        self._output = output
        return output

    def reset(self):
        """Zero out all internal state."""
        self._integral   = 0.0
        self._prev_error = 0.0
        self._output     = 0.0
