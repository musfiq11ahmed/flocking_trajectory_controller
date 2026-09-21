"""Closed-loop bot control using ONLY camera-derived pose feedback.

``BotController`` implements the two SPEC behaviors:

* ``align_to_heading(target_theta=0.0)`` -- closed-loop pivot to +X
* ``move_forward_distance(distance_m=1.0)`` -- closed-loop straight move to
  ``start + (distance, 0)`` with heading/lateral correction

Both run at the 20 Hz command rate, smooth the camera pose with an EMA,
clamp speeds and then apply an anti-stall floor (SPEC wheel-speed shaping
semantics), and COAST (command 0,0 for >= 0.5 s) on success, abort, timeout,
or pose loss.  Ground-truth state is never read: the only feedback is the
``CameraPoseSource`` below, which chains vision -> homography -> EMA.
"""

import numpy as np
import pybullet as p

import config as C
from calibration import CalibrationError


class CameraPoseError(Exception):
    pass


def wrap_angle(a):
    """Wrap to (-pi, pi] -- atan2-based shortest-angle convention."""
    return float(np.arctan2(np.sin(a), np.cos(a)))


def shape_wheel_speeds(v, w):
    """Clamp first, then apply the anti-stall floor (SPEC semantics)."""
    v = float(np.clip(v, -C.MAX_LIN_SPEED, C.MAX_LIN_SPEED))
    w = float(np.clip(w, -C.MAX_ANG_SPEED, C.MAX_ANG_SPEED))
    if 0.0 < abs(v) < C.MIN_LIN_SPEED:
        v = np.sign(v) * C.MIN_LIN_SPEED
    if 0.0 < abs(w) < C.MIN_ANG_SPEED:
        w = np.sign(w) * C.MIN_ANG_SPEED
    return v, w


class CameraPoseSource:
    """The ONLY pose feedback the controller may use.

    update(): render one frame, segment the bot marker, map pixels through
    the locked homography and EMA-smooth the result.  Returns
    ((x, y), theta) in arena coordinates, or None if marker 4 is not visible.
    """

    def __init__(self, vision, calibrator, alpha=C.EMA_ALPHA):
        self.vision = vision
        self.calibrator = calibrator
        self.alpha = alpha
        self._pose = None          # (x, y, theta) smoothed
        self.updates = 0
        self.missed = 0

    def update(self):
        frame = self.vision.render()
        marker = self.vision.detect_bot_marker(frame)
        if marker is None:
            self.missed += 1
            return None
        try:
            (x, y), theta = self.calibrator.bot_pose_from_marker(marker)
        except CalibrationError:
            self.missed += 1
            return None
        self.updates += 1
        if self._pose is None:
            self._pose = (x, y, theta)
        else:
            px, py, _ = self._pose
            # EMA on position; circular EMA on heading.
            a = self.alpha
            sx = a * np.cos(theta) + (1 - a) * np.cos(self._pose[2])
            sy = a * np.sin(theta) + (1 - a) * np.sin(self._pose[2])
            self._pose = (a * x + (1 - a) * px,
                          a * y + (1 - a) * py,
                          float(np.arctan2(sy, sx)))
        return self.read()

    def read(self):
        if self._pose is None:
            return None
        return (self._pose[0], self._pose[1]), self._pose[2]

    def reset(self):
        self._pose = None
        self.updates = 0
        self.missed = 0


class BotController:
    """Kinematic velocity-command controller for the simulated bot."""

    def __init__(self, client, bot_id, pose_source,
                 cmd_period=C.CMD_PERIOD, physics_dt=C.PHYSICS_DT,
                 on_status=None):
        self.client = client
        self.bot_id = bot_id
        self.pose_source = pose_source
        self.cmd_period = cmd_period
        self.physics_dt = physics_dt
        self.steps_per_cmd = int(round(cmd_period / physics_dt))
        self.on_status = on_status or (lambda msg: None)
        self._cmd = (0.0, 0.0)          # (v, omega)
        self._theta_est = 0.0           # last camera-derived heading
        self.sim_time = 0.0

    # ------------------------------------------------------------------
    # Low-level actuation (velocity hold between command cycles)
    # ------------------------------------------------------------------
    def _apply_velocity(self):
        v, w = self._cmd
        # Command velocity along the CURRENT CAMERA-ESTIMATED heading; the
        # lateral component is zeroed by construction each physics step.
        vx = v * np.cos(self._theta_est)
        vy = v * np.sin(self._theta_est)
        p.resetBaseVelocity(self.bot_id,
                            linearVelocity=[vx, vy, 0.0],
                            angularVelocity=[0.0, 0.0, w],
                            physicsClientId=self.client)

    def _step_physics(self, n):
        for _ in range(n):
            self._apply_velocity()
            p.stepSimulation(physicsClientId=self.client)
            self.sim_time += self.physics_dt

    # ------------------------------------------------------------------
    # Coast (SPEC safety: explicit (0,0) for at least 0.5 s)
    # ------------------------------------------------------------------
    def coast(self, duration_s=0.5):
        self._cmd = (0.0, 0.0)
        self._step_physics(max(1, int(round(duration_s / self.physics_dt))))

    # ------------------------------------------------------------------
    # Feedback
    # ------------------------------------------------------------------
    def _sense(self):
        """One camera cycle; raises CameraPoseError on sustained loss."""
        pose = self.pose_source.update()
        if pose is None:
            raise CameraPoseError("marker 4 (bot) not visible")
        (x, y), theta = pose
        self._theta_est = theta
        return np.array([x, y]), theta

    def _aborted(self, abort_check):
        return abort_check is not None and abort_check()

    # ------------------------------------------------------------------
    # SPEC behavior 1: pivot to a target heading (default +X)
    # ------------------------------------------------------------------
    def align_to_heading(self, target_theta=C.TARGET_HEADING,
                         tol=C.ALIGN_TOL_RAD, timeout=C.ALIGN_TIMEOUT_S,
                         abort_check=None):
        """20 Hz closed-loop pivot. Returns True on success; always coasts."""
        k_h = 2.5
        t = 0.0
        ok_streak = 0
        try:
            while t < timeout:
                if self._aborted(abort_check):
                    self.on_status("align aborted by operator")
                    return False
                _, theta = self._sense()
                err = wrap_angle(target_theta - theta)
                if abs(err) <= tol:
                    ok_streak += 1
                    if ok_streak >= 3:
                        self.on_status(
                            "align done: theta=%.3f rad (err %.3f)" % (theta, err))
                        return True
                else:
                    ok_streak = 0
                _, w = shape_wheel_speeds(0.0, k_h * err)
                self._cmd = (0.0, w)
                self._step_physics(self.steps_per_cmd)
                t += self.cmd_period
            self.on_status("align TIMEOUT after %.1f s" % timeout)
            return False
        except CameraPoseError as exc:
            self.on_status("align failed: %s" % exc)
            return False
        finally:
            self.coast()

    # ------------------------------------------------------------------
    # SPEC behavior 2: drive straight 1 m along +X with camera feedback
    # ------------------------------------------------------------------
    def move_forward_distance(self, distance_m=C.MOVE_DISTANCE_M,
                              tol_along=C.MOVE_TOL_ALONG_M,
                              tol_lateral=C.MOVE_TOL_LATERAL_M,
                              timeout=C.MOVE_TIMEOUT_S,
                              abort_check=None):
        """Closed-loop move to start + (distance, 0); heading/lateral trim.

        Returns True on success; always coasts.
        """
        k_v, k_h = 1.6, 3.0
        t = 0.0
        ok_streak = 0
        try:
            start, _ = self._sense()
            target = start + np.array([distance_m, 0.0])
            self.on_status("move: start=(%.3f, %.3f) target=(%.3f, %.3f)"
                           % (start[0], start[1], target[0], target[1]))
            while t < timeout:
                if self._aborted(abort_check):
                    self.on_status("move aborted by operator")
                    return False
                pos, theta = self._sense()
                err = target - pos
                e_along, e_lat = float(err[0]), float(err[1])
                # Accept at 70% of the SPEC tolerance: the acceptance gate is
                # checked on the camera estimate, and this margin absorbs the
                # (small) camera-vs-ground-truth estimation bias.
                if (abs(e_along) <= 0.7 * tol_along
                        and abs(e_lat) <= 0.7 * tol_lateral):
                    ok_streak += 1
                    if ok_streak >= 3:
                        self.on_status(
                            "move done: pos=(%.3f, %.3f) along_err=%.3f lat_err=%.3f"
                            % (pos[0], pos[1], e_along, e_lat))
                        return True
                    v, w = 0.0, 0.0
                else:
                    ok_streak = 0
                    # Desired heading steers back onto the +X track.
                    theta_des = float(np.arctan2(
                        e_lat, max(e_along, 0.15)))
                    theta_des = float(np.clip(theta_des, -0.6, 0.6))
                    theta_err = wrap_angle(theta_des - theta)
                    v = k_v * e_along
                    if abs(theta_err) > 0.5:
                        v *= 0.2          # fix heading before translating
                    v, w = shape_wheel_speeds(v, k_h * theta_err)
                    if ok_streak > 0:
                        v, w = 0.0, 0.0
                self._cmd = (v, w)
                self._step_physics(self.steps_per_cmd)
                t += self.cmd_period
            self.on_status("move TIMEOUT after %.1f s" % timeout)
            return False
        except CameraPoseError as exc:
            self.on_status("move failed: %s" % exc)
            return False
        finally:
            self.coast()
