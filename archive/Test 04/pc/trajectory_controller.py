"""
trajectory_controller.py — Pure Pursuit + heading controller for a
differential-drive robot.

The controller runs on the PC at ~30 Hz.  For every control cycle it:
  1. Finds the closest trajectory point and advances the waypoint index.
  2. Picks a *lookahead* point L_d ahead on the path.
  3. Computes curvature → linear + angular velocity (Pure Pursuit).
  4. Adds a heading-error correction term.
  5. Converts (v, ω) → (v_left, v_right) via differential-drive
     inverse kinematics.
  6. Converts m/s → ticks/s and clamps to motor limits.
"""

import csv
import math
from typing import List, Tuple, Optional

from config import (
    LOOKAHEAD_DISTANCE, CRUISE_SPEED, MIN_SPEED,
    GOAL_TOLERANCE, FINAL_HEADING_TOL, K_HEADING,
    SLOWDOWN_DISTANCE, WHEEL_BASE_M,
    MAX_TICKS_PER_SEC,
)
from utils import normalize_angle, clamp, mps_to_tps, distance


Waypoint = Tuple[float, float, float]          # (x, y, θ) in metres / rad


class TrajectoryController:
    """Pure Pursuit + heading-error controller."""

    def __init__(self):
        self.path: List[Waypoint] = []
        self.current_idx: int = 0
        self._finished = False

    # ── loading ───────────────────────────────────────────────

    def load_csv(self, filepath: str):
        """Load a trajectory CSV with columns *x*, *y*, *theta* (radians)."""
        self.path.clear()
        self.current_idx = 0
        self._finished = False

        with open(filepath, newline="") as f:
            reader = csv.reader(f)
            header = next(reader)                  # skip header row

            # Detect delimiter (whitespace or comma)
            if len(header) == 1:
                # likely whitespace-separated; re-read
                f.seek(0)
                reader = csv.reader(f, delimiter=" ", skipinitialspace=True)
                next(reader)  # skip header

            for row in reader:
                # Filter out empty strings from whitespace splitting
                vals = [v for v in row if v.strip()]
                if len(vals) < 3:
                    continue
                x, y, theta = float(vals[0]), float(vals[1]), float(vals[2])
                self.path.append((x, y, theta))

        if not self.path:
            raise ValueError(f"No waypoints loaded from {filepath}")
        print(f"[TRAJ] Loaded {len(self.path)} waypoints from {filepath}")

    # ── control ───────────────────────────────────────────────

    def compute(self, robot_x: float, robot_y: float, robot_theta: float
                ) -> Tuple[float, float, Optional[int], Optional[Waypoint]]:
        """Run one control step.

        Returns
        -------
        v_left_tps : float   — left  wheel target speed (ticks/s)
        v_right_tps : float  — right wheel target speed (ticks/s)
        target_idx : int | None
        lookahead_wp : Waypoint | None
        """
        if self._finished or not self.path:
            return 0.0, 0.0, None, None

        # 1. Find nearest point on path (search a window ahead)
        self._advance_nearest(robot_x, robot_y)

        # 2. Check if we've reached the final waypoint
        last = self.path[-1]
        dist_to_goal = distance(robot_x, robot_y, last[0], last[1])
        if self.current_idx >= len(self.path) - 1 and dist_to_goal < GOAL_TOLERANCE:
            # Final heading alignment
            heading_err = abs(normalize_angle(last[2] - robot_theta))
            if heading_err < FINAL_HEADING_TOL:
                self._finished = True
                print("[TRAJ] Trajectory FINISHED.")
                return 0.0, 0.0, self.current_idx, None
            else:
                # Rotate in-place to align heading
                omega = K_HEADING * normalize_angle(last[2] - robot_theta)
                omega = clamp(omega, -2.0, 2.0)
                v_l, v_r = self._diff_drive(0.0, omega)
                return mps_to_tps(v_l), mps_to_tps(v_r), self.current_idx, None

        # 3. Find lookahead point
        la_idx = self._find_lookahead(robot_x, robot_y)
        la_wp  = self.path[la_idx]

        # 4. Pure Pursuit curvature
        alpha = math.atan2(la_wp[1] - robot_y, la_wp[0] - robot_x) - robot_theta
        alpha = normalize_angle(alpha)

        ld_actual = distance(robot_x, robot_y, la_wp[0], la_wp[1])
        ld_actual = max(ld_actual, 0.01)          # avoid divide-by-zero

        curvature = 2.0 * math.sin(alpha) / ld_actual

        # 5. Speed profile — slow down near end
        v = CRUISE_SPEED
        if dist_to_goal < SLOWDOWN_DISTANCE:
            ratio = dist_to_goal / SLOWDOWN_DISTANCE
            v = MIN_SPEED + (CRUISE_SPEED - MIN_SPEED) * ratio

        omega = v * curvature

        # 6. Heading correction toward the *current* waypoint's theta
        target_theta = self.path[min(self.current_idx + 1, len(self.path) - 1)][2]
        heading_err  = normalize_angle(target_theta - robot_theta)
        omega += K_HEADING * heading_err

        # 7. Inverse kinematics → wheel velocities (m/s)
        v_l, v_r = self._diff_drive(v, omega)

        # 8. Convert to ticks/s and clamp
        v_l_tps = clamp(mps_to_tps(v_l), -MAX_TICKS_PER_SEC, MAX_TICKS_PER_SEC)
        v_r_tps = clamp(mps_to_tps(v_r), -MAX_TICKS_PER_SEC, MAX_TICKS_PER_SEC)

        return v_l_tps, v_r_tps, la_idx, la_wp

    @property
    def is_finished(self) -> bool:
        return self._finished

    def reset(self):
        self.current_idx = 0
        self._finished = False

    # ── internal ──────────────────────────────────────────────

    def _advance_nearest(self, rx, ry):
        """Move current_idx forward to the nearest path point (never backwards)."""
        best_d   = float("inf")
        best_idx = self.current_idx
        # Search up to 60 points ahead
        end = min(self.current_idx + 60, len(self.path))
        for i in range(self.current_idx, end):
            d = distance(rx, ry, self.path[i][0], self.path[i][1])
            if d < best_d:
                best_d   = d
                best_idx = i
        self.current_idx = best_idx

    def _find_lookahead(self, rx, ry) -> int:
        """Return the index of the first path point ≥ L_d from the robot,
        starting at current_idx."""
        for i in range(self.current_idx, len(self.path)):
            if distance(rx, ry, self.path[i][0], self.path[i][1]) >= LOOKAHEAD_DISTANCE:
                return i
        return len(self.path) - 1

    @staticmethod
    def _diff_drive(v: float, omega: float) -> Tuple[float, float]:
        """Differential-drive inverse kinematics.

        v     — linear  velocity (m/s)
        omega — angular velocity (rad/s, positive = CCW)

        Returns (v_left, v_right) in m/s.
        """
        v_left  = v - omega * WHEEL_BASE_M / 2.0
        v_right = v + omega * WHEEL_BASE_M / 2.0
        return v_left, v_right
