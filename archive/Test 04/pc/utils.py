"""
utils.py — Maths helpers used across the PC code-base.
"""

import math
from config import TICKS_PER_METER, METERS_PER_TICK


def normalize_angle(angle: float) -> float:
    """Wrap *angle* (radians) into the range [−π, π]."""
    angle = angle % (2 * math.pi)
    if angle > math.pi:
        angle -= 2 * math.pi
    return angle


def clamp(value: float, lo: float, hi: float) -> float:
    """Clamp *value* to [lo, hi]."""
    return max(lo, min(hi, value))


def mps_to_tps(velocity_mps: float) -> float:
    """Convert metres-per-second → encoder ticks-per-second."""
    return velocity_mps * TICKS_PER_METER


def tps_to_mps(velocity_tps: float) -> float:
    """Convert encoder ticks-per-second → metres-per-second."""
    return velocity_tps * METERS_PER_TICK


def distance(x1: float, y1: float, x2: float, y2: float) -> float:
    """Euclidean distance between two 2-D points."""
    return math.hypot(x2 - x1, y2 - y1)
