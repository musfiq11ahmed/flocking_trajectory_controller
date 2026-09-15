# Instrumented version of unicycle_model_turn_fixed.py — identical control logic,
# plus per-cycle CSV logging to localize a still-spinning robot.
# Log columns: t, curr_x, curr_y, curr_theta, heading_error, phase, v_left, v_right
#
# How to read the log:
#   robot rotates but curr_theta frozen        -> vision stale/dropout
#   heading_error one sign, theta moves wrong way -> sign inversion or swapped motors
#   heading_error alternates sign every few rows  -> latency oscillation
#   heading_error never < 0.35                   -> wrong units (degrees?)

import math
import time
import csv
import os

LOG_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)), "controller_log.csv")

def calculate_wheel_velocities(curr_x, curr_y, curr_theta, targ_x, targ_y):

    TRACK_WIDTH = 0.074
    MAX_SPEED = 0.20
    MOVE_SPEED = 0.15
    TOLERANCE = 0.10

    K_TURN = 2.0
    W_TURN_MAX = 2.0
    W_TURN_MIN = 1.2
    ENTER_TURN = 0.35
    EXIT_TURN = 0.18

    K_w = 0.8

    if not all(math.isfinite(v) for v in (curr_x, curr_y, curr_theta, targ_x, targ_y)):
        calculate_wheel_velocities._turning = False
        _log(curr_x, curr_y, curr_theta, float("nan"), "GUARD", 0.0, 0.0)
        return 0.0, 0.0, False

    dist_error = math.sqrt((targ_x - curr_x)**2 + (targ_y - curr_y)**2)
    if dist_error < TOLERANCE:
        calculate_wheel_velocities._turning = False
        _log(curr_x, curr_y, curr_theta, 0.0, "DONE", 0.0, 0.0)
        return 0.0, 0.0, True

    target_angle = math.atan2(targ_y - curr_y, targ_x - curr_x)
    heading_error = target_angle - curr_theta
    heading_error = math.atan2(math.sin(heading_error), math.cos(heading_error))

    turning = getattr(calculate_wheel_velocities, "_turning", False)
    if turning and abs(heading_error) < EXIT_TURN:
        turning = False
    elif not turning and abs(heading_error) > ENTER_TURN:
        turning = True
    calculate_wheel_velocities._turning = turning

    if turning:
        w = K_TURN * heading_error
        w = max(-W_TURN_MAX, min(W_TURN_MAX, w))
        if 0.0 < abs(w) < W_TURN_MIN:
            w = math.copysign(W_TURN_MIN, w)
        v_left = -w * TRACK_WIDTH / 2.0
        v_right = w * TRACK_WIDTH / 2.0
        _log(curr_x, curr_y, curr_theta, heading_error, "TURN", v_left, v_right)
        return v_left, v_right, False
    else:
        v = MOVE_SPEED
        w = K_w * heading_error \
            + MOVE_SPEED * math.sin(heading_error) / max(dist_error, TOLERANCE)
        v_left = v - (w * TRACK_WIDTH / 2.0)
        v_right = v + (w * TRACK_WIDTH / 2.0)
        v_left = max(-MAX_SPEED, min(MAX_SPEED, v_left))
        v_right = max(-MAX_SPEED, min(MAX_SPEED, v_right))
        _log(curr_x, curr_y, curr_theta, heading_error, "DRIVE", v_left, v_right)
        return v_left, v_right, False


def _log(cx, cy, ct, he, phase, vl, vr):
    new_file = not os.path.exists(LOG_PATH)
    with open(LOG_PATH, "a", newline="") as f:
        wtr = csv.writer(f)
        if new_file:
            wtr.writerow(["t", "curr_x", "curr_y", "curr_theta",
                          "heading_error", "phase", "v_left", "v_right"])
        wtr.writerow([round(time.time(), 3), cx, cy, ct,
                      round(he, 4) if isinstance(he, float) else he,
                      phase, round(vl, 4), round(vr, 4)])
