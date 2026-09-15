# This version corrects heading error by turning in place towards the target.
#
# FIXED 2026-08-30 — root cause of the "infinite spinning" bug:
# The original Phase 1 was a bang-bang controller: it always pivoted at the FULL
# fixed rate (2 * 0.13 / 0.074 = 3.51 rad/s = 201 deg/s) until the *measured*
# heading entered a +/-0.25 rad (14 deg) window. That window is traversed in only
# 71 ms, but the real feedback loop carries EMA filter lag + camera latency + UDP
# + the 50 ms firmware PID (~150-250 ms total). Every approach therefore blew
# through the window, exited on the opposite side, commanded a full-speed
# reversal, and overshot again — a stable limit cycle that looks like the robot
# "spinning forever" and never entering Phase 2.
#
# The fix makes the turn PROPORTIONAL (decelerating as heading error shrinks),
# adds enter/exit HYSTERESIS so the two phases cannot chatter, clamps the turn
# rate with a small stiction floor, guards against NaN/Inf vision input, and adds
# a bearing-rate feedforward in the drive phase so the robot stops re-entering
# the turn phase near the target.

import math

def calculate_wheel_velocities(curr_x, curr_y, curr_theta, targ_x, targ_y):

    TRACK_WIDTH = 0.074
    MAX_SPEED = 0.20
    MOVE_SPEED = 0.15
    TOLERANCE = 0.10

    # Turn phase (proportional instead of bang-bang)
    K_TURN = 2.0            # rad/s of turn rate per rad of heading error
    W_TURN_MAX = 2.0        # rad/s cap -> wheel speeds +/-0.074 m/s at most
    W_TURN_MIN = 1.2        # rad/s floor -> +/-0.044 m/s, overcomes stiction
    ENTER_TURN = 0.35       # ~20 deg: enter turn phase when error exceeds this
    EXIT_TURN = 0.18        # ~10 deg: leave turn phase once inside this (hysteresis)

    # Drive phase
    K_w = 0.8               # proportional heading correction while driving

    # --- Safety guard: a NaN/Inf pose (lost tag, bad homography) must stop the
    # robot. Previously min(MAX_SPEED, nan) returned MAX_SPEED -> full speed. ---
    if not all(math.isfinite(v) for v in (curr_x, curr_y, curr_theta, targ_x, targ_y)):
        calculate_wheel_velocities._turning = False
        return 0.0, 0.0, False

    # Distance check
    dist_error = math.sqrt((targ_x - curr_x)**2 + (targ_y - curr_y)**2)
    if dist_error < TOLERANCE:
        calculate_wheel_velocities._turning = False
        return 0.0, 0.0, True

    # Heading error (wrapped to [-pi, pi])
    target_angle = math.atan2(targ_y - curr_y, targ_x - curr_x)
    heading_error = target_angle - curr_theta
    heading_error = math.atan2(math.sin(heading_error), math.cos(heading_error))

    # --- Phase selection with hysteresis (state kept on the function object;
    # fine for the single-robot dispatcher — use a class/instance attribute if
    # this is ever reused per-robot in the swarm). ---
    turning = getattr(calculate_wheel_velocities, "_turning", False)
    if turning and abs(heading_error) < EXIT_TURN:
        turning = False
    elif not turning and abs(heading_error) > ENTER_TURN:
        turning = True
    calculate_wheel_velocities._turning = turning

    if turning:
        # PHASE 1: turn in place, rate proportional to remaining error.
        # As heading_error -> 0 the turn rate -> W_TURN_MIN and the phase exits
        # at EXIT_TURN, so feedback latency can no longer cause overshoot-reversal
        # oscillation. Sign convention preserved: heading_error > 0 (target left)
        # -> v_left negative, v_right positive (CCW).
        w = K_TURN * heading_error
        w = max(-W_TURN_MAX, min(W_TURN_MAX, w))
        if 0.0 < abs(w) < W_TURN_MIN:
            w = math.copysign(W_TURN_MIN, w)

        v_left = -w * TRACK_WIDTH / 2.0
        v_right = w * TRACK_WIDTH / 2.0
        return v_left, v_right, False

    else:
        # PHASE 2: move forward. The feedforward term cancels the bearing-rotation
        # term (v * sin(e) / d) that otherwise pushes the heading error back above
        # the gate within ~0.5 m of the target and caused repeated
        # align-drive-realign cycles.
        v = MOVE_SPEED
        w = K_w * heading_error \
            + MOVE_SPEED * math.sin(heading_error) / max(dist_error, TOLERANCE)

        v_left = v - (w * TRACK_WIDTH / 2.0)
        v_right = v + (w * TRACK_WIDTH / 2.0)

        v_left = max(-MAX_SPEED, min(MAX_SPEED, v_left))
        v_right = max(-MAX_SPEED, min(MAX_SPEED, v_right))

        return v_left, v_right, False
