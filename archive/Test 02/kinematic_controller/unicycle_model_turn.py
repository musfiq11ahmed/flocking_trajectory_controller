# This version corrects heading error by turning in place towards the target.

import math

def calculate_wheel_velocities(curr_x, curr_y, curr_theta, targ_x, targ_y):
    
    TRACK_WIDTH = 0.074
    MAX_SPEED = 0.20
    MOVE_SPEED = 0.15
    TURN_SPEED = 0.13      # Fixed slow turn speed — slow enough not to overshoot
    TOLERANCE = 0.10
    HEADING_TOLERANCE = 0.25  # ~14 degrees — stop turning when within this
    K_w = 0.3              # Only used during movement phase for gentle correction

    # Distance check
    dist_error = math.sqrt((targ_x - curr_x)**2 + (targ_y - curr_y)**2)
    if dist_error < TOLERANCE:
        return 0.0, 0.0, True

    # Heading error
    target_angle = math.atan2(targ_y - curr_y, targ_x - curr_x)
    heading_error = target_angle - curr_theta
    heading_error = math.atan2(math.sin(heading_error), math.cos(heading_error))

    if abs(heading_error) > HEADING_TOLERANCE:
        # PHASE 1: Turn in place toward target at fixed slow speed
        if heading_error > 0:
            # Turn left — right wheel faster
            v_left = -TURN_SPEED
            v_right = TURN_SPEED
        else:
            # Turn right — left wheel faster
            v_left = TURN_SPEED
            v_right = -TURN_SPEED
        return v_left, v_right, False

    else:
        # PHASE 2: Move forward with gentle heading correction
        v = MOVE_SPEED
        w = K_w * heading_error

        v_left = v - (w * TRACK_WIDTH / 2.0)
        v_right = v + (w * TRACK_WIDTH / 2.0)

        v_left = max(-MAX_SPEED, min(MAX_SPEED, v_left))
        v_right = max(-MAX_SPEED, min(MAX_SPEED, v_right))

        return v_left, v_right, False