# This version corrects heading error by calculating an arc towards the target.

import math

def calculate_wheel_velocities(curr_x, curr_y, curr_theta, targ_x, targ_y):
    
    TRACK_WIDTH = 0.074
    MAX_SPEED = 0.20
    MIN_SPEED = 0.25
    TOLERANCE = 0.10
    K_w = 0.8

    # Distance check
    dist_error = math.sqrt((targ_x - curr_x)**2 + (targ_y - curr_y)**2)
    if dist_error < TOLERANCE:
        return 0.0, 0.0, True

    # Heading error
    target_angle = math.atan2(targ_y - curr_y, targ_x - curr_x)
    heading_error = target_angle - curr_theta
    heading_error = math.atan2(math.sin(heading_error), math.cos(heading_error))

    # Always move forward at fixed speed, steer with w
    v = MIN_SPEED
    w = K_w * heading_error

    # Differential drive
    v_left = v - (w * TRACK_WIDTH / 2.0)
    v_right = v + (w * TRACK_WIDTH / 2.0)

    # Clamp to safe range
    v_left = max(-MAX_SPEED, min(MAX_SPEED, v_left))
    v_right = max(-MAX_SPEED, min(MAX_SPEED, v_right))

    # Ensure minimum speed on both wheels
    #if abs(v_left) < MIN_SPEED and abs(v_left) > 0.01:
        #v_left = math.copysign(MIN_SPEED, v_left)
    #if abs(v_right) < MIN_SPEED and abs(v_right) > 0.01:
        #v_right = math.copysign(MIN_SPEED, v_right)

    return v_left, v_right, False