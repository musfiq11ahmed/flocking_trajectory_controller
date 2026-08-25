import math

def calculate_wheel_velocities(curr_x, curr_y, curr_theta, targ_x, targ_y):
    """
    Translates spatial tracking error between the camera frame and the CSV 
    trajectory into differential drive targets (meters per second).
    """
    # 1. Physical Parameters of Test Bot 1
    TRACK_WIDTH = 0.074  # Distance between left & right wheels in METERS
    MAX_SPEED = 0.75    # Capped to leave overhead for PID adjustments
    TOLERANCE = 0.05    # Consider target reached if within 5 cm
    
    # 2. Control Gains (Tweak these to change how aggressively it pursues a point)
    K_v = 1.5   # Linear velocity gain (Forward speed)
    K_w = 3.0   # Angular velocity gain (Turning speed)

    # 3. Calculate Distance (Spatial Error)
    dist_error = math.sqrt((targ_x - curr_x)**2 + (targ_y - curr_y)**2)
    
    if dist_error < TOLERANCE:
        return 0.0, 0.0, True  # Arrived at waypoint
        
    # 4. Calculate Heading Error
    target_angle = math.atan2(targ_y - curr_y, targ_x - curr_x)
    heading_error = target_angle - curr_theta
    
    # Normalize heading error to keep it strictly between [-pi, pi]
    heading_error = math.atan2(math.sin(heading_error), math.cos(heading_error))
    
    # 5. Unicycle Control Law
    # If heading error is massive, slow down forward motion and favor turning first
    v = K_v * dist_error * max(0, math.cos(heading_error)) 
    w = K_w * heading_error
    
    # 6. Kinematics Transformation Matrix (Unicycle to Differential Drive)
    v_left = v - (w * TRACK_WIDTH / 2.0)
    v_right = v + (w * TRACK_WIDTH / 2.0)
    
    # 7. Bound Outputs and Apply Deadband (The Humming Fix)
    MIN_SPEED = 0.35 # The minimum speed required to break static friction
    
    # If the math wants the wheel to move, ensure it gets at least MIN_SPEED
    if v_left > 0.05: 
        v_left = max(MIN_SPEED, v_left)
    if v_right > 0.05: 
        v_right = max(MIN_SPEED, v_right)
    
    v_left = max(0.0, min(MAX_SPEED, v_left))
    v_right = max(0.0, min(MAX_SPEED, v_right))
    
    return v_left, v_right, False