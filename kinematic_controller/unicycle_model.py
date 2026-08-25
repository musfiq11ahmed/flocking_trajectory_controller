import math

def calculate_wheel_velocities(curr_x, curr_y, curr_theta, targ_x, targ_y):
    """Translates spatial tracking error into differential drive m/s with proportional scaling."""
    
    # 1. Physical Parameters
    TRACK_WIDTH = 0.074  # Test Bot 1 track width
    MAX_SPEED = 0.30     # Top speed cap
    MIN_SPEED = 0.15     # Minimum speed to break static friction
    TOLERANCE = 0.10     # Arrival tolerance (5cm)
    
    # 2. Control Gains 
    K_v = 0.5  
    K_w = 0.3  

    # 3. Calculate Spatial Error
    dist_error = math.sqrt((targ_x - curr_x)**2 + (targ_y - curr_y)**2)
    
    if dist_error < TOLERANCE:
        return 0.0, 0.0, True  # Arrived
        
    # 4. Calculate Heading Error
    target_angle = math.atan2(targ_y - curr_y, targ_x - curr_x)
    heading_error = target_angle - curr_theta
    heading_error = math.atan2(math.sin(heading_error), math.cos(heading_error))

    if abs(heading_error) > math.pi / 2:
        forward_scale = 0.2  # Mostly turn, little forward
    else:
        forward_scale = max(0.2, math.cos(heading_error))

    v = K_v * dist_error * forward_scale
    
    # 5. Unicycle Control Law
    v = K_v * dist_error * max(0.2, math.cos(heading_error)) 
    w = K_w * heading_error
    
    # 6. Differential Drive Matrix
    v_left = v - (w * TRACK_WIDTH / 2.0)
    v_right = v + (w * TRACK_WIDTH / 2.0)
    
    
    # 7. PROPORTIONAL DEADBAND SCALING
   
    # Find which wheel is being commanded to spin the fastest
    max_wheel_speed = max(abs(v_left), abs(v_right))
    
    # If the robot is trying to move, but the fastest wheel is too weak to break friction
    if 0.01 < max_wheel_speed < MIN_SPEED:
        # Calculate the exact multiplier needed to boost the fastest wheel to MIN_SPEED
        boost_factor = MIN_SPEED / max_wheel_speed
        
        # Apply that exact same multiplier to both wheels to preserve the turn radius
        v_left *= boost_factor
        v_right *= boost_factor
        
    # 8. Absolute Safety Bounds (Allowing negative speeds for tight turning)
    v_left = max(-MAX_SPEED, min(MAX_SPEED, v_left))
    v_right = max(-MAX_SPEED, min(MAX_SPEED, v_right))
    
    return v_left, v_right, False