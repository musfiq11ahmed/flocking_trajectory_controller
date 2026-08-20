import cv2
import numpy as np

# ==========================================
# 1. SETUP ARUCO DETECTOR
# ==========================================
aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
parameters = cv2.aruco.DetectorParameters()

# ADD THIS LINE: Force fractional pixel accuracy for the corners
parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX

detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)

# 1. Add 'cv2.CAP_DSHOW' to bypass Windows restrictions and unlock the full sensor
cam = cv2.VideoCapture(1, cv2.CAP_DSHOW) 

# 2. Force MJPG compression to bypass USB bandwidth limits
cam.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))

# 3. Request the full 2K widescreen resolution
cam.set(cv2.CAP_PROP_FRAME_WIDTH, 2560)
cam.set(cv2.CAP_PROP_FRAME_HEIGHT, 1440)
self.cam.set(cv2.CAP_PROP_AUTOFOCUS, 0) # Turn off Auto-Focus
self.cam.set(cv2.CAP_PROP_FOCUS, 0)     # Lock focus to infinity/floor

if not cam.isOpened():
    raise RuntimeError("Could not open Rapoo camera.")

# 4. Print the actual resolution to prove it worked
actual_w = cam.get(cv2.CAP_PROP_FRAME_WIDTH)
actual_h = cam.get(cv2.CAP_PROP_FRAME_HEIGHT)
print(f"[INFO] Camera started at resolution: {actual_w} x {actual_h}")

# Real-world coordinates (update these with your actual tape measure values!)
# Origin (0,0) is the exact physical center of the 2.7m x 1.67m arena
WORLD_ANCHORS = np.array([
    [-1.35, -0.835],  # ID 100: Bottom-Left
    [ 1.35, -0.835],  # ID 101: Bottom-Right 
    [-1.35,  0.835],  # ID 102: Top-Left
    [ 1.35,  0.835]   # ID 103: Top-Right
], dtype=np.float32)

def calculate_angle_radians(corners):
    """Calculates heading in radians, normalized to [-pi, pi]."""
    top_left, top_right, bottom_right, bottom_left = corners
    vector = top_right - top_left
    angle = np.arctan2(vector[1], vector[0])
    return (angle + np.pi) % (2 * np.pi) - np.pi

def draw_text_with_outline(img, text, position, font_scale, color, thickness):
    """Draws text with a thick black outline for maximum contrast."""
    # Draw the black background outline first (thicker)
    cv2.putText(img, text, position, cv2.FONT_HERSHEY_SIMPLEX, font_scale, (0, 0, 0), thickness + 2)
    # Draw the colored foreground text on top
    cv2.putText(img, text, position, cv2.FONT_HERSHEY_SIMPLEX, font_scale, color, thickness)

def draw_virtual_grid(frame, homography_matrix, arena_width=2.7, arena_height=1.67, grid_step=0.5):
    """Overlays a virtual coordinate grid and boundary box on the raw camera feed."""
    H_inv = np.linalg.inv(homography_matrix)
    
    x_min, x_max = -arena_width / 2.0, arena_width / 2.0
    y_min, y_max = -arena_height / 2.0, arena_height / 2.0
    
    def project_point(x, y):
        pt_meters = np.array([[[x, y]]], dtype=np.float32)
        pt_pixels = cv2.perspectiveTransform(pt_meters, H_inv)
        return (int(pt_pixels[0][0][0]), int(pt_pixels[0][0][1]))

    # Draw Grid Lines
    for x in np.arange(x_min, x_max + 0.01, grid_step):
        pt1, pt2 = project_point(x, y_min), project_point(x, y_max)
        color, thickness = ((0, 200, 0), 2) if abs(x) < 0.05 else ((120, 120, 120), 1)
        cv2.line(frame, pt1, pt2, color, thickness)
        
    for y in np.arange(y_min, y_max + 0.01, grid_step):
        pt1, pt2 = project_point(x_min, y), project_point(x_max, y)
        color, thickness = ((0, 200, 0), 2) if abs(y) < 0.05 else ((120, 120, 120), 1)
        cv2.line(frame, pt1, pt2, color, thickness)

    # Draw the Arena Boundary Box (Thick Red Line)
    pt_tl = project_point(x_min, y_min) # Top Left
    pt_tr = project_point(x_max, y_min) # Top Right
    pt_br = project_point(x_max, y_max) # Bottom Right
    pt_bl = project_point(x_min, y_max) # Bottom Left
    
    arena_corners = np.array([pt_tl, pt_tr, pt_br, pt_bl], dtype=np.int32)
    cv2.polylines(frame, [arena_corners], isClosed=True, color=(0, 0, 255), thickness=3)
        
    # Label the Origin with high contrast
    origin_px = project_point(0.0, 0.0)
    draw_text_with_outline(frame, "(0,0)", (origin_px[0] + 10, origin_px[1] - 10), 0.6, (0, 255, 255), 2)
                
    return frame

print("[INFO] Position all 4 anchor tags (100, 101, 102, 103) in view.")
print("[INFO] Press 'c' to lock calibration matrix, or 'q' to quit.")

homography_matrix = None
is_calibrated = False

# ==========================================
# EMA FILTER SETUP
# ==========================================
# Dictionary to store the smoothed states of each robot
bot_states = {}

# Smoothing factors (0.0 to 1.0)
# Lower = smoother but more lag. Higher = faster response but more jitter.
ALPHA_POS = 0.4   # For X, Y
ALPHA_THETA = 0.1 # Heading usually needs heavier smoothing

# ==========================================
# 2. MAIN VISION PIPELINE
# ==========================================
while True:
    success, img = cam.read()
    if not success:
        print("Failed to grab frame")
        break

    corners, ids, rejectedImgPoints = detector.detectMarkers(img)
    
    # Reset detected anchors every single frame
    detected_anchors = {}

    if ids is not None:
        ids = ids.flatten()
        cv2.aruco.drawDetectedMarkers(img, corners)

        for i in range(len(ids)):
            # 1. Extract the ID and corner data for the current tag
            marker_id = int(ids[i])
            marker_corners = corners[i][0]
            pixel_center = marker_corners.mean(axis=0)

            # 2. If it's an anchor tag, store its pixel location
            if marker_id in [100, 101, 102, 103]:
                detected_anchors[marker_id] = pixel_center

            # 3. If it is ANY other tag (0, 1, 2, etc.), treat it as a robot!
            elif is_calibrated:
                pt = np.array([[[pixel_center[0], pixel_center[1]]]], dtype=np.float32)
                transformed_pt = cv2.perspectiveTransform(pt, homography_matrix)[0][0]
                
                raw_x, raw_y = transformed_pt[0], transformed_pt[1]
                raw_theta = calculate_angle_radians(marker_corners)

                # --- APPLY EXPONENTIAL MOVING AVERAGE (EMA) ---
                if marker_id not in bot_states:
                    # First time seeing this bot, initialize with raw values
                    bot_states[marker_id] = {'x': raw_x, 'y': raw_y, 'theta': raw_theta}
                else:
                    prev_state = bot_states[marker_id]
                    
                    # Smooth X and Y
                    smooth_x = ALPHA_POS * raw_x + (1 - ALPHA_POS) * prev_state['x']
                    smooth_y = ALPHA_POS * raw_y + (1 - ALPHA_POS) * prev_state['y']
                    
                    # Smooth Theta (Shortest Path Calculation)
                    diff = raw_theta - prev_state['theta']
                    # Normalize difference to [-pi, pi]
                    diff = (diff + np.pi) % (2 * np.pi) - np.pi 
                    
                    # THE DEADBAND: If the rotational change is less than 0.02 rad (~1.1 degrees), 
                    # assume it's just camera noise and hold the angle completely static.
                    if abs(diff) < 0.035:
                        smooth_theta = prev_state['theta']
                    else:
                        smooth_theta = prev_state['theta'] + ALPHA_THETA * diff
                        # Re-normalize the final smoothed angle to [-pi, pi]
                        smooth_theta = (smooth_theta + np.pi) % (2 * np.pi) - np.pi

                    # Update the state dictionary
                    bot_states[marker_id] = {'x': smooth_x, 'y': smooth_y, 'theta': smooth_theta}

                # Extract the final smoothed values for display
                bot_x = bot_states[marker_id]['x']
                bot_y = bot_states[marker_id]['y']
                bot_theta = bot_states[marker_id]['theta']

                # Visualize real-world metrics on screen using dynamic marker_id
                cx, cy = int(pixel_center[0]), int(pixel_center[1])
                
                coord_text = f"Bot {marker_id}: X:{bot_x:.3f}m Y:{bot_y:.3f}m"
                draw_text_with_outline(img, coord_text, (cx - 50, cy - 35), 0.5, (0, 255, 0), 2)
                
                theta_text = f"Theta: {bot_theta:.3f} rad"
                draw_text_with_outline(img, theta_text, (cx - 40, cy + 45), 0.5, (0, 255, 255), 2)
                
                print(f"[TRACKING] Bot {marker_id} -> X: {bot_x:.3f}m, Y: {bot_y:.3f}m, Theta: {bot_theta:.3f} rad")

    # ==========================================
    # 3. KEYBOARD CONTROLS (Properly Scoped)
    # ==========================================
    key = cv2.waitKey(1) & 0xFF
    
    if key == ord('c'):
        # Ensure all 4 anchors are currently detected before doing the math
        if all(k in detected_anchors for k in [100, 101, 102, 103]):
            pixel_anchors = np.array([
                detected_anchors[100],
                detected_anchors[101],
                detected_anchors[102],
                detected_anchors[103]
            ], dtype=np.float32)
            
            # Compute the 3x3 Homography Matrix
            homography_matrix, _ = cv2.findHomography(pixel_anchors, WORLD_ANCHORS)
            is_calibrated = True
            print("[SUCCESS] Arena calibration locked successfully!")
        else:
            print("[ERROR] Cannot calibrate. Make sure tags 100, 101, 102, and 103 are fully visible without glare.")
            
    elif key == ord('q'):
        break

    # ==========================================
    # 4. STATUS OVERLAY & VIRTUAL GRID
    # ==========================================
    # Draw the AR grid if calibration is locked
    if is_calibrated:
        img = draw_virtual_grid(img, homography_matrix, arena_width=2.7, arena_height=1.67, grid_step=0.5)

    status_text = "Calibrated" if is_calibrated else "Uncalibrated - Press 'c' to Lock"
    color = (0, 255, 0) if is_calibrated else (0, 0, 255)
    cv2.putText(img, status_text, (30, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, color, 2)

    # --- THE DISPLAY FIX ---
    # Shrink the image down to 720p purely so it fits on your monitor.
    # This does NOT affect the high-res math and homography running above it!
    display_img = cv2.resize(img, (1280, 720))
    
    cv2.imshow("Arena Calibration & Vision Pipeline", display_img)

cam.release()
cv2.destroyAllWindows()