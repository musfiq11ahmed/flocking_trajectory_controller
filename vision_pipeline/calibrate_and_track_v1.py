import cv2
import numpy as np


# 1. SETUP ARUCO DETECTOR

aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
parameters = cv2.aruco.DetectorParameters()
detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)

# 1. Add 'cv2.CAP_DSHOW' to bypass Windows restrictions and unlock the full sensor
cam = cv2.VideoCapture(1, cv2.CAP_DSHOW) 

# 2. Force MJPG compression to bypass USB bandwidth limits
cam.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))

# 3. Request the full 2K widescreen resolution
cam.set(cv2.CAP_PROP_FRAME_WIDTH, 2560)
cam.set(cv2.CAP_PROP_FRAME_HEIGHT, 1440)

if not cam.isOpened():
    raise RuntimeError("Could not open Rapoo camera.")

# 4. Print the actual resolution to prove it worked
actual_w = cam.get(cv2.CAP_PROP_FRAME_WIDTH)
actual_h = cam.get(cv2.CAP_PROP_FRAME_HEIGHT)
print(f"[INFO] Camera started at resolution: {actual_w} x {actual_h}")

# Real-world coordinates
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

print("[INFO] Position all 4 anchor tags (100, 101, 102, 103) in view.")
print("[INFO] Press 'c' to lock calibration matrix, or 'q' to quit.")

homography_matrix = None
is_calibrated = False


# 2. MAIN VISION PIPELINE

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
                # Reshape pixel center for perspective transform function
                pt = np.array([[[pixel_center[0], pixel_center[1]]]], dtype=np.float32)
                transformed_pt = cv2.perspectiveTransform(pt, homography_matrix)[0][0]
                
                bot_x, bot_y = transformed_pt[0], transformed_pt[1]
                bot_theta = calculate_angle_radians(marker_corners)

                # Visualize real-world metrics on screen using dynamic marker_id
                cx, cy = int(pixel_center[0]), int(pixel_center[1])
                cv2.putText(img, f"Bot {marker_id}: X:{bot_x:.2f}m Y:{bot_y:.2f}m", (cx, cy - 20), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                cv2.putText(img, f"Theta: {bot_theta:.2f} rad", (cx, cy + 30), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                
                print(f"[TRACKING] Bot {marker_id} -> X: {bot_x:.3f}m, Y: {bot_y:.3f}m, Theta: {bot_theta:.2f} rad")

    
    # 3. KEYBOARD CONTROLS (Properly Scoped)
    
    key = cv2.waitKey(1) & 0xFF
    
    if key == ord('c'):
        # Ensure all 4 anchors are currently detected
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


    # 4. STATUS OVERLAY
    
    status_text = "Calibrated" if is_calibrated else "Uncalibrated - Press 'c' to Lock"
    color = (0, 255, 0) if is_calibrated else (0, 0, 255)
    cv2.putText(img, status_text, (30, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, color, 2)

    # THE DISPLAY FIX
    
    display_img = cv2.resize(img, (1280, 720))
    
    cv2.imshow("Arena Calibration & Vision Pipeline", display_img)

cam.release()
cv2.destroyAllWindows()