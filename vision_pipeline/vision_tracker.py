import cv2
import time
import math
import numpy as np
from pupil_apriltags import Detector

# ==========================================
# 1. CAMERA INITIALIZATION
# ==========================================
# 0 is usually the default built-in webcam. Change to 1 or 2 if the Rapoo C280 is an external USB camera.
cap = cv2.VideoCapture(1) 

# Set resolution (We will use 1080p to maintain a high framerate on the Rapoo)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)

# LOCK AUTO-FOCUS AND AUTO-EXPOSURE (CRITICAL FOR ROBOTICS)
cap.set(cv2.CAP_PROP_AUTOFOCUS, 0) 

# Note: OpenCV exposure flags vary by Operating System.
# Windows (DirectShow): 0.25 is manual, 0.75 is auto.
# Linux (V4L2): 1 is manual, 3 is auto.
cap.set(cv2.CAP_PROP_AUTO_EXPOSURE, 0.25) 
# cap.set(cv2.CAP_PROP_EXPOSURE, -5) # Uncomment and tweak this to darken the room and kill glare

# ==========================================
# 2. APRILTAG DETECTOR & STATE VARIABLES
# ==========================================
# We use tag36h11 as it is the industry standard for robotics
detector = Detector(families='tag36h11', nthreads=1)

# Dead Reckoning State Memory
last_pose = None  # (x, y, theta)
last_time = time.time()
vx, vy = 0.0, 0.0  # Velocity in pixels per second
alpha = 0.5  # Smoothing factor for our vision-only velocity filter

print("[INFO] Vision Pipeline Initialized. Press 'q' to quit.")

while True:
    ret, frame = cap.read()
    if not ret:
        print("[ERROR] Could not read from webcam.")
        break
        
    current_time = time.time()
    dt = current_time - last_time
    
    # Convert to grayscale for the AprilTag library
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    results = detector.detect(gray)
    
    if len(results) > 0:
        # ==========================================
        # 3. TAG DETECTED (TRUE TRACKING)
        # ==========================================
        tag = results[0] 
        
        # Extract 2D Center Position
        x, y = tag.center
        
        # Calculate Theta (Heading) from the tag's corners
        # Corners: 0=Bottom-Left, 1=Bottom-Right, 2=Top-Right, 3=Top-Left
        ptA, ptB, ptC, ptD = tag.corners
        
        # Find the vector from the bottom edge to the top edge to determine where the tag is "looking"
        bottom_mid_x = (ptA[0] + ptB[0]) / 2
        bottom_mid_y = (ptA[1] + ptB[1]) / 2
        top_mid_x = (ptD[0] + ptC[0]) / 2
        top_mid_y = (ptD[1] + ptC[1]) / 2
        
        dx = top_mid_x - bottom_mid_x
        dy = top_mid_y - bottom_mid_y
        theta = math.atan2(dy, dx)
        
        # Update Velocity Filter for Dead Reckoning
        if last_pose is not None and dt > 0:
            raw_vx = (x - last_pose[0]) / dt
            raw_vy = (y - last_pose[1]) / dt
            
            # Exponential moving average to smooth out camera jitter
            vx = (alpha * raw_vx) + ((1 - alpha) * vx)
            vy = (alpha * raw_vy) + ((1 - alpha) * vy)
            
        # Save state
        last_pose = (x, y, theta)
        last_time = current_time
        status = "TRACKING (EKF/VISION)"
        color = (0, 255, 0) # Green
        
    else:
        # ==========================================
        # 4. TAG LOST (DEAD RECKONING FAILSAFE)
        # ==========================================
        if last_pose is not None and dt > 0:
            # Predict new position using last known smoothed velocity
            x = last_pose[0] + (vx * dt)
            y = last_pose[1] + (vy * dt)
            theta = last_pose[2] # Assume heading hasn't drifted wildly in a split second
            
            # Save the guessed state so it continues to drift along the vector
            last_pose = (x, y, theta)
            last_time = current_time
            
            status = "OCCLUDED: DEAD RECKONING"
            color = (0, 165, 255) # Orange
        else:
            x, y, theta = 0, 0, 0
            status = "SEARCHING..."
            color = (0, 0, 255) # Red

    # ==========================================
    # 5. VISUALIZATION
    # ==========================================
    if status != "SEARCHING...":
        # Draw Robot Center
        cv2.circle(frame, (int(x), int(y)), 8, color, -1)
        
        # Draw Heading Line (Pointer)
        end_x = int(x + 60 * math.cos(theta))
        end_y = int(y + 60 * math.sin(theta))
        cv2.line(frame, (int(x), int(y)), (end_x, end_y), (255, 0, 0), 3)
        
        # Print Data to Screen
        cv2.putText(frame, f"X: {int(x)} Y: {int(y)}", (20, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        cv2.putText(frame, f"Heading: {math.degrees(theta):.1f} deg", (20, 110), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)

    # Print Status
    cv2.putText(frame, status, (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, color, 3)
    
    cv2.imshow("Swarm Vision Observer", frame)
    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()