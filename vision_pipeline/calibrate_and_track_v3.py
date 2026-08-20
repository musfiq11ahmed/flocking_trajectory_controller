import cv2
import numpy as np

class SwarmVision:
    def __init__(self):
        # ==========================================
        # SETUP ARUCO DETECTOR
        # ==========================================
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_250)
        self.parameters = cv2.aruco.DetectorParameters()
        
        # 1. Make the shape approximation more forgiving (Helps fight motion blur smearing)
        self.parameters.polygonalApproxAccuracyRate = 0.05 
        
        # 2. Lower the minimum size threshold so it doesn't accidentally filter out the 70mm tag
        self.parameters.minMarkerPerimeterRate = 0.015
        
        # 3. Boost the threshold constant to force a higher contrast difference
        self.parameters.adaptiveThreshConstant = 10 
        
        self.parameters.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_SUBPIX
        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.parameters)

        # Initialize Rapoo camera at 2560x1440 with MJPG compression
        self.cam = cv2.VideoCapture(1, cv2.CAP_DSHOW)
        self.cam.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        self.cam.set(cv2.CAP_PROP_FRAME_WIDTH, 2560)
        self.cam.set(cv2.CAP_PROP_FRAME_HEIGHT, 1440)
        self.cam.set(cv2.CAP_PROP_AUTOFOCUS, 0) # Turn off Auto-Focus
        self.cam.set(cv2.CAP_PROP_FOCUS, 0)     # Lock focus to infinity/floor

        if not self.cam.isOpened():
            raise RuntimeError("Could not open Rapoo camera.")

        # World Anchors (Origin shifted to the center of the left wall)
        self.WORLD_ANCHORS = np.array([
            [ 0.0, -0.835],  # ID 100: Bottom-Left
            [ 2.7, -0.835],  # ID 101: Bottom-Right
            [ 0.0,  0.835],  # ID 102: Top-Left
            [ 2.7,  0.835]   # ID 103: Top-Right
        ], dtype=np.float32)

        self.homography_matrix = None
        self.is_calibrated = False
        
        # EMA Filter Setup
        self.bot_states = {}
        self.ALPHA_POS = 0.15
        self.ALPHA_THETA = 0.05

    def _calculate_angle_radians(self, corners):
        top_left, top_right, bottom_right, bottom_left = corners
        vector = top_right - top_left
        angle = np.arctan2(vector[1], vector[0])
        return (angle + np.pi) % (2 * np.pi) - np.pi

    def _draw_text_with_outline(self, img, text, position, font_scale, color, thickness):
        cv2.putText(img, text, position, cv2.FONT_HERSHEY_SIMPLEX, font_scale, (0, 0, 0), thickness + 2)
        cv2.putText(img, text, position, cv2.FONT_HERSHEY_SIMPLEX, font_scale, color, thickness)

    def _draw_virtual_grid(self, frame, H_inv, arena_width=2.7, arena_height=1.67, grid_step=0.5):
        # X now starts at 0 (left wall) and goes to arena_width (right wall)
        x_min, x_max = 0.0, arena_width
        y_min, y_max = -arena_height / 2.0, arena_height / 2.0
        
        def project_point(x, y):
            pt_meters = np.array([[[x, y]]], dtype=np.float32)
            pt_pixels = cv2.perspectiveTransform(pt_meters, H_inv)
            return (int(pt_pixels[0][0][0]), int(pt_pixels[0][0][1]))

        for x in np.arange(x_min, x_max + 0.01, grid_step):
            pt1, pt2 = project_point(x, y_min), project_point(x, y_max)
            color, thickness = ((0, 200, 0), 2) if abs(x) < 0.05 else ((120, 120, 120), 1)
            cv2.line(frame, pt1, pt2, color, thickness)
            
        for y in np.arange(y_min, y_max + 0.01, grid_step):
            pt1, pt2 = project_point(x_min, y), project_point(x_max, y)
            color, thickness = ((0, 200, 0), 2) if abs(y) < 0.05 else ((120, 120, 120), 1)
            cv2.line(frame, pt1, pt2, color, thickness)

        arena_corners = np.array([project_point(x_min, y_min), project_point(x_max, y_min), project_point(x_max, y_max), project_point(x_min, y_max)], dtype=np.int32)
        cv2.polylines(frame, [arena_corners], isClosed=True, color=(0, 0, 255), thickness=3)
        return frame

    def update(self):
        """Called once per frame by the master script. Returns a dict of bot poses and calibration status."""
        success, img = self.cam.read()
        if not success:
            return {}, False, False

        # ADD THIS: Convert the frame to pure grayscale to kill color noise
        gray_frame = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        # CHANGE THIS: Pass the 'gray_frame' to the detector instead of 'img'
        corners, ids, rejectedImgPoints = self.detector.detectMarkers(gray_frame)

        
        detected_anchors = {}
        current_poses = {}

        if ids is not None:
            ids = ids.flatten()
            cv2.aruco.drawDetectedMarkers(img, corners)

            for i in range(len(ids)):
                marker_id = int(ids[i])
                marker_corners = corners[i][0]
                pixel_center = marker_corners.mean(axis=0)

                if marker_id in [100, 101, 102, 103]:
                    detected_anchors[marker_id] = pixel_center
                elif self.is_calibrated:
                    pt = np.array([[[pixel_center[0], pixel_center[1]]]], dtype=np.float32)
                    transformed_pt = cv2.perspectiveTransform(pt, self.homography_matrix)[0][0]
                    
                    raw_x, raw_y = transformed_pt[0], transformed_pt[1]
                    raw_theta = self._calculate_angle_radians(marker_corners)

                    # EMA Filter Application
                    if marker_id not in self.bot_states:
                        self.bot_states[marker_id] = {'x': raw_x, 'y': raw_y, 'theta': raw_theta}
                    else:
                        # THE MISSING LINE: Grab the previous state first!
                        prev = self.bot_states[marker_id]
                        
                        smooth_x = self.ALPHA_POS * raw_x + (1 - self.ALPHA_POS) * prev['x']
                        smooth_y = self.ALPHA_POS * raw_y + (1 - self.ALPHA_POS) * prev['y']
                        
                        # Smooth Theta (Shortest Path Calculation)
                        diff = (raw_theta - prev['theta'] + np.pi) % (2 * np.pi) - np.pi
                        
                        # The Impossible Physics Check
                        if abs(diff) > 0.8: 
                            smooth_theta = prev['theta']
                        elif abs(diff) < 0.035:
                            smooth_theta = prev['theta']
                        else:
                            smooth_theta = (prev['theta'] + self.ALPHA_THETA * diff + np.pi) % (2 * np.pi) - np.pi

                        self.bot_states[marker_id] = {'x': smooth_x, 'y': smooth_y, 'theta': smooth_theta}

                    bot_x = self.bot_states[marker_id]['x']
                    bot_y = self.bot_states[marker_id]['y']
                    bot_theta = self.bot_states[marker_id]['theta']
                    
                    current_poses[marker_id] = (bot_x, bot_y, bot_theta)

                    cx, cy = int(pixel_center[0]), int(pixel_center[1])
                    self._draw_text_with_outline(img, f"Bot {marker_id}", (cx - 50, cy - 35), 0.5, (0, 255, 0), 2)
                    self._draw_text_with_outline(img, f"{bot_theta:.2f} rad", (cx - 40, cy + 45), 0.5, (0, 255, 255), 2)

        # Keyboard & Calibration logic
        key = cv2.waitKey(1) & 0xFF
        quit_flag = (key == ord('q'))
        
        if key == ord('c'):
            if all(k in detected_anchors for k in [100, 101, 102, 103]):
                pixel_anchors = np.array([detected_anchors[100], detected_anchors[101], detected_anchors[102], detected_anchors[103]], dtype=np.float32)
                self.homography_matrix, _ = cv2.findHomography(pixel_anchors, self.WORLD_ANCHORS)
                self.is_calibrated = True
                print("[SUCCESS] Calibration locked!")

        if self.is_calibrated:
            H_inv = np.linalg.inv(self.homography_matrix)
            img = self._draw_virtual_grid(img, H_inv)

        status = "Calibrated" if self.is_calibrated else "Press 'c' to Lock"
        cv2.putText(img, status, (30, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0) if self.is_calibrated else (0, 0, 255), 2)

        display_img = cv2.resize(img, (1280, 720))
        cv2.imshow("Swarm Command Center", display_img)

        return current_poses, self.is_calibrated, quit_flag

    def close(self):
        self.cam.release()
        cv2.destroyAllWindows()