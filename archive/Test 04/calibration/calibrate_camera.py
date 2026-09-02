"""
calibrate_camera.py — Optional camera intrinsic calibration using a
                       printed checkerboard pattern.

This script is NOT required for the homography-based arena system to work,
but it can improve accuracy by undistorting the image before ArUco detection.

Usage
-----
    1. Print a checkerboard (e.g. 9×6 inner corners).
    2. Run:  python calibrate_camera.py
    3. Show the board to the camera from 15–20 different angles.
    4. Press 'c' to capture a frame.  Repeat until you have ≥15 captures.
    5. Press 'q' to compute calibration and save to camera_calib.npz.
"""

import cv2
import numpy as np

# Checkerboard inner-corner dimensions (columns × rows)
BOARD_COLS = 9
BOARD_ROWS = 6
SQUARE_SIZE_MM = 25          # physical size of one square

CAMERA_INDEX = 0
SAVE_FILE = "camera_calib.npz"


def main():
    cap = cv2.VideoCapture(CAMERA_INDEX, cv2.CAP_DSHOW)
    if not cap.isOpened():
        cap = cv2.VideoCapture(CAMERA_INDEX)
    if not cap.isOpened():
        print("Cannot open camera.")
        return

    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  1280)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)

    # Prepare 3-D object points
    objp = np.zeros((BOARD_ROWS * BOARD_COLS, 3), np.float32)
    objp[:, :2] = np.mgrid[0:BOARD_COLS, 0:BOARD_ROWS].T.reshape(-1, 2)
    objp *= SQUARE_SIZE_MM

    obj_points = []      # 3-D points
    img_points = []      # 2-D image points

    print("Show the checkerboard from different angles.")
    print("  c = capture   |   q = finish & compute calibration")

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        found, corners = cv2.findChessboardCorners(gray, (BOARD_COLS, BOARD_ROWS), None)

        vis = frame.copy()
        if found:
            cv2.drawChessboardCorners(vis, (BOARD_COLS, BOARD_ROWS), corners, found)

        cv2.putText(vis, f"Captures: {len(obj_points)}   (c=capture, q=compute)",
                    (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        cv2.imshow("Camera Calibration", vis)

        key = cv2.waitKey(1) & 0xFF
        if key == ord("c") and found:
            corners2 = cv2.cornerSubPix(
                gray, corners, (11, 11), (-1, -1),
                (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001))
            obj_points.append(objp)
            img_points.append(corners2)
            print(f"  Captured frame #{len(obj_points)}")
        elif key == ord("q"):
            break

    cap.release()
    cv2.destroyAllWindows()

    if len(obj_points) < 5:
        print("Need at least 5 captures.  Aborting.")
        return

    print(f"\nCalibrating with {len(obj_points)} images …")
    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(
        obj_points, img_points, gray.shape[::-1], None, None)

    print(f"RMS re-projection error: {ret:.4f}")
    print(f"Camera matrix:\n{mtx}")
    print(f"Distortion coefficients:\n{dist.ravel()}")

    np.savez(SAVE_FILE, camera_matrix=mtx, dist_coeffs=dist)
    print(f"\nSaved to {SAVE_FILE}")


if __name__ == "__main__":
    main()
