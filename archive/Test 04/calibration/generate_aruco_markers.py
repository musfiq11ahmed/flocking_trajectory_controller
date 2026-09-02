"""
generate_aruco_markers.py — Print ArUco markers for the arena and robot.

Run this script and print the generated images on MATTE paper (no gloss).
Cut them out and tape/fix them at the arena corners and on the robot.

    python generate_aruco_markers.py
"""

import cv2
import cv2.aruco as aruco
import os

OUTPUT_DIR = "markers"
DICT_ID = aruco.DICT_4X4_50
MARKER_SIZE_PX = 400        # pixels per marker image

# IDs to generate
MARKERS = {
    0: "arena_bottom_left",
    1: "arena_bottom_right",
    2: "arena_top_right",
    3: "arena_top_left",
    4: "robot",
}


def main():
    os.makedirs(OUTPUT_DIR, exist_ok=True)
    aruco_dict = aruco.getPredefinedDictionary(DICT_ID)

    for marker_id, label in MARKERS.items():
        img = aruco.generateImageMarker(aruco_dict, marker_id, MARKER_SIZE_PX)
        # Add white border for easier detection
        bordered = cv2.copyMakeBorder(
            img, 40, 40, 40, 40,
            cv2.BORDER_CONSTANT, value=255)
        
        filename = os.path.join(OUTPUT_DIR, f"marker_{marker_id}_{label}.png")
        cv2.imwrite(filename, bordered)
        print(f"  Saved {filename}  (ID={marker_id})")

    print(f"\n{len(MARKERS)} markers saved to '{OUTPUT_DIR}/'.")
    print("Print on MATTE paper.  Recommended physical sizes:")
    print("  Arena corners:  8–10 cm")
    print("  Robot marker :  5–8 cm")


if __name__ == "__main__":
    main()
