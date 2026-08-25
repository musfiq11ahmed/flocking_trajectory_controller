import csv
import math
import socket
from kinematic_controller.unicycle_model_v1 import calculate_wheel_velocities
from vision_pipeline.calibrate_and_track_v3 import SwarmVision


# 1. SWARM CONFIGURATION

UDP_IP = "192.168.50.105"
UDP_PORT = 4210
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
TARGET_BOT_ID = 2  # The ArUco marker ID attached to Test Bot 1

def load_trajectory(filepath):
    """Reads the scaled x, y, theta coordinates from the CSV."""
    trajectory = []
    print(f"Loading trajectory from {filepath}...")
    with open(filepath, 'r') as file:
        reader = csv.reader(file)
        next(reader, None) # Skip header
        for row in reader:
            trajectory.append((float(row[0]), float(row[1]), float(row[2])))
    return trajectory

def main():
    # Load the CSV
    trajectory = load_trajectory("dummy_trajectory/scaled_s_trajectory.csv")
    current_waypoint_idx = 0
    
    # Initialize the Vision Pipeline Class
    tracker = SwarmVision()
    print("[INFO] Vision Pipeline initialized. Waiting for calibration...")
    
    try:
        while True:
            # 1. Grab the latest data from the camera module
            active_poses, is_calibrated, quit_flag = tracker.update()
            
            if quit_flag:
                break
                
            # 2. Wait until the user presses 'c' to lock the arena
            if not is_calibrated:
                continue
                
            # 3. Emergency Stop if the camera loses sight of the robot
            if TARGET_BOT_ID not in active_poses:
                sock.sendto(b"TARGET,0.0,0.0", (UDP_IP, UDP_PORT))
                continue
                
            # 4. Extract current coordinates
            robot_x, robot_y, robot_theta = active_poses[TARGET_BOT_ID]
            
            # 5. Execute Trajectory
            if current_waypoint_idx < len(trajectory):
                target_x, target_y, target_theta = trajectory[current_waypoint_idx]
                
                # Math module processes the spatial error
                left_mps, right_mps, arrived = calculate_wheel_velocities(
                    robot_x, robot_y, robot_theta, target_x, target_y
                )
                
                if arrived:
                    current_waypoint_idx += 1
                else:
                    msg = f"TARGET,{left_mps:.2f},{right_mps:.2f}"
                    sock.sendto(msg.encode(), (UDP_IP, UDP_PORT))
            else:
                sock.sendto(b"TARGET,0.0,0.0", (UDP_IP, UDP_PORT))
                print("Trajectory Complete.")
                break

    except KeyboardInterrupt:
        print("Script interrupted.")
        
    finally:
        sock.sendto(b"TARGET,0.0,0.0", (UDP_IP, UDP_PORT))
        tracker.close()

if __name__ == "__main__":
    main()