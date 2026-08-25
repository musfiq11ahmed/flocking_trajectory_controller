import csv
import math
import socket
from kinematic_controller.unicycle_model_v2 import calculate_wheel_velocities
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
    try:
        with open(filepath, 'r') as file:
            reader = csv.reader(file)
            next(reader, None) # Skip header
            for row in reader:
                trajectory.append((float(row[0]), float(row[1]), float(row[2])))
        return trajectory
    except Exception as e:
        print(f"Failed to load CSV: {e}")
        return []

def main():
    # Load CSV file
    
    trajectory = load_trajectory("dummy_trajectory/scaled_dummy_trajectory.csv")
    current_waypoint_idx = 0
    
    
    # INITIALIZE VISION & MEMORY
    
    tracker = SwarmVision()
    print("[INFO] Vision Pipeline initialized. Waiting for calibration...")
    
    lost_frames = 0
    MAX_LOST_FRAMES = 15  
    last_pose = None
    last_velocity = (0.0, 0.0, 0.0) # (vx, vy, vtheta)

    # Acceleration limits
    current_cmd_left = 0.0
    current_cmd_right = 0.0
    MAX_ACCEL = 0.015  # Max m/s change per frame (prevents wheel slip)
    
    try:
        while True:
            # 1. Grab the latest data from the camera module
            active_poses, is_calibrated, quit_flag = tracker.update()
            
            if quit_flag:
                break
                
            # 2. Wait until the user presses 'c' to lock the arena
            if not is_calibrated:
                continue
                
            
            # 3. OPTICAL DEAD RECKONING
            
            if TARGET_BOT_ID not in active_poses:
                lost_frames += 1
                if lost_frames > MAX_LOST_FRAMES or last_pose is None:
                    # Lost for too long, predicting is too dangerous. Emergency stop.
                    sock.sendto(b"TARGET,0.0,0.0", (UDP_IP, UDP_PORT))
                    continue
                else:
                    # PREDICTIVE TRACKING: Project the robot forward using its last known velocity
                    rx, ry, rtheta = last_pose
                    vx, vy, vtheta = last_velocity
                    
                    rx += vx
                    ry += vy
                    rtheta += vtheta
                    
                    
                    last_pose = (rx, ry, rtheta)
                    robot_x, robot_y, robot_theta = last_pose
            else:
                current_pose = active_poses[TARGET_BOT_ID]
                
                # If it was seen last frame, calculate its velocity vector
                if last_pose is not None and lost_frames == 0:
                    vx = current_pose[0] - last_pose[0]
                    vy = current_pose[1] - last_pose[1]
                    # Calculate shortest angular velocity
                    vtheta = (current_pose[2] - last_pose[2] + math.pi) % (2 * math.pi) - math.pi
                    last_velocity = (vx, vy, vtheta)
                
                # Update memory with the hard visual truth
                last_pose = current_pose
                lost_frames = 0
                robot_x, robot_y, robot_theta = last_pose
                
            
            # 4. EXECUTE TRAJECTORY
            
            if current_waypoint_idx < len(trajectory):
                target_x, target_y, target_theta = trajectory[current_waypoint_idx]
                
                # Math module processes the spatial error
                target_left, target_right, arrived = calculate_wheel_velocities(
                    robot_x, robot_y, robot_theta, target_x, target_y
                )
                
                
                # ACCELERATION RAMPING (The Anti-Jerk Filter)
                
                # Smoothly ramp the left wheel
                if target_left > current_cmd_left + MAX_ACCEL:
                    current_cmd_left += MAX_ACCEL
                elif target_left < current_cmd_left - MAX_ACCEL:
                    current_cmd_left -= MAX_ACCEL
                else:
                    current_cmd_left = target_left
                    
                # Smoothly ramp the right wheel
                if target_right > current_cmd_right + MAX_ACCEL:
                    current_cmd_right += MAX_ACCEL
                elif target_right < current_cmd_right - MAX_ACCEL:
                    current_cmd_right -= MAX_ACCEL
                else:
                    current_cmd_right = target_right
                
                # Execution
                if arrived:
                    current_waypoint_idx += 1
                else:
                    msg = f"TARGET,{current_cmd_left:.2f},{current_cmd_right:.2f}"
                    sock.sendto(msg.encode(), (UDP_IP, UDP_PORT))
            else:
                # Smoothly decelerate to zero when trajectory is done
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