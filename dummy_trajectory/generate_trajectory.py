import csv
import random
import math
import os

# ==========================================
# 1. SIMULATION PARAMETERS
# ==========================================
dt = 0.02          # 50Hz timestep
duration = 10.0    # 10 seconds of simulated flight time
steps = int(duration / dt)

# Starting state of the virtual robot
x, y, theta = 0.0, 0.0, 0.0
v = 0.2     # Constant forward speed (0.2 m/s)
omega = 0.0 # Initial turn rate

filename = "dummy_random_trajectory.csv"
filepath = os.path.join(os.path.dirname(__file__), filename)

print("[INFO] Generating kinematic random walk trajectory...")

with open(filepath, mode='w', newline='') as file:
    writer = csv.writer(file)
    writer.writerow(['x', 'y', 'theta'])
    
    for _ in range(steps):
        # 1. Record the current state
        writer.writerow([round(x, 4), round(y, 4), round(theta, 4)])
        
        # 2. Randomly adjust the steering (omega) a tiny bit
        # This acts like a driver smoothly wiggling the steering wheel
        omega += random.uniform(-0.1, 0.1)
        
        # Clamp omega so the robot doesn't spin out of control
        omega = max(-1.0, min(1.0, omega)) 
        
        # 3. Calculate where the robot will be next frame (Dead Reckoning math)
        x += v * math.cos(theta) * dt
        y += v * math.sin(theta) * dt
        theta += omega * dt
        
        # Normalize Theta to stay strictly within [-pi, pi]
        theta = (theta + math.pi) % (2 * math.pi) - math.pi

print(f"[SUCCESS] {filename} created using random walk.")