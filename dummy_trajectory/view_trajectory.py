import csv
import os
import math
import matplotlib.pyplot as plt


# 1. LOAD THE DATA

filename = "scaled_s_trajectory.csv"
filepath = os.path.join(os.path.dirname(__file__), filename)

x_data = []
y_data = []
theta_data = []

print(f"[INFO] Loading data from {filename}...")

with open(filepath, mode='r') as file:
    reader = csv.DictReader(file)
    for row in reader:
        x_data.append(float(row['x']))
        y_data.append(float(row['y']))
        theta_data.append(float(row['theta']))


# 2. PLOT THE TRAJECTORY

plt.figure(figsize=(10, 8))

# Plot the continuous path
plt.plot(x_data, y_data, label='Robot Path', color='royalblue', linewidth=2)

# Mark the starting point
plt.plot(x_data[0], y_data[0], 'go', markersize=8, label='Start')

# Mark the ending point
plt.plot(x_data[-1], y_data[-1], 'ro', markersize=8, label='End')


# 3. ADD HEADING ARROWS (THETA)

# Draw an arrow every 25 frames (every 0.5 seconds).
step = 25 

# Calculate the arrow direction vectors using trigonometry
u_data = [math.cos(t) for t in theta_data]
v_data = [math.sin(t) for t in theta_data]

# Use quiver to draw the arrows
plt.quiver(
    x_data[::step], y_data[::step], 
    u_data[::step], v_data[::step], 
    color='darkorange', scale=15, width=0.005, 
    label='Heading (Theta)', zorder=3
)


# 4. FORMATTING

plt.title('Kinematic Random Walk Trajectory', fontsize=14, fontweight='bold')
plt.xlabel('X Position (meters)', fontsize=12)
plt.ylabel('Y Position (meters)', fontsize=12)

# Enforce an equal aspect ratio so circles look like circles and curves are true to life
plt.axis('equal') 

# Locking the plot directly to exact physical arena dimensions
plt.xlim(0.0, 2.7)        # Left wall at 0, Right wall at 2.7
plt.ylim(-1.0, 1.0)       # Physical Y is -0.835 to 0.835, padded slightly to 1.0 so arrows fit

plt.grid(True, linestyle='--', alpha=0.7)
plt.legend(loc='best')

print("[SUCCESS] Displaying plot. Close the window to exit.")
plt.show()