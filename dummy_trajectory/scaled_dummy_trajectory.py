import pandas as pd

# 1. Load the specific synthetic CSV file
input_file = r"F:\Research Assistant\NIRO Lab\CTRG_RLC\1. Hardware\test_swarm_controller\dummy_trajectory\dummy_random_trajectory.csv"
df = pd.read_csv(input_file)

# 2. Find the global bounding box of the trajectory
x_min, x_max = df['x'].min(), df['x'].max()
y_min, y_max = df['y'].min(), df['y'].max()

x_range = x_max - x_min
y_range = y_max - y_min

# 3. Define the exact physical arena limits
ARENA_WIDTH = 2.7  
ARENA_HEIGHT = 1.67 

# 4. Calculate the uniform scale factor
scale_factor = min(ARENA_WIDTH / x_range, ARENA_HEIGHT / y_range)

# 5. Calculate the dead center of the simulation data
x_center = (x_max + x_min) / 2.0
y_center = (y_max + y_min) / 2.0

# 6. Apply the transformation: Shift to (1.35, 0) center, then scale
# X shifts the origin to the left wall (0 to 2.7)
df['x_scaled'] = ((df['x'] - x_center) * scale_factor) + (ARENA_WIDTH / 2.0)
# Y remains centered (-0.835 to 0.835)
df['y_scaled'] = (df['y'] - y_center) * scale_factor

# 7. Create a clean dataframe for the hardware controller
hardware_df = pd.DataFrame({
    'x': df['x_scaled'],
    'y': df['y_scaled'],
    'theta': df['theta']
})

# 8. Save the hardware-ready data
output_file = "scaled_dummy_trajectory.csv"
hardware_df.to_csv(output_file, index=False)

print("[SUCCESS] Trajectory Scaled and Shifted!")
print(f"Scale Factor Applied: {scale_factor:.4f}")
print(f"New X Bounds: {hardware_df['x'].min():.3f}m to {hardware_df['x'].max():.3f}m")
print(f"New Y Bounds: {hardware_df['y'].min():.3f}m to {hardware_df['y'].max():.3f}m")
print(f"Saved to: {output_file}")