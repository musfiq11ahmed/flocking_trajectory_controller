import numpy as np
import pandas as pd
import os


# 1. ARENA & TRAJECTORY PARAMETERS
# Generate a parametric S-curve that natively fits the arena

num_points = 250

# Keep X safely away from the physical walls (Arena is 0.0 to 2.7)
x_start = 0.3
x_end = 2.4

# Keep Y safely inside the limits (Arena is -0.835 to 0.835)
y_amplitude = 0.6  

# Generate the parameter 't' from 0 to 2*pi (One full sine wave = one "S" shape)
t = np.linspace(0, 2 * np.pi, num_points)


# 2. CALCULATE X, Y, AND THETA

# X moves linearly forward from start to end
x = x_start + (x_end - x_start) * (t / (2 * np.pi))

# Y oscillates to create the S-curve
y = y_amplitude * np.sin(t)

# Heading (Theta) is the arctangent of the derivatives (dy/dt over dx/dt)
dx_dt = (x_end - x_start) / (2 * np.pi)
dy_dt = y_amplitude * np.cos(t)

theta = np.arctan2(dy_dt, dx_dt)


# 3. EXPORT CSV

df = pd.DataFrame({
    'x': x,
    'y': y,
    'theta': theta
})


filename = "scaled_s_trajectory.csv"
filepath = os.path.join(os.path.dirname(__file__), filename)
df.to_csv(filepath, index=False)

print(f"[SUCCESS] S-Curve generated natively! Saved to: {filepath}")
print(f"X bounds: {x.min():.2f}m to {x.max():.2f}m")
print(f"Y bounds: {y.min():.2f}m to {y.max():.2f}m")