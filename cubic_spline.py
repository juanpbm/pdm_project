import numpy as np
import matplotlib.pyplot as plt
from cubic_spline_definition import trajectory_planning

# Define the via-points (waypoints) and maximum velocity
via_points = np.array([
    [0, 0],   # Start point
    [1, 2],   # First via-point
    [2, 3],   # Second via-point
    [3, 5],   # Third via-point
    [4, 4],   # Fourth via-point
    [5, 6]    # End point
])
V_max = 1.0  # Maximum velocity (units per time unit)

coordinates_trajectory=trajectory_planning(via_points,V_max,100)
print(coordinates_trajectory[0])
# Plot the original via-points and the interpolated trajectory
plt.figure(figsize=(8, 6))
plt.plot(via_points[:, 0],via_points[:, 1], 'ro', label='Via-Points')
plt.plot(coordinates_trajectory[:,0],coordinates_trajectory[:,1], 'b-', label='Trajectory')
plt.xlabel('X')
plt.ylabel('Y')
plt.title('Via-Points Trajectory Profile with Maximum Velocity Constraint')
plt.legend()
plt.grid(True)
plt.show()
