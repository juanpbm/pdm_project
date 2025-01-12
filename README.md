# Final Project 
# Planning and Decision Making RO47005 
# 2024/2025 TU Delft

| Student | Student Number |
|---|---|
|Abhilash, Adithya | 6187781 |
|Bernal Medina, Juan | 6202845 |
|Gutierrez Jimenez, Jose | 6296793 |
|Redondo Garcia, Zuleika | 6290418 |

## Description
This Code contains the code for the final project of Planning and Decision Making RO47005. In this project, a simulation environment was created to simulate a warehouse. In this environment, a mobile manipulator had to find a viable path from a start position to a goal and back. This ROS2 workspace contains three packages to achieve this goal. The `sim_env` manages the simulation environment and state machine, `motion_planning` computes trajectories for the arm and the base, and `control` computes the velocities needed to achieve the goals. 

## How to Run
To run this workspace ROS2 Humble was used. After the setup bash has been sourced and an internet connection is available run the following commands:

```
git clone https://github.com/juanpbm/pdm_project.git
cd </path/to>/pdm_ws 
rosdep install --from-paths src --ignore-src -r -y
colcon build
source install/setup.bash
ros2 launch sim_env pdm_project.launch.xml
```

### Known bugs.
1. From time to time the RRT* computation generates a `segmentation fault` if this happens just restart the simulation and relaunch the project. Due to time constraints, we could not fully debug this issue. The error appears to be caused after increasing the RRT* iterations beyond the first path found. Since it occurs rarely and resolves upon restarting, we decided to leave it as is, given that the system works reliably most of the time.
2. The arm consistently starts moving randomly midway between the start and pickup locations, despite being sent zero-velocity commands, likely due to an unknown issue with the environment management. This unexpected movement occasionally interferes with the base's movements, preventing it from completing the trajectory. A rare occurrence similar to the previous issue, which resolves upon restarting.

