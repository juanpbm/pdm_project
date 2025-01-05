import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import CubicSpline
from urdfenvs.robots.generic_urdf import GenericUrdfReacher
from urdfenvs.urdf_common.urdf_env import UrdfEnv
from cubic_spline_definition import trajectory_planning

velocity_limit=2.5
"This python code should receive all the waypoints necessary to compute the trajectory to this variable"
target_xyz = np.array([[1, 1, 0],[2,2,0],[1,2,0],[0,0,0]])

def run_mobile_reacher(n_steps=10000, render=False, goal=True, obstacles=True):
    robots = [
        GenericUrdfReacher(urdf="mobilePanda_with_gripper.urdf", mode="vel"),
    ]
    env: UrdfEnv = UrdfEnv(
        dt=0.01, robots=robots, render=render, num_sub_steps=200,
    )
    action = np.zeros(env.n())
    ob = env.reset()

    ob, *_ = env.step(action) 
    
    current_xyz = np.round(ob['robot_0']['joint_state']['position'][:3],4)

    
    history = []
    
  
    action_to_send=[0,0,0]

    V_max = 0.1  # Maximum velocity (units per time unit)
    goal_xyz=np.vstack((current_xyz,target_xyz))
    print(goal_xyz)
    coordinates_trajectory=trajectory_planning(goal_xyz,V_max,500)
    print(len(coordinates_trajectory))
    print(coordinates_trajectory[0])
    # Plot the original via-points and the interpolated trajectory
    plt.figure(figsize=(8, 6))
    plt.plot(goal_xyz[:, 0],goal_xyz[:, 1], 'ro', label='Via-Points')
    plt.plot(coordinates_trajectory[:,0],coordinates_trajectory[:,1], 'b-', label='Trajectory')
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.title('Via-Points Trajectory Profile with Maximum Velocity Constraint')
    plt.legend()
    plt.grid(True)
    plt.show()

    Kp=5
    Kd=2
    e_prev=[0,0]
    time_prev=0
    for i in range(n_steps):
        if (len(coordinates_trajectory)>i): 
           
            error_xyz = coordinates_trajectory[i] - current_xyz[:2]
            Derivative_error=Kd*(error_xyz - e_prev)/(i+1 - time_prev)
            desired_velocity_xyz = Kp*error_xyz + Derivative_error
            action_to_send = desired_velocity_xyz

            e_prev = error_xyz
            time_prev = i
        elif(len(coordinates_trajectory)+100>i):   
            error_xyz = coordinates_trajectory[-1] - current_xyz[:2]
            Derivative_error=Kd*(error_xyz - e_prev)/(i+1 - time_prev)
            desired_velocity_xyz = Kp*error_xyz + Derivative_error
            action_to_send = desired_velocity_xyz

            e_prev = error_xyz
            time_prev = i
            
        else:
            action_to_send = 0
            print(current_xyz)

        "Action is the variable with the taregt velocities for the robot"
        action[:2]=action_to_send
        "This is what actually gives the command for the robot to move, so I suppose this is what this node should emmit"
        ob, *_ = env.step(action) 
        current_xyz = np.round(ob['robot_0']['joint_state']['position'][:3],4)  
        history.append(ob)
    env.close()

    

    return history


if __name__ == "__main__":
    run_mobile_reacher(render=True)