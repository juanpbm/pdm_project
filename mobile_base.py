import numpy as np
from urdfpy import URDF
from scipy.interpolate import CubicSpline
from urdfenvs.robots.generic_urdf import GenericUrdfReacher
from urdfenvs.urdf_common.urdf_env import UrdfEnv

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
    x=0
    for i in range(n_steps):
        if (np.linalg.norm(target_xyz[x] - current_xyz) > 0.1): 
           
            error_xyz = target_xyz[x] - current_xyz
            desired_velocity_xyz = 1*error_xyz
            
            action_to_send = desired_velocity_xyz

        elif (x < target_xyz.shape[0]-1 ):
            x+=1
        else:
            action_to_send = 0

        "Action is the variable with the taregt velocities for the robot"
        action[:3]=action_to_send
        "This is what actually gives the command for the robot to move, so I suppose this is what this node should emmit"
        ob, *_ = env.step(action) 
        current_xyz = np.round(ob['robot_0']['joint_state']['position'][:3],4)  
        history.append(ob)
    env.close()

    

    return history


if __name__ == "__main__":
    run_mobile_reacher(render=True)