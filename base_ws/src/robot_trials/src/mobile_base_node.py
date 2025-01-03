#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import numpy as np
#from urdfpy import URDF
from scipy.interpolate import CubicSpline
from urdfenvs.robots.generic_urdf import GenericUrdfReacher
from urdfenvs.urdf_common.urdf_env import UrdfEnv

from robot_trials.msg import BasePlan

velocity_limit=2.5
"This python code should receive all the waypoints necessary to compute the trajectory to this variable"
#target_xyz = np.array([[0.5,0.5,0],[4.43,0.07,0],[4.78,4.42,0],[3.25, 4.50, 0]])

n_steps = 10000
render = True
obstacles = True


class BaseMove(Node):

    def __init__(self):
        super().__init__('mobile_base_node')
        self.subscription = self.create_subscription(BasePlan,'/base_points', self.run_mobile_reacher,10)
        self.subscription  # prevent unused variable warning

    def run_mobile_reacher(self, msg):
        target_xyz = []
        for i in range(0,len(msg.pos)-1,2):
            target_xyz.append((msg.pos[i],msg.pos[i+1],0))

        target_xyz = np.asarray(target_xyz)
 
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

def main(args=None):
    rclpy.init(args=args)

    base_move = BaseMove()

    rclpy.spin(base_move)

    base_move.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
