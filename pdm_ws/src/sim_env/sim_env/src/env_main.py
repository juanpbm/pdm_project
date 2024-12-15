from robot_env import Albert_sym
import warnings
import gymnasium as gym
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import String

if __name__ == "__main__":
    show_warnings = False
    warning_flag = "default" if show_warnings else "ignore"
    with warnings.catch_warnings():
        warnings.filterwarnings(warning_flag)
        # Define state machine
        # Define robot_env and 
        # Use node to perform actions 
        # albert_sym = Albert_sym(render=True)
        # albert_sym.move()