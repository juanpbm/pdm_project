import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sim_env.src.robot_env import Panda_Sym
import warnings
import gymnasium as gym
import numpy as np
from std_msgs.msg import String
from geometry_msgs.msg import Point
from std_msgs.msg import Float64MultiArray

class PandaEnvNode(Node):
    def __init__(self):
        super().__init__('panda_env')
        self.map_publisher_ = self.create_publisher(String, 'map', 10)
        self.base_pos_publisher_ = self.create_publisher(Point, 'base_pos', 10)
        self.arm_pos_publisher_ = self.create_publisher(Float64MultiArray, 'arm_pos', 10)
        self.cmd_vel_subscription = self.create_subscription(
            Float64MultiArray,
            'cmd_vel',
            self.cmd_callback,
            10)
        
        self.panda_sym = Panda_Sym(render=True)
        # TODO: what other information or topics are needed?
        print("panda env Node has been created.")


    def cmd_callback(self, msg):
        self.get_logger().info('Got in panda_env Node sub: "%s"' % msg.data)
        # Turn message into np array
        action = np.array(msg.data)
        # Call the function that moves the robot with the received action command
        self.panda_sym.move_panda(action)


    def pub_map(self):
        msg_new = String()
        msg_new.data = 'map'
        self.map_publisher_.publish(msg_new)
        self.get_logger().info('Publishing from panda_env Node map: "%s"' % msg_new.data)

    def pub_base_pos(self):
        # Get the current position of the robot
        ob = self.panda_sym.Get_Ob()
        current_xyz = np.round(ob['robot_0']['joint_state']['position'][:3],4)
        
        # Create Point msg
        msg = Point()
        msg.x = current_xyz[0]+4.0
        msg.y = current_xyz[1]-14.0
        msg.z = current_xyz[2]

        # Publish current position
        self.base_pos_publisher_.publish(msg)
        self.get_logger().info('Publishing base pos from panda_env Node: "%s"' % msg)
    
    def pub_arm_pos(self):
        ob = self.panda_sym.Get_Ob()
        print(ob)
        current_joint_pos = np.round(ob['robot_0']['joint_state']['position'][3:-2],4)
        msg = Float64MultiArray()
        msg.data = current_joint_pos.astype(np.float64).tolist()

        # Publish cmd_vel msg
        self.arm_pos_publisher_.publish(msg)
        self.get_logger().info('Publishing arm pos from panda_env Node: "%s"' % msg.data)

    #Functions that use the env class
    
def main(args=None):
    # start the panda_env node
    try:
        rclpy.init(args=args)
        panda_env_node = PandaEnvNode()
        panda_env_node.pub_map()
        while(rclpy.ok()):
            panda_env_node.pub_base_pos()
            panda_env_node.pub_arm_pos()
            rclpy.spin_once(panda_env_node, timeout_sec=5)

    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if panda_env_node is not None:
            panda_env_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
            
        print("panda env Node has been shut down.")

if __name__ == "__main__":
    # Call Main Function
    main()