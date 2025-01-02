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
        action = np.array(msg.data)
        self.panda_sym.move_panda(action)
        # act on received command
        # get new info to publish 


    def pub_map(self):
        msg_new = String()
        msg_new.data = 'map'
        self.map_publisher_.publish(msg_new)
        self.get_logger().info('Publishing from panda_env Node map: "%s"' % msg_new.data)

    def pub_base_pos(self):
        ob = self.panda_sym.Get_Ob()
        current_xyz = np.round(ob['robot_0']['joint_state']['position'][:3],4)
        msg = Point()
        msg.x = current_xyz[0]
        msg.y = current_xyz[1]
        msg.z = current_xyz[2]

        print('ob data current pos: ', current_xyz)
        
        self.base_pos_publisher_.publish(msg)
        self.get_logger().info('Publishing from panda_env Node: "%s"' % msg)
    
    #Functions that use the env class
    
def main(args=None):
    # start the panda_env node
    try:
        rclpy.init(args=args)
        panda_env_node = PandaEnvNode()
        while(rclpy.ok()):
            panda_env_node.pub_map()
            panda_env_node.pub_base_pos()
            rclpy.spin_once(panda_env_node)

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