import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sim_env.src.robot_env import Panda_Sym
import warnings
import gymnasium as gym
import numpy as np
from geometry_msgs.msg import Point
from std_msgs.msg import Float64MultiArray, MultiArrayDimension, Int32, Bool

class PandaEnvNode(Node):
    def __init__(self):
        super().__init__('panda_env')
        self.state_publisher_ = self.create_publisher(Int32, 'new_state', 10)
        self.map_publisher_ = self.create_publisher(Float64MultiArray, 'map', 10)
        self.base_pos_publisher_ = self.create_publisher(Point, 'base_pos', 10)
        self.arm_pos_publisher_ = self.create_publisher(Float64MultiArray, 'arm_pos', 10)
        self.cmd_vel_subscription_ = self.create_subscription(
            Float64MultiArray,
            'cmd_vel',
            self.cmd_callback,
            10)
        self.new_state_reached_subscription_ = self.create_subscription(
            Bool,
            'goal_reached',
            self.goal_reached_callback,
            10)
        
        self.panda_sym = Panda_Sym(render=True)
        self.state_ = Int32()
        self.state_.data = 0
        # TODO: what other information or topics are needed?
        print("panda env Node has been created.")


    def cmd_callback(self, msg):
        self.get_logger().info('Got vel command in panda_env Node sub: "%s"' % msg.data)
        # Turn message into np array
        action = np.array(msg.data)
        # Call the function that moves the robot with the received action command
        self.panda_sym.move_panda(action)

    def goal_reached_callback(self, msg):
        self.get_logger().info('Goal reached')
        # Set the new value of the state
        if(self.state_.data < 3 and self.state_.data > 0):
            self.state_.data += 1
            self.state_publisher_.publish(self.state_)
        elif(self.state_ == 3):
            self.state_.data = 1

    def pub_map(self):
        ob = self.panda_sym.Get_Ob()
        occupancy_map = np.array(ob['robot_0']['Occupancy'], dtype=np.float64)
        # Create array message with the base trajectory information

        map_msg = Float64MultiArray()
        map_msg.data = occupancy_map.flatten().tolist()
        assert all(isinstance(val, float) for val in map_msg.data) # All elements must be floats
        # Define dimensions of the msg to reconstruct by the subscribers
        # Define dimensions based on the array's shape
        stride = 1
        map_msg.layout.dim = []
        for i, size in reversed(list(enumerate(occupancy_map.shape))):
            map_msg.layout.dim.insert(0, MultiArrayDimension(label=f'dim{i}', size=size, stride=stride))
            stride *= size

        # Publish base Trajectory 
        self.map_publisher_.publish(map_msg)
        self.get_logger().info('Published occupancy map')
        # Publish state 1
        if(self.state_.data == 0):
            self.state_.data = 1
            self.state_publisher_.publish(self.state_)
            print("SEEEEEEEEEEEEEEEEENT")

    def pub_base_pos(self):
        # Get the current position of the robot
        ob = self.panda_sym.Get_Ob()
        current_xyz = np.round(ob['robot_0']['joint_state']['position'][:3],4)
        
        # Create Point msg
        msg = Point()
        msg.x = current_xyz[0] + self.panda_sym.init_pos[0]
        msg.y = current_xyz[1] + self.panda_sym.init_pos[1]
        msg.z = current_xyz[2] + self.panda_sym.init_pos[2]

        # Publish current position
        self.base_pos_publisher_.publish(msg)
        self.get_logger().info('Publishing base pos from panda_env Node: "%s"' % msg)
    
    def pub_arm_pos(self):
        ob = self.panda_sym.Get_Ob()
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
        while(rclpy.ok()):
            panda_env_node.pub_map()
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