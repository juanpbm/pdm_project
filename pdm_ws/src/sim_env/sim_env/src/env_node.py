from geometry_msgs.msg import Point
import gymnasium as gym
import numpy as np
import rclpy
import time
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sim_env.src.robot_env import Panda_Sym
from std_msgs.msg import Float64MultiArray, MultiArrayDimension, Int32, Bool
import warnings

class PandaEnvNode(Node):
    def __init__(self):
        super().__init__('panda_env')
        self.state_publisher_ = self.create_publisher(Int32, 'new_state', 10)
        self.map_publisher_ = self.create_publisher(Float64MultiArray, 'map', 10)
        self.base_pos_publisher_ = self.create_publisher(Point, 'base_pos', 10)
        self.arm_pos_publisher_ = self.create_publisher(Float64MultiArray, 'arm_pos', 10)
        self.goal_pos_publisher_ = self.create_publisher(Point, 'goal_pos', 10)
        self.init_pos_publisher_ = self.create_publisher(Point, 'init_pos', 10)
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
        self.prev_pos = None
        self.position_xyz = None
        self.prev_vel = None
        self.velocity_xyz = None
        self.start_acc_time = 0.0
        self.end_acc_time = 0.0
        self.first = True
        self.panda_sym = Panda_Sym(render=True)
        self.state_ = Int32()
        self.state_.data = 0

        self.accelerations = []
        self.dist_time = []
        # TODO: what other information or topics are needed?
        print("panda env Node has been created.")


    def cmd_callback(self, msg):
        # Turn message into np array
        action = np.array(msg.data)
        self.get_logger().info('Got vel command in panda_env Node sub: "%s"' % action)
        # Call the function that moves the robot with the received action command
        self.panda_sym.move_panda(action)

    def goal_reached_callback(self, msg):
        self.get_logger().info('Goal reached')
        # Set the new value of the state
        if(self.state_.data < 6 and self.state_.data > 0):
            self.state_.data += 1
            if(self.state_.data==3):
                self.panda_sym.reset_robot_arm()
            self.state_publisher_.publish(self.state_)
        elif(self.state_ == 6):
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
        if(self.first == False):
            self.end_acc_time = time.time()
            self.position_xyz =  np.round(ob['robot_0']['joint_state']['position'][:3],4)
            self.velocity_xyz = np.round(ob['robot_0']['joint_state']['velocity'][:3],4)
            if(np.round(np.linalg.norm(self.velocity_xyz)) > 0 and np.round(np.linalg.norm(self.prev_vel) > 0)):
                self.accelerations.append((self.velocity_xyz - self.prev_vel)/(self.end_acc_time - self.start_acc_time))


            if np.linalg.norm(self.position_xyz - self.prev_pos) > 0.0 :
                self.dist_time.append((np.linalg.norm(self.position_xyz - self.prev_pos))/(self.end_acc_time - self.start_acc_time))
                print(np.linalg.norm(self.prev_pos))
                print(np.linalg.norm(self.position_xyz))

            self.prev_pos = self.position_xyz
            self.prev_vel = self.velocity_xyz
            self.start_acc_time = self.end_acc_time

        else:
            self.start_acc_time = time.time()
            self.prev_pos =  np.round(ob['robot_0']['joint_state']['position'][:3],4)
            self.prev_vel = np.round(ob['robot_0']['joint_state']['velocity'][:3],4)
            self.first = False
    
    def pub_arm_pos(self):
        ob = self.panda_sym.Get_Ob()
        current_joint_pos = np.round(ob['robot_0']['joint_state']['position'][3:-2],4)
        msg = Float64MultiArray()
        msg.data = current_joint_pos.astype(np.float64).tolist()
        
        # Publish cmd_vel msg
        self.arm_pos_publisher_.publish(msg)

        self.get_logger().info('Publishing arm pos from panda_env Node: "%s"' % msg.data)

    def pub_goal_pos(self):
        # Get the current position of the robot
        goal_pos = np.array(self.panda_sym.Get_Goal_Pos(), dtype=float)

        # Create Point msg
        msg = Point()
        msg.x = goal_pos[0] 
        msg.y = goal_pos[1] 
        msg.z = goal_pos[2]
        # Publish current position
        self.goal_pos_publisher_.publish(msg)
        self.get_logger().info('Publishing goal pos from panda_env Node: "%s"' % msg)

    def pub_init_pos(self):
        # Get the current position of the robot
        init_pos = np.array(self.panda_sym.Get_Init_Pos(), dtype=float)

        # Create Point msg
        msg = Point()
        msg.x = init_pos[0] 
        msg.y = init_pos[1] 
        msg.z = init_pos[2]
        # Publish current position
        self.init_pos_publisher_.publish(msg)
        self.get_logger().info('Publishing init pos from panda_env Node: "%s"' % msg)

    #Functions that use the env class
    
def main(args=None):
    # start the panda_env node
    start_time = time.time()
    try:
        rclpy.init(args=args)
        panda_env_node = PandaEnvNode()
        while(rclpy.ok()):
            panda_env_node.pub_goal_pos()
            panda_env_node.pub_init_pos()
            panda_env_node.pub_map()
            panda_env_node.pub_base_pos()
            panda_env_node.pub_arm_pos()
            rclpy.spin_once(panda_env_node, timeout_sec=5)
            if(panda_env_node.state_.data == 6):
                break

    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if panda_env_node is not None:
            panda_env_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

        aver_dist_time = np.round(np.mean(np.abs(panda_env_node.dist_time)),3)
        print("Average distance/time: " + str(aver_dist_time))
        diff_acc = np.diff(panda_env_node.accelerations)
        aver_acc = np.mean(np.abs(diff_acc))
        print("Smoothness value: " + str(aver_acc))

        end_time = time.time() 
        execution_time = end_time - start_time
        print("Time: " + str(np.round(execution_time,3)) + "s")
        print("panda env Node has been shut down.")

if __name__ == "__main__":
    # Call Main Function
    main()