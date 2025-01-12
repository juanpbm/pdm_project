from control.src.control import Controller
from geometry_msgs.msg import Point
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Int32, Bool
from control.src.control import Controller

class ControlNode(Node):
    def __init__(self):
        super().__init__('control')
        self.cmd_vel_publisher_ = self.create_publisher(Float64MultiArray, 'cmd_vel', 10)
        self.goal_reached_ = self.create_publisher(Bool, 'goal_reached', 10)
        self.base_trajectory_subscription = self.create_subscription(
            Float64MultiArray,
            'base_trajectory',
            self.base_trajectory_callback,
            10)
        self.arm_trajectory_subscription = self.create_subscription(
            Float64MultiArray,
            'arm_trajectory',
            self.arm_trajectory_callback,
            10)
        self.base_pos_subscription = self.create_subscription(
            Point,
            'base_pos',
            self.base_pos_callback,
            10)
        self.arm_pos_subscription = self.create_subscription(
            Float64MultiArray,
            'arm_pos',
            self.arm_pos_callback,
            10)
        self.new_state_subscription_ = self.create_subscription(
            Int32,
            'new_state',
            self.new_state_callback,
            10)
        
        self.state_ = 0

        self.base_current_pos = np.empty(0)
        self.arm_current_pos = np.empty(0)
        self.base_trajectory = np.empty(0)
        self.arm_trajectory = np.empty(0)
        self.base_target_reached = False
        self.arm_target_reached = False
        self.new_trajectory = False
        self.base_waypoint = 0
        self.arm_waypoint = 0
        self.controller = Controller()

        self.get_logger().info("control Node has been created.")

    def base_trajectory_callback(self, msg):
        
        # Recover trajectory information as a 2D Array
        trajectory_data = msg.data
        rows = msg.layout.dim[0].size
        cols = msg.layout.dim[1].size
        new_trajectory = np.array(trajectory_data, dtype=float).reshape(rows, cols)

        # Only update trajectory if the new one is different
        if not np.array_equal(new_trajectory, self.base_trajectory):
            self.base_trajectory = new_trajectory
            # Reset progress variables
            self.base_waypoint = 0
            self.controller.base_current_target_waypoint=0
            self.base_target_reached = False
            temp_trajectory_base=np.vstack((self.base_current_pos,self.base_trajectory))
            self.base_trajectory_cubic,self.base_target_waypoints = self.controller.cubic_spline_interpolation(temp_trajectory_base,self.controller.base_max_vel, 0.01)
            self.get_logger().info('Got base trajectory in Control Node sub: "%s"' % self.base_trajectory_cubic)

    def base_pos_callback(self, msg):    
        # Recover position information as a 1D array
        self.base_current_pos = np.array([msg.x, msg.y, msg.z], dtype=float)
        self.get_logger().info('Got base pos in Control Node sub: "%s"' % self.base_current_pos)

    def arm_trajectory_callback(self, msg):
        # Recover trajectory information as a 2D Array
        trajectory_data = msg.data
        rows = msg.layout.dim[0].size
        cols = msg.layout.dim[1].size
        new_trajectory = np.array(trajectory_data, dtype=float).reshape(rows, cols)

        # Only update trajectory if the new one is different
        if not np.array_equal(new_trajectory, self.arm_trajectory):
            self.arm_trajectory = new_trajectory
            # Reset progress variables
            self.arm_waypoint = 0
            self.controller.arm_current_target_waypoint=0
            self.arm_target_reached = False
            self.new_trajectory=True
            # Get current position and trajectory
            current_arm_joint_pos, _ = self.controller.compute_forward_kinematics(self.controller.joints_list, self.arm_current_pos)
            temp_trajectory_arm=np.vstack((current_arm_joint_pos,self.arm_trajectory))
            self.arm_trajectory_cubic, self.arm_target_waypoints=self.controller.cubic_spline_interpolation(temp_trajectory_arm,self.controller.arm_max_vel, 0.01)
            self.get_logger().info('Got arm trajectory in Control Node arm sub: "%s"' % self.arm_trajectory_cubic)

    def arm_pos_callback(self, msg):
        # Recover position information as a 1D array
        self.arm_current_pos = np.array(msg.data, dtype=float)
        self.get_logger().info('Got arm pos in Control Node sub: "%s"' % self.arm_current_pos)

    # Get the new state from the environment
    def new_state_callback(self, msg):
        self.state_ = msg.data

    # Get the new state from the environment
    def new_state_callback(self, msg):
        self.state_ = msg.data

    def move_base(self):
        
        action, self.base_target_reached, self.base_waypoint = self.controller.run_panda_base(self.base_current_pos, self.base_trajectory_cubic,self.base_trajectory, self.base_target_waypoints, self.base_waypoint, self.base_target_reached)

        # Construct the cmd_vel msg for the base
        msg = Float64MultiArray()
        msg.data = action.astype(np.float64).tolist()

        # Publish cmd_vel msg 
        self.cmd_vel_publisher_.publish(msg)
        self.get_logger().info('Publishing base actions from control Node: "%s"' % msg.data)

    def move_arm(self):
        
        action, self.arm_target_reached, self.arm_waypoint = self.controller.run_panda_arm(self.arm_current_pos, self.arm_trajectory_cubic, self.arm_trajectory,self.arm_target_waypoints, self.arm_waypoint, self.arm_target_reached)
        
        # Construct the cmd_vel msg for the joints
        msg = Float64MultiArray()
        msg.data = action.astype(np.float64).tolist()
        self.new_trajectory = False
        # Publish cmd_vel msg
        self.cmd_vel_publisher_.publish(msg)
        self.get_logger().info('Publishing arm actions from control Node: "%s"' % msg.data)

    def pub_goal_reached(self):
        msg = Bool()
        msg.data = True
        self.goal_reached_.publish(msg)

    def pub_goal_reached(self):
        msg = Bool()
        msg.data = True
        self.goal_reached_.publish(msg)

    def ready_for_base(self):
        # Make sure all the required information is available
        return self.base_current_pos.size != 0 and self.base_trajectory.size != 0
    
    def ready_for_arm(self):
        # Make sure all the required information is available
        return self.arm_current_pos.size != 0 and self.arm_trajectory.size != 0 and self.new_trajectory 
    
def main(args=None):
    # start the control node
    try:
        rclpy.init(args=args)
        control_node = ControlNode()
        last_state=0
        while (rclpy.ok()):
            rclpy.spin_once(control_node)
            
            # Only start calculations once all the information is available
            if (control_node.ready_for_base() and control_node.state_ in {2, 5} and control_node.state_ != last_state):
                while(not control_node.base_target_reached): # Depending on state
                    control_node.move_base()
                    rclpy.spin_once(control_node)
                last_state = control_node.state_
                control_node.base_target_reached = False
                control_node.pub_goal_reached()


            if (control_node.ready_for_arm() and control_node.state_ in {1, 3, 4, 6} and control_node.state_ != last_state): # Depending on state
                while (not control_node.arm_target_reached):
                    control_node.move_arm()
                    rclpy.spin_once(control_node)
                last_state = control_node.state_
                control_node.arm_target_reached = False
                control_node.pub_goal_reached()

    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if control_node is not None:
            control_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

        control_node.get_logger().info("ControlNode has been shut down.")

if __name__ == "__main__":
    # Call Main Function
    main()