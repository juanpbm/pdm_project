from std_msgs.msg import String
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
import numpy as np
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Point
import pickle
import os
from ament_index_python.packages import get_package_share_directory

class custom_URDF:
    def __init__(self,origin=np.identity(4),axis=np.zeros(3)):
      self.origin=origin
      self.axis=axis
      
    def create_list(self):
        self.joints_list=[]

    def add_joint(self,joint):
        self.joints_list.append(joint)

    def get_joints(self):
        return self.joints_list
    
class ControlNode(Node):
    def __init__(self):
        super().__init__('control')
        self.cmd_vel_publisher_ = self.create_publisher(Float64MultiArray, 'cmd_vel', 10)
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

        self.base_current_pos = np.empty(0)
        self.arm_current_pos = np.empty(0)
        self.base_trajectory = np.empty(0)
        self.arm_trajectory = np.empty(0)
        self.base_target_reached = False
        self.arm_target_reached = False
        self.base_waypoint = 0
        self.arm_waypoint = 0

        # Robot constants
        self.n_actions = 12 # Number of actuators only the first 3 are used by the base
        self.base_max_vel = 2.5 # limit of the robot TODO: get exact value
        self.arm_max_vel = 0.5

        # TODO: what other information or topics are needed?
        print("control Node has been created.")

    def base_trajectory_callback(self, msg):
        self.get_logger().info('Got in Control Node base traj sub: "%s"' % msg.data)
        
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
            self.base_target_reached = False
        print(self.base_trajectory)

    def base_pos_callback(self, msg):
        self.get_logger().info('Got in Control Node base pos sub: "%s"' % msg)
        
        # Recover position information as a 1D array
        self.base_current_pos = np.array([msg.x, msg.y, msg.z], dtype=float)

    def arm_trajectory_callback(self, msg):
        self.get_logger().info('Got in Control Node arm traj sub: "%s"' % msg.data)
        
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
            self.arm_target_reached = False
        print(self.arm_trajectory)

    def arm_pos_callback(self, msg):
        self.get_logger().info('Got in Control Node arm pos sub: "%s"' % msg.data)
        
        # Recover position information as a 1D array
        self.arm_current_pos = np.array(msg.data, dtype=float)
        print(self.arm_current_pos)

    def run_panda_base(self):

        action = np.zeros(self.n_actions, dtype=float)
        
        # Set current position and target
        current_xyz = self.base_current_pos
        target_xyz = self.base_trajectory[self.base_waypoint]
        
        action_to_send = [0.0,0.0,0.0]

        if (np.linalg.norm(target_xyz - current_xyz) > 0.1): 
            # Controller to calculate velocities 
            # TODO: PID, cubic, quintic
            error_xyz = target_xyz - current_xyz
            desired_velocity_xyz = 1 * error_xyz
            action_to_send = desired_velocity_xyz

            # Limit the actions to the max velocity of the robot
            # TODO: Check and match with arm
            for vel in action_to_send:
                if vel > self.base_max_vel:
                    vel = self.base_max_vel
        elif (self.base_waypoint < target_xyz.shape[0]-1 ):
            # If the target waypoint is reached update the target waypoint to the next index
            self.base_waypoint += 1
        else:
            # If all waypoints have been reached stop the base and update the target reached variable
            action_to_send = [0,0,0]
            self.base_target_reached = True # TODO: Publish this in case other pkgs need it to continue 


        # Action is the variable with the target velocities for the robot
        action[:3] = action_to_send
        
        # Construct the cmd_vel msg for the base
        msg = Float64MultiArray()
        msg.data = action.astype(np.float64).tolist()

        # Publish cmd_vel msg 
        self.cmd_vel_publisher_.publish(msg)
        self.get_logger().info('Publishing base action from control Node: "%s"' % msg.data)

    def ready_for_base(self):
        # Make sure all the required information is available
        return self.base_current_pos.size != 0 and self.base_trajectory.size != 0
    
    def ready_for_arm(self):
        # Make sure all the required information is available
        return self.arm_current_pos.size != 0 and self.arm_trajectory.size != 0
    
    # TODO: arm control
    arm_joints= [3,4,5,6,7,8,9,10,11]
    velocity_limit=2.5
    "This is the target xyz that the robot should receive to move the arm to that position"
    
    def revolute_transform(self, axis, angle):
        """Compute the rotation matrix for a revolute joint."""
        cosine= np.cos(angle) 
        sine=np.sin(angle)
        R = np.eye(3)
        if np.allclose(axis, [1, 0, 0]):  # Rotation about x-axis
            R = np.array([[1, 0, 0], [0, cosine, -sine], [0, sine, cosine]])
        elif np.allclose(axis, [0, 1, 0]):  # Rotation about y-axis
            R = np.array([[cosine, 0, sine], [0, 1, 0], [-sine, 0, cosine]])
        elif np.allclose(axis, [0, 0, 1]):  # Rotation about z-axis
            R = np.array([[cosine, -sine, 0], [sine, cosine, 0], [0, 0, 1]])

        transformation_matrix=np.block([[R, np.zeros((3, 1))], [np.zeros((1, 3)), 1]])
        return transformation_matrix

    def compute_forward_kinematics(self, robot_joints, joint_angles):
        """Compute the forward kinematics to get the end-effector position."""
        # Start with the identity matrix
        T = np.eye(4)  
        for x in range(len(robot_joints)-2):
            origin_transform = robot_joints[x].origin  # Base transformation from URDF
            axis = robot_joints[x].axis
            joint_transform = origin_transform @ self.revolute_transform(axis, joint_angles[x])

            T = T @ joint_transform  # Multiply transformations

        position = T[:3, 3]  
        return position

    def compute_jacobian(self, robot_joints, joint_angles, p_end):
        """Compute the Jacobian for the robot given joint angles."""
        T = np.eye(4)  # Start with the identity matrix
        J = []  # Initialize Jacobian matrix


        for x in range(len(robot_joints)-2):
            origin_transform = robot_joints[x].origin
            axis = robot_joints[x].axis

            T_joint = origin_transform @ self.revolute_transform(axis, joint_angles[x])

            T = T @ T_joint
            p_joint = T[:3, 3]  # Joint position
            z_axis = T[:3, 2]  # z-axis in world frame (rotation or translation axis)
            v = np.cross(z_axis, (p_end - p_joint))  # Linear velocity
            omega = z_axis  # Angular velocity

            J.append(np.hstack((v, omega)))
            

        return np.array(J).T  # Convert to numpy array and transpose
    
    def run_panda_arm(self):
        action = np.zeros(self.n_actions)
        
        # path to pikle files containing arm information
        file_path_axis = os.path.join(os.path.dirname(get_package_share_directory('control')), 'control', 'resource', 'joints_axis.pickle')
        file_path_origin = os.path.join(os.path.dirname(get_package_share_directory('control')), 'control', 'resource', 'joints_origin.pickle')

        # Load Arm information
        joints_class = custom_URDF()
        joints_class.create_list()
        with open(file_path_origin, 'rb') as file:
            # Load the joint origin data
            joints_origin_list_loaded = pickle.load(file)

        with open(file_path_axis, 'rb') as file:
            # Load the joints axis data
            joints_axis_list_loaded = pickle.load(file)

        # Combine joints data
        for x in range(len(joints_origin_list_loaded)):
            joint_temp = custom_URDF(joints_origin_list_loaded[x],joints_axis_list_loaded[x])
            joints_class.add_joint(joint_temp)

        joints_list = joints_class.get_joints()
    
        # Get current position and trajectory
        current_arm_pos = self.compute_forward_kinematics(joints_list, self.arm_current_pos)
        target_arm_pos = self.arm_trajectory[self.arm_waypoint]

        actions_to_send = np.zeros(self.n_actions)

        if (np.linalg.norm(target_arm_pos - current_arm_pos) > 0.01): 
            # Controller to calculate velocities 
            # TODO: PID, cubic, quinti
            error_arm_pos = target_arm_pos - current_arm_pos
            desired_velocity_arm = 1.0 * error_arm_pos  

            # Limit the actions to the max velocity of the robot
            if np.linalg.norm(desired_velocity_arm) > self.arm_max_vel:
                desired_velocity_arm = desired_velocity_arm / np.linalg.norm(desired_velocity_arm) * self.arm_max_vel
            desired_velocity_arm = np.hstack((desired_velocity_arm, np.zeros(3))) 
        
            # transform endpoint vel to joint vel
            J = self.compute_jacobian(joints_list, self.arm_current_pos, current_arm_pos)

            joint_velocities = np.linalg.pinv(J) @ desired_velocity_arm # Use pseudoinverse to solve

            actions_to_send = joint_velocities
        elif (self.base_waypoint < target_arm_pos.shape[0]-1 ):
            # If the target waypoint is reached update the target waypoint to the next index
            self.base_waypoint += 1 
        else:
            # If all waypoints have been reached stop the base and update the target reached variable
            actions_to_send = np.zeros(self.n_actions)
            self.arm_target_reached = True # TODO: Publish this in case other pkgs need it to continue
            
        # These are the instructions to move the arm
        for i in range(len(joints_list)-2):      
                action[i + 3] = actions_to_send[i]
        
        # Construct the cmd_vel msg for the joints
        msg = Float64MultiArray()
        msg.data = action.astype(np.float64).tolist()

        # Publish cmd_vel msg
        self.cmd_vel_publisher_.publish(msg)
        self.get_logger().info('Publishing arm action from control Node: "%s"' % msg.data)


def main(args=None):
    # start the control node
    try:
        rclpy.init(args=args)
        control_node = ControlNode()
        # control_node.run_mobile_reacher()
        while (rclpy.ok()):
            rclpy.spin_once(control_node)
            
            # Only start calculations once all the information is available
            if (control_node.ready_for_base()):
                while(not control_node.base_target_reached):
                    control_node.run_panda_base()
                    rclpy.spin_once(control_node)
            if (control_node.ready_for_arm() and control_node.base_target_reached):
                while (not control_node.arm_target_reached):
                    control_node.run_panda_arm()
                    rclpy.spin_once(control_node)

    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if control_node is not None:
            control_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

        print("ControlNode has been shut down.")

if __name__ == "__main__":
    # Call Main Function
    main()