from std_msgs.msg import String
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
import numpy as np
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Point

class ControlNode(Node):
    def __init__(self):
        super().__init__('control')
        self.cmd_vel_publisher_ = self.create_publisher(Float64MultiArray, 'cmd_vel', 10)
        self.trajectory_subscription = self.create_subscription(
            Float64MultiArray,
            'trajectory',
            self.trajectory_callback,
            10)
        self.pos_subscription = self.create_subscription(
            Point,
            'base_pos',
            self.pos_callback,
            10)

        self.current_pos = np.empty(0)
        self.trajectory = np.empty(0)
        self.waypoint = 0
        self.target_reached = False
        # TODO: what other information or topics are needed?
        print("control Node has been created.")

    def trajectory_callback(self, msg):
        self.get_logger().info('Got in Control Node traj sub: "%s"' % msg.data)
        
        # Recover trajectory information as a 2D Array
        trajectory_data = msg.data
        rows = msg.layout.dim[0].size
        cols = msg.layout.dim[1].size
        new_trajectory = np.array(trajectory_data).reshape(rows, cols)

        # Only update trajectory if the new one is different
        if not np.array_equal(new_trajectory, self.trajectory):
            self.trajectory = new_trajectory
            # Reset progress variables
            self.waypoint = 0
            self.target_reached = False
        print(self.trajectory)

    def pos_callback(self, msg):
        self.get_logger().info('Got in Control Node pos sub: "%s"' % msg)
        
        # Recover position information as a 1D array
        self.current_pos = np.array([msg.x, msg.y, msg.z])

    def run_mobile_base(self):
        env_n = 12 # Number of actuators only the first 3 are used by the base
        action = np.zeros(env_n)
        
        # Set current position and target
        current_xyz = self.current_pos
        target_xyz = self.trajectory[self.waypoint]
        
        action_to_send = [0,0,0]

        if (np.linalg.norm(target_xyz - current_xyz) > 0.1): 
            # Controller to calculate velocities 
            # TODO: PID, cubic, quintic
            error_xyz = target_xyz - current_xyz
            desired_velocity_xyz = 1 * error_xyz
            action_to_send = desired_velocity_xyz
        elif (self.waypoint < target_xyz.shape[0]-1 ):
            # if the waypoint is reached change the target waypoint to the next
            self.waypoint += 1
        else:
            # If all waypoints have been reached stop the base and update the target reached variable
            action_to_send = [0,0,0]
            self.target_reached = True

        # Action is the variable with the target velocities for the robot
        action[:3] = action_to_send
        
        # Construct the cmd_vel msg
        msg = Float64MultiArray()
        msg.data = action.astype(np.float64).tolist()

        # Publish cmd_vel msg
        self.cmd_vel_publisher_.publish(msg)
        self.get_logger().info('Publishing action from control Node: "%s"' % msg.data)

    def ready(self):
        # Make sure all the required information is available
        # TODO: arm pose or anything else ? 
        return self.current_pos.size != 0 and self.trajectory.size != 0
    
    # TODO: arm control
    '''
    def revolute_transform(self, axis, angle):
        """Compute the rotation matrix for a revolute joint."""
        # Assuming 'axis' is normalized (e.g., [0, 0, 1] for z-axis)
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

        # Extract position from the final transformation matrix
        position = T[:3, 3]  # Top-right 3x1 part of T
        # print("Position from matrix:")
        # print(position)
        # vector=URDF.matrix_to_xyz_rpy(T)
        # print("Vector:")
        # print(vector)
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
'''

def main(args=None):
    # start the control node
    try:
        rclpy.init(args=args)
        control_node = ControlNode()

        while (rclpy.ok()):
            rclpy.spin_once(control_node)
            # Only start calculations once all the information is available
            if (control_node.ready()):
                while(not control_node.target_reached):
                    control_node.run_mobile_base()
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