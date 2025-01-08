import numpy as np
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
    
class Controller:

    def __init__(self):
        # Robot constants
        self.n_actions = 12 # Number of actuators only the first 3 are used by the base
        self.base_max_vel = 0.5 # limit of the robot TODO: get exact value
        self.arm_max_vel = 0.5
        self.Kp=5
        self.Kd=2
        self.e_prev=[0,0]
        self.time_prev=0

    def trajectory_planning(self, via_points,V_max, desired_dt):
        # Extract x and y coordinates of via-points
        x = via_points[:, 0]
        y = via_points[:, 1]
        n = len(x)

        # Calculate distances between via-points
        distances = np.sqrt(np.diff(x)**2 + np.diff(y)**2)

        # Calculate minimum time intervals based on maximum velocity
        min_time_intervals = distances / V_max

        # Ensure cumulative time intervals
        times = np.zeros(n)
        times[1:] = np.cumsum(min_time_intervals)

        # Step lengths in time
        h = np.diff(times)

        # Solve for the coefficients of the cubic spline
        A = np.zeros((n, n))
        bx = np.zeros(n)
        by = np.zeros(n)

        # Natural spline conditions
        A[0, 0] = 1
        A[-1, -1] = 1

        # Setting up the system of equations for x and y coordinates
        for i in range(1, n-1):
            A[i, i-1] = h[i-1]
            A[i, i] = 2 * (h[i-1] + h[i])
            A[i, i+1] = h[i]
            bx[i] = 3 * ((x[i+1] - x[i]) / h[i] - (x[i] - x[i-1]) / h[i-1])
            by[i] = 3 * ((y[i+1] - y[i]) / h[i] - (y[i] - y[i-1]) / h[i-1])

        # Solve for the second derivatives
        second_derivatives_x = np.linalg.solve(A, bx)
        second_derivatives_y = np.linalg.solve(A, by)

        # Calculate the spline coefficients for x and y
        splines_x = []
        splines_y = []
        for i in range(n-1):
            ax = (second_derivatives_x[i+1] - second_derivatives_x[i]) / (6 * h[i])
            bx = second_derivatives_x[i] / 2
            cx = (x[i+1] - x[i]) / h[i] - (h[i] * (2 * second_derivatives_x[i] + second_derivatives_x[i+1])) / 6
            dx = x[i]

            ay = (second_derivatives_y[i+1] - second_derivatives_y[i]) / (6 * h[i])
            by = second_derivatives_y[i] / 2
            cy = (y[i+1] - y[i]) / h[i] - (h[i] * (2 * second_derivatives_y[i] + second_derivatives_y[i+1])) / 6
            dy = y[i]

            splines_x.append((ax, bx, cx, dx))
            splines_y.append((ay, by, cy, dy))

        # Generate points along the spline
        total_time = times[-1] 
        time_new =np.arange(0,total_time, desired_dt)
        x_new = np.zeros_like(time_new)
        y_new = np.zeros_like(time_new)

        coordinates_trajectory=[]
        for j in range(len(time_new)):
            for i in range(n-1):
                if times[i] <= time_new[j] <= times[i+1]:
                    dt = time_new[j] - times[i]
                    ax, bx, cx, dx = splines_x[i]
                    ay, by, cy, dy = splines_y[i]
                    x_new[j] = ax * dt**3 + bx * dt**2 + cx * dt + dx
                    y_new[j] = ay * dt**3 + by * dt**2 + cy * dt + dy
                    coordinates_trajectory.append([x_new[j],y_new[j]])

        return np.array(coordinates_trajectory)


    def trajectory_planning(self, via_points,V_max, desired_dt):
        # Extract x and y coordinates of via-points
        x = via_points[:, 0]
        y = via_points[:, 1]
        n = len(x)

        # Calculate distances between via-points
        distances = np.sqrt(np.diff(x)**2 + np.diff(y)**2)

        # Calculate minimum time intervals based on maximum velocity
        min_time_intervals = distances / V_max

        # Ensure cumulative time intervals
        times = np.zeros(n)
        times[1:] = np.cumsum(min_time_intervals)

        # Step lengths in time
        h = np.diff(times)

        # Solve for the coefficients of the cubic spline
        A = np.zeros((n, n))
        bx = np.zeros(n)
        by = np.zeros(n)

        # Natural spline conditions
        A[0, 0] = 1
        A[-1, -1] = 1

        # Setting up the system of equations for x and y coordinates
        for i in range(1, n-1):
            A[i, i-1] = h[i-1]
            A[i, i] = 2 * (h[i-1] + h[i])
            A[i, i+1] = h[i]
            bx[i] = 3 * ((x[i+1] - x[i]) / h[i] - (x[i] - x[i-1]) / h[i-1])
            by[i] = 3 * ((y[i+1] - y[i]) / h[i] - (y[i] - y[i-1]) / h[i-1])

        # Solve for the second derivatives
        second_derivatives_x = np.linalg.solve(A, bx)
        second_derivatives_y = np.linalg.solve(A, by)

        # Calculate the spline coefficients for x and y
        splines_x = []
        splines_y = []
        for i in range(n-1):
            ax = (second_derivatives_x[i+1] - second_derivatives_x[i]) / (6 * h[i])
            bx = second_derivatives_x[i] / 2
            cx = (x[i+1] - x[i]) / h[i] - (h[i] * (2 * second_derivatives_x[i] + second_derivatives_x[i+1])) / 6
            dx = x[i]

            ay = (second_derivatives_y[i+1] - second_derivatives_y[i]) / (6 * h[i])
            by = second_derivatives_y[i] / 2
            cy = (y[i+1] - y[i]) / h[i] - (h[i] * (2 * second_derivatives_y[i] + second_derivatives_y[i+1])) / 6
            dy = y[i]

            splines_x.append((ax, bx, cx, dx))
            splines_y.append((ay, by, cy, dy))

        # Generate points along the spline
        total_time = times[-1] 
        time_new =np.arange(0,total_time, desired_dt)
        x_new = np.zeros_like(time_new)
        y_new = np.zeros_like(time_new)

        coordinates_trajectory=[]
        for j in range(len(time_new)):
            for i in range(n-1):
                if times[i] <= time_new[j] <= times[i+1]:
                    dt = time_new[j] - times[i]
                    ax, bx, cx, dx = splines_x[i]
                    ay, by, cy, dy = splines_y[i]
                    x_new[j] = ax * dt**3 + bx * dt**2 + cx * dt + dx
                    y_new[j] = ay * dt**3 + by * dt**2 + cy * dt + dy
                    coordinates_trajectory.append([x_new[j],y_new[j]])

        return np.array(coordinates_trajectory)

    def run_panda_base(self, current_xyz, base_trajectory, base_waypoint, base_target_reached):

        action = np.zeros(self.n_actions, dtype=float)
        
        # Set current position and target
        
        # action_to_send = [0.0,0.0,0.0]
        action_to_send = [0.0,0.0]


        # V_max = 10 # Maximum velocity (units per time unit)
        # dt=0.01
        # trajectory_xyz=np.vstack((current_xyz,target_xyz))
        # coordinates_trajectory=self.trajectory_planning(trajectory_xyz,V_max, dt)
       
  
        # if (len(coordinates_trajectory)>i): 
           
        #     error_xyz = coordinates_trajectory[i] - current_xyz[:2]
        #     Derivative_error=Kd*(error_xyz - e_prev)/(i+1 - time_prev)
        #     desired_velocity_xyz = Kp*error_xyz + Derivative_error
        #     action_to_send = desired_velocity_xyz

        #     e_prev = error_xyz
        #     time_prev = i
            
        # elif(len(coordinates_trajectory)+100>i):   
        #     error_xyz = coordinates_trajectory[-1] - current_xyz[:2]
        #     Derivative_error=Kd*(error_xyz - e_prev)/(i+1 - time_prev)
        #     desired_velocity_xyz = Kp*error_xyz + Derivative_error
        #     action_to_send = desired_velocity_xyz

        #     e_prev = error_xyz
        #     time_prev = i
        
        # if (np.linalg.norm(target_xyz - current_xyz) > 0.1): 
        #     # Controller to calculate velocities 
        #     # TODO: PID, cubic, quintic
        #     error_xyz = target_xyz - current_xyz
        #     desired_velocity_xyz = 1 * error_xyz

        # V_max = 10 # Maximum velocity (units per time unit)
        # dt=0.01
        # trajectory_xyz=np.vstack((current_xyz,target_xyz))
        # coordinates_trajectory=self.trajectory_planning(trajectory_xyz,V_max, dt)
        
  
        if (len(base_trajectory)>base_waypoint): 
           
            error_xyz = base_trajectory[base_waypoint] - current_xyz[:2]
            Derivative_error=self.Kd*(error_xyz - self.e_prev)/(base_waypoint+1 - self.time_prev)
            desired_velocity_xyz = self.Kp*error_xyz + Derivative_error

            action_to_send = desired_velocity_xyz

            self.e_prev = error_xyz
            self.time_prev = base_waypoint
            base_waypoint += 1
            
        elif(len(base_trajectory)+100>base_waypoint):   
            error_xyz = base_trajectory[-1] - current_xyz[:2]
            Derivative_error=self.Kd*(error_xyz - self.e_prev)/(base_waypoint+1 - self.time_prev)
            desired_velocity_xyz = self.Kp*error_xyz + Derivative_error
            action_to_send = desired_velocity_xyz

            self.e_prev = error_xyz
            self.time_prev = base_waypoint
            base_waypoint += 1
        else:
            # If all waypoints have been reached stop the base and update the target reached variable
            action_to_send = [0,0]
            base_target_reached = True # TODO: Publish this in case other pkgs need it to continue 

        # if (np.linalg.norm(target_xyz - current_xyz) > 0.1): 
        #     # Controller to calculate velocities 
        #     # TODO: PID, cubic, quintic
        #     error_xyz = target_xyz - current_xyz
        #     desired_velocity_xyz = 1 * error_xyz
        #     action_to_send = desired_velocity_xyz

        #     # Limit the actions to the max velocity of the robot
        #     # TODO: Check and match with arm
        #     for vel in action_to_send:
        #         if vel > self.base_max_vel:
        #             vel = self.base_max_vel
        # elif (base_waypoint < base_trajectory.shape[0] - 1):
        #     # If the target waypoint is reached update the target waypoint to the next index
        #     base_waypoint += 1
        # else:
        #     # If all waypoints have been reached stop the base and update the target reached variable
        #     action_to_send = [0,0,0]
        #     base_target_reached = True # TODO: Publish this in case other pkgs need it to continue 

        for vel in action_to_send:
                if vel > self.base_max_vel:
                    vel = self.base_max_vel
        # Action is the variable with the target velocities for the robot
        # action[:3] = action_to_send
        action[:2] = action_to_send
        return action, base_target_reached, base_waypoint
    
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
        orientation_euler = self.matrix_to_euler(T[:3, :3])

        return position, orientation_euler 

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
    
    def calc_rot_error(self, reference, actual):

        Actual_Matrix=actual[:3, :3]
        Reference_Matrix=reference[:3, :3]
        err=0.5 *(np.cross(Actual_Matrix[:,0],Reference_Matrix[:,0])
                 +np.cross(Actual_Matrix[:,1],Reference_Matrix[:,1])
                 +np.cross(Actual_Matrix[:,2],Reference_Matrix[:,2]))
      
        return err

    def euler_to_matrix(self, euler): 
        roll, pitch, yaw = euler 
        R_x = np.array([[1, 0, 0],
                        [0, np.cos(roll), -np.sin(roll)], 
                        [0, np.sin(roll), np.cos(roll)]]) 
        R_y = np.array([[np.cos(pitch), 0, np.sin(pitch)], 
                        [0, 1, 0], 
                        [-np.sin(pitch), 0, np.cos(pitch)]]) 
        R_z = np.array([[np.cos(yaw), -np.sin(yaw), 0], 
                        [np.sin(yaw), np.cos(yaw), 0], 
                        [0, 0, 1]]) 
        R = np.dot(R_z, np.dot(R_y, R_x)) 
        return R

    def matrix_to_euler(self, matrix): 
        roll = np.arctan2(matrix[2, 1], matrix[2, 2]) 
        pitch = np.arctan2(-matrix[2, 0], np.sqrt(matrix[2, 1]**2 + matrix[2, 2]**2)) 
        yaw = np.arctan2(matrix[1, 0], matrix[0, 0]) 
        return np.array([roll, pitch, yaw])

    def pseudo_jacobian(self, J, desired_velocity, damping_factor=0.1): 
        JJT = J @ J.T 
        damping_matrix = damping_factor**2 * np.eye(JJT.shape[0]) 
        pseudo_inverse = J.T @ np.linalg.inv(JJT + damping_matrix) 

        return pseudo_inverse @ desired_velocity

    def run_panda_arm(self, arm_current_pos, arm_trajectory, arm_waypoint, arm_target_reached):
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
        current_arm_joint_pos, current_orientation = self.compute_forward_kinematics(joints_list, arm_current_pos)
        Kp=1.0
        target_arm_pos = arm_trajectory[arm_waypoint]
        
        target_orientation = np.array([1, 0, 0]) # Euler Angles TODO: where would this come from. 
        actions_to_send = np.zeros(self.n_actions)

        if (np.linalg.norm(target_arm_pos - current_arm_joint_pos) > 0.01): 
            # Controller to calculate velocities 
            # TODO: PID, cubic, quinti
            error_arm_pos = target_arm_pos - current_arm_joint_pos
            desired_velocity_arm = 1.0 * error_arm_pos  

            # Limit the actions to the max velocity of the robot
            if np.linalg.norm(desired_velocity_arm) > self.arm_max_vel:
                desired_velocity_arm = desired_velocity_arm / np.linalg.norm(desired_velocity_arm) * self.arm_max_vel

            # desired_velocity_arm = np.hstack((desired_velocity_arm, np.zeros(3))) 
        
            # Orientation error 
            current_orientation_matrix = self.euler_to_matrix(current_orientation) 
            target_orientation_matrix = self.euler_to_matrix(target_orientation) 
          
            orientation_error = self.calc_rot_error(target_orientation_matrix, current_orientation_matrix)
            desired_velocity_orientation = Kp * orientation_error

            desired_velocity = np.hstack((desired_velocity_arm, desired_velocity_orientation)) 
           
            # transform endpoint vel to joint vel
            J = self.compute_jacobian(joints_list, arm_current_pos, current_arm_joint_pos)

            joint_velocities = self.pseudo_jacobian(J, desired_velocity)

            actions_to_send = joint_velocities
        elif (arm_waypoint < arm_trajectory.shape[0]-1 ):
            # If the target waypoint is reached update the target waypoint to the next index
            self.arm_waypoint += 1 
        else:
            # If all waypoints have been reached stop the base and update the target reached variable
            actions_to_send = np.zeros(self.n_actions)
            arm_target_reached = True # TODO: Publish this in case other pkgs need it to continue
            
        # These are the instructions to move the arm
        for i in range(len(joints_list)-2):      
                action[i + 3] = actions_to_send[i]

        return action, arm_target_reached, arm_waypoint