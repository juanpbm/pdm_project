import numpy as np
import pickle
import os
from ament_index_python.packages import get_package_share_directory

# Custom class that mimics URDF file structure in order to pick up the data from pickle files
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

# Class where the kinematics and control logics are implemented   
class Controller:

    def __init__(self):
        # Robot constants
        self.n_actions = 12 # Number of actuators only the first 3 are used by the base
        self.base_max_vel = 1.2 # limit of the robot TODO: get exact value
        self.arm_max_vel = 0.7
        self.Kp=6
        self.arm_current_target_waypoint=0
        self.base_current_target_waypoint=0

        # Path to pickle files containing relevant arm information
        file_path_axis = os.path.join(os.path.dirname(get_package_share_directory('control')), 'control', 'resource', 'joints_axis.pickle')
        file_path_origin = os.path.join(os.path.dirname(get_package_share_directory('control')), 'control', 'resource', 'joints_origin.pickle')

        # Load Arm information into custom class
        joints_class = custom_URDF()
        joints_class.create_list()

        with open(file_path_origin, 'rb') as file:
            # Load the joint origin data (original position of the joints)
            joints_origin_list_loaded = pickle.load(file)

        with open(file_path_axis, 'rb') as file:
            # Load the joints axis data (axis in which joint affect)
            joints_axis_list_loaded = pickle.load(file)

        # Combine previously obtained joints data
        for x in range(len(joints_origin_list_loaded)):
            joint_temp = custom_URDF(joints_origin_list_loaded[x],joints_axis_list_loaded[x])
            joints_class.add_joint(joint_temp)

        # Transform joints data into list
        self.joints_list = joints_class.get_joints()

    def cubic_spline_interpolation(self,via_points,V_max, desired_dt):
        
        # Extract x, y, and z coordinates of via_points (current position + target waypoints)
        x = via_points[:, 0]
        y = via_points[:, 1]
        z = via_points[:, 2]
        n = len(x)

        # Calculate distances between via_points
        distances = np.sqrt(np.diff(x)**2 + np.diff(y)**2 + np.diff(z)**2)

        # Calculate minimum time intervals based on maximum velocity
        min_time_intervals = distances / V_max

        # Ensure cumulative (continous) time intervals
        times = np.zeros(n)
        times[1:] = np.cumsum(min_time_intervals)

        # Step lengths in time
        h = np.diff(times)

        # Solve for the coefficients of the cubic spline
        A = np.zeros((n, n))
        bx = np.zeros(n)
        by = np.zeros(n)
        bz = np.zeros(n)

        # Natural spline conditions
        A[0, 0] = 1
        A[-1, -1] = 1

        # Setting up the system of equations for x, y, and z coordinates
        for i in range(1, n - 1):
            A[i, i - 1] = h[i - 1]
            A[i, i] = 2 * (h[i - 1] + h[i])
            A[i, i + 1] = h[i]
            bx[i] = 3 * ((x[i + 1] - x[i]) / h[i] - (x[i] - x[i - 1]) / h[i - 1])
            by[i] = 3 * ((y[i + 1] - y[i]) / h[i] - (y[i] - y[i - 1]) / h[i - 1])
            bz[i] = 3 * ((z[i + 1] - z[i]) / h[i] - (z[i] - z[i - 1]) / h[i - 1])

        # Solve for the second derivatives
        second_derivatives_x = np.linalg.solve(A, bx)
        second_derivatives_y = np.linalg.solve(A, by)
        second_derivatives_z = np.linalg.solve(A, bz)

        # Calculate the spline coefficients for x, y, and z
        splines_x = []
        splines_y = []
        splines_z = []
        for i in range(n - 1):
            ax = (second_derivatives_x[i + 1] - second_derivatives_x[i]) / (6 * h[i])
            bx = second_derivatives_x[i] / 2
            cx = (x[i + 1] - x[i]) / h[i] - (h[i] * (2 * second_derivatives_x[i] + second_derivatives_x[i + 1])) / 6
            dx = x[i]

            ay = (second_derivatives_y[i + 1] - second_derivatives_y[i]) / (6 * h[i])
            by = second_derivatives_y[i] / 2
            cy = (y[i + 1] - y[i]) / h[i] - (h[i] * (2 * second_derivatives_y[i] + second_derivatives_y[i + 1])) / 6
            dy = y[i]

            az = (second_derivatives_z[i + 1] - second_derivatives_z[i]) / (6 * h[i])
            bz = second_derivatives_z[i] / 2
            cz = (z[i + 1] - z[i]) / h[i] - (h[i] * (2 * second_derivatives_z[i] + second_derivatives_z[i + 1])) / 6
            dz = z[i]

            splines_x.append((ax, bx, cx, dx))
            splines_y.append((ay, by, cy, dy))
            splines_z.append((az, bz, cz, dz))

        # Generate points along the spline
        total_time = times[-1]
        time_new = np.arange(0,total_time, desired_dt)
        x_new = np.zeros_like(time_new)
        y_new = np.zeros_like(time_new)
        z_new = np.zeros_like(time_new)

        coordinates_trajectory = []
        for j in range(len(time_new)):
            for i in range(n - 1):
                if times[i] <= time_new[j] <= times[i + 1]:
                    dt = time_new[j] - times[i]
                    ax, bx, cx, dx = splines_x[i]
                    ay, by, cy, dy = splines_y[i]
                    az, bz, cz, dz = splines_z[i]
                    x_new[j] = ax * dt**3 + bx * dt**2 + cx * dt + dx
                    y_new[j] = ay * dt**3 + by * dt**2 + cy * dt + dy
                    z_new[j] = az * dt**3 + bz * dt**2 + cz * dt + dz
                    coordinates_trajectory.append([x_new[j], y_new[j], z_new[j]])

        coordinates_trajectory=np.array(coordinates_trajectory)

        # Mark at which "time-step" (index) of the cubic spline it passes through one of the target waypoints
        # First via_point is skipped as it is the current position, not a target waypoint
        target_waypoints=np.zeros(len(via_points)-1)
        b=0
        
        if (len(target_waypoints)>1):
            for a in range(len(coordinates_trajectory)):
                if(b<(len(via_points)-1)):
                    if (np.linalg.norm(coordinates_trajectory[a]-via_points[b+1])<0.01):
                        target_waypoints[b]=a
                        b+=1
            target_waypoints=np.array(target_waypoints)
        else: 
            target_waypoints=np.array([len(coordinates_trajectory)-1])
        return coordinates_trajectory, target_waypoints


    def run_panda_base(self, current_xyz, base_trajectory_cubic, base_target_trajectory, base_target_waypoints, base_waypoint, base_target_reached):
        
        # Set all velocities to zero
        action = np.zeros(self.n_actions, dtype=float)
        
        # Initialize the action velocities that will be sent
        action_to_send = [ 0.0, 0.0, 0.0]

        # Calculate the error between the reference cubic trajectory and the current base position
        # - Only x and y are measured as with the movement of the holonomic base did not require to get the z error
        # - If changing the orientation was needed, [:2] would be removed
        error_xyz = base_trajectory_cubic[base_waypoint][:2] - current_xyz[:2]
    
        # Proportional controller is applied
        desired_velocity_xyz = self.Kp*error_xyz 
        action_to_send = desired_velocity_xyz

        # Waypoints incremenation logic
         # If the current waypoint is lower that the marked target waypoint, it increases
        if(base_target_waypoints[self.base_current_target_waypoint]>base_waypoint):
            base_waypoint+=1 
         # If the current waypoint matches where the target waypoint should be, check if the actual position matches the 
         # target position. If it does, increases the target waypoint and let's the robot keep moving, if not just wait until
         # the error is low
        elif((self.base_current_target_waypoint<(len(base_target_trajectory)-1) ) and 
                (np.linalg.norm(base_target_trajectory[self.base_current_target_waypoint][:2]-current_xyz[:2])<0.1)):
            self.base_current_target_waypoint+=1
        
        # For the final target waypoint (final point), have a mopre strict error to have it near the goal 
        elif((self.base_current_target_waypoint==(len(base_target_trajectory)-1)) and
                (abs(base_target_trajectory[self.base_current_target_waypoint][0]-current_xyz[0])<0.03) and
                (abs(base_target_trajectory[self.base_current_target_waypoint][1]-current_xyz[1])<0.03)):
            # If all waypoints have been reached stop the base and update the target reached variable
            action_to_send = [0,0,0]
            base_target_reached = True  

        # Check if the velocities exceed the maximum velocity  
        for i in range(len(action_to_send)):
                if action_to_send[i] > self.base_max_vel:
                    action_to_send[i] = self.base_max_vel
                
        # Change the desired action velocities
        action[:2] = action_to_send[:2]
      
        return action, base_target_reached, base_waypoint
    
    def revolute_transform(self, axis, angle):

        # Compute the rotation matrix for a revolute joint.
        cosine= np.cos(angle) 
        sine=np.sin(angle)

        # Initialize the rotation matrix
        R = np.eye(3)

        # Rotation around x-axis
        if np.allclose(axis, [1, 0, 0]):  
            R = np.array([[1, 0, 0], [0, cosine, -sine], [0, sine, cosine]])

        # Rotation around y-axis
        elif np.allclose(axis, [0, 1, 0]):  
            R = np.array([[cosine, 0, sine], [0, 1, 0], [-sine, 0, cosine]])

        # Rotation around z-axis
        elif np.allclose(axis, [0, 0, 1]):  
            R = np.array([[cosine, -sine, 0], [sine, cosine, 0], [0, 0, 1]])

        # Combine the rotation matrix into one transformation matrix
        transformation_matrix=np.block([[R, np.zeros((3, 1))], [np.zeros((1, 3)), 1]])
        return transformation_matrix

    def compute_forward_kinematics(self, robot_joints, joint_angles):
        # Forward kinematics to get the end-effector position
        # Intialize the identity matrix
        T = np.eye(4)  
        for x in range(len(robot_joints)-2):
            # Get the base transformation from the list
            origin_transform = robot_joints[x].origin 
            # Get the axis in which the joint moves from the list 
            axis = robot_joints[x].axis
            # Get transformation matrix
            joint_transform = origin_transform @ self.revolute_transform(axis, joint_angles[x])
            # Multiply transformations
            T = T @ joint_transform  

        # End-effector position
        position = T[:3, 3] 
        # Get the orientation from the trasnformation matrix
        orientation_euler = self.matrix_to_euler(T[:3, :3])

        return position, orientation_euler 

    def compute_jacobian(self, robot_joints, joint_angles, p_end):
        # Compute the Jacobian for the robot given joint angles.
        # Initialize Transformation adn Jacobian matrix 
        T = np.eye(4)  
        J = []  

        # Compute the Jacobian for all arm joints except the grippers
        for x in range(len(robot_joints)-2):
            origin_transform = robot_joints[x].origin
            axis = robot_joints[x].axis
            T_joint = origin_transform @ self.revolute_transform(axis, joint_angles[x])
            T = T @ T_joint
            # Joint position
            p_joint = T[:3, 3]  
            # z-axis in world frame (rotation or translation axis, in these joint all rotations)
            z_axis = T[:3, 2]  
            # Get the linear velocity
            v = np.cross(z_axis, (p_end - p_joint)) 
            # Get the angular velocity 
            omega = z_axis  
            # Combine velocities into the Jacobian matrix
            J.append(np.hstack((v, omega)))
            

        return np.array(J).T  # Convert to numpy array and transpose
    
    def calc_rot_error(self, reference, actual):

        # Get only the orientation part of the matrices 
        Actual_Matrix = actual[:3, :3]
        Reference_Matrix = reference[:3, :3]
        # Compute the orientation error between the two matrices
        err = 0.5 *(np.cross(Actual_Matrix[:,0],Reference_Matrix[:,0])
                 +np.cross(Actual_Matrix[:,1],Reference_Matrix[:,1])
                 +np.cross(Actual_Matrix[:,2],Reference_Matrix[:,2]))
      
        return err

    def euler_to_matrix(self, euler): 
        # Extract the roll, pitch and yaw angles
        roll, pitch, yaw = euler 
        # Compute the roll angle into rotation matrix
        R_x = np.array([[1, 0, 0],
                        [0, np.cos(roll), -np.sin(roll)], 
                        [0, np.sin(roll), np.cos(roll)]]) 
        # Compute the pitch angle into rotation matrix
        R_y = np.array([[np.cos(pitch), 0, np.sin(pitch)], 
                        [0, 1, 0], 
                        [-np.sin(pitch), 0, np.cos(pitch)]]) 
        # Compute the yaw angle into rotation matrix
        R_z = np.array([[np.cos(yaw), -np.sin(yaw), 0], 
                        [np.sin(yaw), np.cos(yaw), 0], 
                        [0, 0, 1]]) 
        # Get the combined rotation matrix
        R = np.dot(R_z, np.dot(R_y, R_x)) 
        return R

    def matrix_to_euler(self, matrix): 
        # Get the roll angle from the matrix
        roll = np.arctan2(matrix[2, 1], matrix[2, 2]) 
        # Get the pitch angle from the matrix
        pitch = np.arctan2(-matrix[2, 0], np.sqrt(matrix[2, 1]**2 + matrix[2, 2]**2)) 
        # Get the yaw angle from the matrix
        yaw = np.arctan2(matrix[1, 0], matrix[0, 0]) 

        return np.array([roll, pitch, yaw])

    def pseudo_jacobian(self, J, desired_velocity, damping_factor=0.01): 
        # First compute the multiplication to get J @ transpose(J)
        JJT = J @ J.T 
        # Then get the damping matrix
        damping_matrix = damping_factor * np.eye(JJT.shape[0]) 
        # Compute the pseudo_inverse
        pseudo_inverse = J.T @ np.linalg.inv(JJT + damping_matrix) 
        # Do the final calculation on the return 
        return pseudo_inverse @ desired_velocity

    def run_panda_arm(self, arm_current_pos, arm_trajectory_cubic, arm_target_trajectory, arm_target_waypoints, arm_waypoint, arm_target_reached):
        
        # Set all velocities to zero
        action = np.zeros(self.n_actions)
    
        # Get current position and current orientation of the arm
        current_arm_joint_pos, current_orientation = self.compute_forward_kinematics(self.joints_list, arm_current_pos)

        target_orientation = np.array([ 3.14, -0.37,  0.  ]) # Euler Angles, fixed position  

        # Initialize the action velocities that will be sent
        actions_to_send = np.zeros(self.n_actions)

        # Position error
         # Calculate the error between the reference cubic trajectory and the current arm position
         # - [:3] Just as safe measure of only getting the x,y,z positions 
        error_xyz = arm_trajectory_cubic[arm_waypoint] - current_arm_joint_pos[:3]    
      
         # Proportional controller 
        desired_velocity_xyz = self.Kp*error_xyz 
        actions_to_send = desired_velocity_xyz

        # Orientation error 
         # Convert orientations into matrices
        current_orientation_matrix = self.euler_to_matrix(current_orientation) 
        target_orientation_matrix = self.euler_to_matrix(target_orientation) 
        
         # Calculate the orientation error between the two matrices and proportional controller
        orientation_error=self.calc_rot_error(target_orientation_matrix, current_orientation_matrix)
        desired_velocity_orientation = self.Kp * orientation_error

        # Combine both desired velocities into one matrix
        desired_velocity = np.hstack((desired_velocity_xyz, desired_velocity_orientation)) 
        
        # Compute the Jacobian and the pseudo jacobian (singularity avoidance)
        J= self.compute_jacobian(self.joints_list, arm_current_pos, current_arm_joint_pos)
        joint_velocities = self.pseudo_jacobian(J, desired_velocity)

        # Assign the velocities
        actions_to_send=joint_velocities

        # Waypoints incremenation logic
         # If the current waypoint is lower that the marked target waypoint, it increases
        if(arm_target_waypoints[self.arm_current_target_waypoint]>arm_waypoint):
            arm_waypoint+=1 
         # If the current waypoint matches where the target waypoint should be, check if the actual position matches the 
         # target position. If it does, increases the target waypoint and let's the robot keep moving, if not just wait until
         # the error is low    
        elif((self.arm_current_target_waypoint<len(arm_target_trajectory)-1 ) and 
                (np.linalg.norm(arm_target_trajectory[self.arm_current_target_waypoint]-current_arm_joint_pos)<0.01)):
            self.arm_current_target_waypoint+=1

         # For the final target waypoint (final point), have a mopre strict error to have it near the goal 
        elif((self.arm_current_target_waypoint==len(arm_target_trajectory)-1 ) and 
                (np.linalg.norm(arm_target_trajectory[self.arm_current_target_waypoint]-current_arm_joint_pos)<0.01)):
            # If all waypoints have been reached stop the base and update the target reached variable
            actions_to_send = np.zeros(self.n_actions)
            arm_target_reached = True 
            
               
       # Check if the velocities exceed the maximum velocity and assign the velocity to the corresponding index of the action
       # - 3 first position are the base velocities
        for i in range(len(self.joints_list)-2):  
                if(actions_to_send[i]>self.arm_max_vel):  
                    action[i + 3] = self.arm_max_vel
                else:
                    action[i + 3] = actions_to_send[i]

        return action, arm_target_reached, arm_waypoint
