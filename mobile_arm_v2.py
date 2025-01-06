import numpy as np
from urdfenvs.robots.generic_urdf import GenericUrdfReacher
from urdfenvs.urdf_common.urdf_env import UrdfEnv
import pickle

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

arm_joints= [3,4,5,6,7,8,9,10,11]
velocity_limit=2.5
"This is the target xyz that the robot should receive to move the arm to that position"
target_xyz = np.array([0.6, 1, 0.5])
target_orientation = np.array([1, 0, 0]) # Euler Angles

###############    NEW  ########################
def calc_rot_error(reference, actual):

        Actual_Matrix=actual[:3, :3]
        Reference_Matrix=reference[:3, :3]
        err=0.5 *(np.cross(Actual_Matrix[:,0],Reference_Matrix[:,0])
                 +np.cross(Actual_Matrix[:,1],Reference_Matrix[:,1])
                 +np.cross(Actual_Matrix[:,2],Reference_Matrix[:,2]))
      
        return err


def euler_to_matrix(euler): 
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

def matrix_to_euler(matrix): 
    roll = np.arctan2(matrix[2, 1], matrix[2, 2]) 
    pitch = np.arctan2(-matrix[2, 0], np.sqrt(matrix[2, 1]**2 + matrix[2, 2]**2)) 
    yaw = np.arctan2(matrix[1, 0], matrix[0, 0]) 
    return np.array([roll, pitch, yaw])

def pseudo_jacobian(J, desired_velocity, damping_factor=0.1): 
    JJT = J @ J.T 
    damping_matrix = damping_factor**2 * np.eye(JJT.shape[0]) 
    pseudo_inverse = J.T @ np.linalg.inv(JJT + damping_matrix) 

    return pseudo_inverse @ desired_velocity

###############   END OF NEW  ########################     

def revolute_transform(axis, angle):
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


def compute_forward_kinematics(robot_joints, joint_angles):
    """Compute the forward kinematics to get the end-effector position."""
    # Start with the identity matrix
    T = np.eye(4)  
    for x in range(len(robot_joints)-2):
        origin_transform = robot_joints[x].origin  # Base transformation from URDF
        axis = robot_joints[x].axis
        joint_transform = origin_transform @ revolute_transform(axis, joint_angles[x])

        T = T @ joint_transform  # Multiply transformations

    position = T[:3, 3]  

    ###############   NEW  ########################   
    orientation_euler = matrix_to_euler(T[:3, :3])
    ###############   END OF NEW  (CAREFUL, NEW PARAMETER TO RETURN) ########################  

    return position, orientation_euler


def compute_jacobian(robot_joints, joint_angles, p_end):
    """Compute the Jacobian for the robot given joint angles."""
    T = np.eye(4)  # Start with the identity matrix
    J = []  # Initialize Jacobian matrix


    for x in range(len(robot_joints)-2):
        origin_transform = robot_joints[x].origin
        axis = robot_joints[x].axis

        T_joint = origin_transform @ revolute_transform(axis, joint_angles[x])

        T = T @ T_joint
        p_joint = T[:3, 3]  # Joint position
        z_axis = T[:3, 2]  # z-axis in world frame (rotation or translation axis)
        v = np.cross(z_axis, (p_end - p_joint))  # Linear velocity
        omega = z_axis  # Angular velocity

        J.append(np.hstack((v, omega)))
        

    return np.array(J).T  # Convert to numpy array and transpose



def run_mobile_reacher(n_steps=10000, render=False, goal=True, obstacles=True):
    robots = [
        GenericUrdfReacher(urdf="mobilePanda_with_gripper.urdf", mode="vel"),
    ]
    env: UrdfEnv = UrdfEnv(
        dt=0.01, robots=robots, render=render, num_sub_steps=200,
    )
    action = np.zeros(env.n())
    ob = env.reset()
    print(f"Initial observation : {ob}")
   
    joints_class = custom_URDF()
    joints_class.create_list()
    
    with open('joints_origin.pickle', 'rb') as file:
        # Load the joinst data
        joints_origin_list_loaded = pickle.load(file)

    with open('joints_axis.pickle', 'rb') as file:
        # Load the joinst data
        joints_axis_list_loaded = pickle.load(file)

    for x in range(len(joints_origin_list_loaded)):
        joint_temp = custom_URDF(joints_origin_list_loaded[x],joints_axis_list_loaded[x])
        joints_class.add_joint(joint_temp)

    joints_list= joints_class.get_joints()

    ob, *_ = env.step(action) 
    
    ###############   NEW PART ########################  
    current_xyz, current_orientation =compute_forward_kinematics(joints_list, np.round(ob['robot_0']['joint_state']['position'][3:-2],4))
    Kp=1.0
    ###############   END OF NEW  PART ########################  

    print(np.round(current_xyz))
    history = []
    actions_to_send=np.zeros(env.n())
    max_velocity = 0.5
    for i in range(n_steps):
        if (np.linalg.norm(target_xyz - current_xyz) > 0.01): 
           
            error_xyz = target_xyz - current_xyz
            desired_velocity_xyz = Kp * error_xyz  

            if np.linalg.norm(desired_velocity_xyz) > max_velocity:
                desired_velocity_xyz = desired_velocity_xyz / np.linalg.norm(desired_velocity_xyz) * max_velocity
            
            ###############   NEW   ######################## 

            # Orientation error 
            current_orientation_matrix = euler_to_matrix(current_orientation) 
            target_orientation_matrix = euler_to_matrix(target_orientation) 
          
            orientation_error=calc_rot_error(target_orientation_matrix, current_orientation_matrix)
            desired_velocity_orientation = Kp * orientation_error
            
            ###############   END OF NEW  PART ######################## 
            
            desired_velocity = np.hstack((desired_velocity_xyz, desired_velocity_orientation)) 
           
            J= compute_jacobian(joints_list, np.round(ob['robot_0']['joint_state']['position'][3:-2],4), current_xyz)
            joint_velocities = pseudo_jacobian(J, desired_velocity)

            actions_to_send=joint_velocities
            
        else:

            actions_to_send=np.zeros(env.n())
            
        "These are the instructions to move the arm"
        for x in range(len(joints_list)-2):      
                action[x+3]=actions_to_send[x]
        "This what moves the arm"
        ob, *_ = env.step(action) 
        current_xyz, current_orientation  = compute_forward_kinematics(joints_list, np.round(ob['robot_0']['joint_state']['position'][3:-2],4))   
        history.append(ob)
    env.close()

    

    return history


if __name__ == "__main__":
    run_mobile_reacher(render=True)