import numpy as np
from urdfpy import URDF
from urdfenvs.robots.generic_urdf import GenericUrdfReacher
from urdfenvs.urdf_common.urdf_env import UrdfEnv
from tqdm import tqdm

arm_joints= [3,4,5,6,7,8,9,10,11]
velocity_limit=2.5


def revolute_transform(axis, angle):
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


def compute_forward_kinematics(robot_joints, joint_angles):
    """Compute the forward kinematics to get the end-effector position."""
    # Start with the identity matrix
    T = np.eye(4)  
    for x in range(len(robot_joints)-2):
        origin_transform = robot_joints[x].origin  # Base transformation from URDF
        axis = robot_joints[x].axis
        joint_transform = origin_transform @ revolute_transform(axis, joint_angles[x])

        T = T @ joint_transform  # Multiply transformations

    # Extract position from the final transformation matrix
    position = T[:3, 3]  # Top-right 3x1 part of T
    # print("Position from matrix:")
    # print(position)
    # vector=URDF.matrix_to_xyz_rpy(T)
    # print("Vector:")
    # print(vector)
    return position


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
    # action[0] = 0.1
    # action[1] = 0.1
    # action[3] = 1
    # action[1] = 0.1
    # action[arm_joints[7]] = 1
    # action[arm_joints[5]]=0
    ob = env.reset()
    print(f"Initial observation : {ob}")
    urdf_path="/home/jose/anaconda3/envs/PDM/lib/python3.10/site-packages/robotmodels/mobilePanda/urdf/mobilePanda_with_gripper.urdf"
    robot = URDF.load(urdf_path)
    
    range_joints=range(len(robot.actuated_joints)-3)
    joints=[]
    print(robot.actuated_joints[0])
    for x in tqdm(range_joints):
        print("X.",x)
        joint=robot.actuated_joints[x+3]
        joints.append(joint)
        # joint_type = joints[x].joint_type
        # joint_limits = joints[x].limit
        # print(f"{joints[x].name:<20} {joint_type:<15} {x:<10}")
        # print(f"Lower Limit: {joint_limits.lower:<15} Upper Limit: {joint_limits.upper:<10}")
        # print(np.round(joints[x].origin,5))

    ob, *_ = env.step(action) 
    print(np.round(ob['robot_0']['joint_state']['position'],2))
    print(np.round(ob['robot_0']['joint_state']['position'][3:-2],4))
    current_xyz=compute_forward_kinematics(joints, np.round(ob['robot_0']['joint_state']['position'][3:-2],4))
    print("End Position")
    print(np.round(current_xyz))
    ja=compute_jacobian(joints, np.round(ob['robot_0']['joint_state']['position'][3:-2],4), current_xyz)
    print("Jacobian")
    print(ja)
    robot.show()
    history = []
    target_xyz = np.array([0.8, 0, 0.5])
    max_velocity = 0.5
    # print(f"Intial velocity: {ob['robot_0']['joint_state']['velocity']}")
    for i in range(n_steps):
        # if (int(i / 100)) % 2 == 0:
        #     action[11] = -0.01
        #     action[10] = -0.01
        # else:
        #     action[11] = 0.01
        #     action[10] = 0.01
        if (np.linalg.norm(target_xyz - current_xyz) > 0.01):  # Loop until close to target
            # Step 1: Compute desired Cartesian velocity (proportional control for simplicity)
            error_xyz = target_xyz - current_xyz
            desired_velocity_xyz = 1.0 * error_xyz  # Proportional gain (1.0)

            if np.linalg.norm(desired_velocity_xyz) > max_velocity:
                desired_velocity_xyz = desired_velocity_xyz / np.linalg.norm(desired_velocity_xyz) * max_velocity
            desired_velocity = np.hstack((desired_velocity_xyz, np.zeros(3))) 
            # Step 2: Compute Jacobian at current joint positions
            J= compute_jacobian(joints, np.round(ob['robot_0']['joint_state']['position'][3:-2],4), current_xyz)

            # Step 3: Compute joint velocities
            print("Jacobian")
            print(J)
            print("Error")
            print(desired_velocity)
            joint_velocities = np.linalg.pinv(J) @ desired_velocity # Use pseudoinverse to solve

            # Step 4: Send joint velocities to the robot
            for x in range(len(joints)-2):
                # print("X.",x)
                action[x+3]=joint_velocities[x]
                

            # Step 5: Update current joint positions and end-effector position
            
            
        else:
            for x in range(len(joints)-2):
                # print("X.",x)
                action[x+3]=0

        ob, *_ = env.step(action) 
        current_xyz = compute_forward_kinematics(joints, np.round(ob['robot_0']['joint_state']['position'][3:-2],4))  # FK to get xyz    
        # for x in range(len(action)):
        #     if (np.abs(np.round(ob['robot_0']['joint_state']['velocity'][x],4)) > 0.1):
        #         action[x]=-ob['robot_0']['joint_state']['velocity'][x]
        #     else:
        #         action[x]=0
        # print(f"Velocity: {np.round(ob['robot_0']['joint_state']['position'],2)}")
        # if (ob['robot_0']['joint_state']['velocity'][arm_joints[0]]>0.5):
        #     action[arm_joints[0]] = 0.0
        history.append(ob)
    env.close()

    

    return history


if __name__ == "__main__":
    run_mobile_reacher(render=True)