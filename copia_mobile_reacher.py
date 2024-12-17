import numpy as np

from urdfenvs.robots.generic_urdf import GenericUrdfReacher
from urdfenvs.urdf_common.urdf_env import UrdfEnv
from urdfpy import URDF

def compute_dh_parameters(robot):
    """
    Compute the DH parameters for a given URDF robot model.
    """
    dh_table = []
    print(f"{'Joint Name':<15} {'a (m)':<10} {'alpha (rad)':<15} {'d (m)':<10} {'theta (rad)':<15}")
    print("=" * 60)
    
    for joint in robot.joints:
        if joint.joint_type not in ['revolute', 'prismatic']:
            continue  # Skip fixed joints as they don't contribute to DH parameters

        # Extract joint parameters
        origin = joint.origin
        xyz = origin.xyz if origin else [0, 0, 0]
        rpy = origin.rpy if origin else [0, 0, 0]
        axis = joint.axis if joint.axis else [0, 0, 1]

        # DH parameters
        a = np.linalg.norm(xyz[:2])  # Distance between z-axes in the x-y plane
        alpha = rpy[0]  # Twist angle (rotation about x-axis)
        d = xyz[2]      # Offset along z-axis
        theta = rpy[2]  # Angle about z-axis

        # Append to DH table
        dh_table.append({
            "joint_name": joint.name,
            "a": a,
            "alpha": alpha,
            "d": d,
            "theta": theta
        })

        # Print DH row
        print(f"{joint.name:<15} {a:<10.4f} {alpha:<15.4f} {d:<10.4f} {theta:<15.4f}")

    return dh_table

def compute_fk(robot, joint_angles):
    """
    Compute forward kinematics for a robot arm described by a URDF file.
    
    Args:
        urdf_path (str): Path to the URDF file.
        joint_angles (dict): Dictionary mapping joint names to angles (in radians).
    
    Returns:
        np.ndarray: 4x4 transformation matrix of the end effector.
    """
    # Load the URDF file
    
    
    # Define the joint configuration
    joint_states = {}
    for joint_name, angle in joint_angles.items():
        joint_states[joint_name] = angle
    
    # Compute forward kinematics
    end_effector_name = robot.links[-1].name  # Assuming last link is the end effector
    fk_result = robot.link_fk(cfg=joint_states)
    
    # Extract the end effector's transformation matrix
    end_effector_transform = fk_result[robot.link_map[end_effector_name]]
    print(end_effector_transform)
    return end_effector_transform


def run_mobile_reacher(n_steps=1000, render=False, goal=True, obstacles=True):
    robots = [
        GenericUrdfReacher(urdf="mobilePanda_with_gripper.urdf", mode="vel"),
    ]
    env: UrdfEnv = UrdfEnv(
        dt=0.01, robots=robots, render=render, num_sub_steps=200,
    )
    action = np.zeros(env.n())
    
    ob = env.reset()
    # action[0] = 0.1
    # action[5] = -0.0
    # action[-1] = 1
    print(f"Initial observation : {ob}")

    urdf_path="/home/jose/anaconda3/envs/PDM/lib/python3.10/site-packages/robotmodels/mobilePanda/urdf/mobilePanda_with_gripper.urdf"
    robot = URDF.load(urdf_path)
    fk= robot.link_fk()
    print(len(robot.links))
    for x in range(len(robot.links)):
        print(robot.links[x].name)
        print(np.round(fk[robot.links[x]]))
      
    dh_table = compute_dh_parameters(robot)
    print("\nProgrammatically Accessible DH Table:")
    # for row in dh_table:
    #     print(row)
    # joint_angles = {
    #     "panda_joint_base": 0.0,
    #     "panda_joint1": np.pi / 4,
    #     "panda_joint2": np.pi / 6,
    #     "panda_joint3": 0.0,
    #     "panda_joint4": np.pi / 3,
    #     "panda_joint5": np.pi / 4,
    #     "panda_joint6": np.pi / 2,
    #     "panda_joint7": np.pi / 2
    # }
    
    # Compute the FK
    # end_effector_pose = compute_fk(robot, joint_angles)


    history = []
    # ob, *_ = env.step(action)
    # history.append(ob)
    for i in range(n_steps):
        # action[0] = 1
        # end_effector_pose = compute_fk(robot, joint_angles)
        ob, *_ = env.step(action)
        history.append(ob)
    env.close()
    return history


if __name__ == "__main__":
    run_mobile_reacher(render=True)
