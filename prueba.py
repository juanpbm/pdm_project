
from urdfpy import URDF
import numpy as np
def run_mobile_reacher(n_steps=1000, render=False, goal=True, obstacles=True):

    urdf_path="/home/jose/anaconda3/envs/PDM/lib/python3.10/site-packages/robotmodels/mobilePanda/urdf/mobilePanda_with_gripper.urdf"
    robot = URDF.load(urdf_path)

    # Print robot name
    print(f"Robot Name: {robot.name if robot.name else 'Unnamed Robot'}")

    # Summarize joints
    print("\nSummary of Joints:")
    print(f"{'Joint Name':<20} {'Type':<15} {'Parent Link':<15} {'Child Link':<15}")
    fk= robot.link_fk()
    for x in range(len(robot.links)):
        print(robot.links[x].name)
        # print(np.round(fk[robot.links[x]]))
    
    print("Origen")
    for joint in robot.joints:
        if joint.joint_type not in ['revolute', 'prismatic']:
            continue  # Skip fixed joints as they don't contribute to DH parameters

        # Extract joint parameters
        print(f"Joint Name: {joint.name}")
        print(np.round(joint.origin,5))
        # if joint.origin:
        #     xax = joint.origin.xyz
        #     print(f"Origin XYZ (rounded): {np.round(xax)}")
        # else:
        #     print("No origin defined for this joint.")

    for joint in robot.actuated_joints:
        joint_type = joint.joint_type
        if joint_type in ['revolute', 'prismatic']:
        # Default value is 0 if not explicitly specified in the URDF
            print(f"{joint.name:<20} {joint_type:<15} 0.0")

    robot.show()

    return 0


if __name__ == "__main__":
    run_mobile_reacher(render=True)
