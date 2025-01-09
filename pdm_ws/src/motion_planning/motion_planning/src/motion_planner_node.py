import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
import numpy as np
import matplotlib.pyplot as plt

from std_msgs.msg import Float64MultiArray, MultiArrayDimension

class MotionPlannerNode(Node):
    def __init__(self):
        super().__init__('motion_planner')
        self.base_trajectory_publisher_ = self.create_publisher(Float64MultiArray, 'base_trajectory', 10)
        self.arm_trajectory_publisher_ = self.create_publisher(Float64MultiArray, 'arm_trajectory', 10)
        self.subscription = self.create_subscription(
            Float64MultiArray,
            'map',
            self.map_callback,
            10)

        # TODO: what other information or topics are needed?
        print("motion_planner Node has been created.")

    def map_callback(self, msg):
        self.get_logger().info('Got Map in Motion Planning Node sub')

        dims = msg.layout.dim
        if len(dims) == 0:
            self.get_logger().error('Received an array with no dimensions.')
            return
        shape = tuple(dim.size for dim in dims)
        map = np.array(msg.data).reshape(shape)

        slice_index = 1
        slice_data = map[:,:,slice_index]

        plt.imshow(slice_data, cmap="gray", origin="lower")
        plt.title(f"Occupancy map")
        plt.show()

        # TODO: compute trajectory

        # Dummy trajectory. The computed trajectory should return something similar
        base_trajectory = np.array([[1, 1, 0], [2, 2, 0], [1, 2, 0], [0, 0, 0]], dtype=float) 
        arm_trajectory = np.array([[0.6, 0, 0.5]])
        self.pub_trajectories(base_trajectory, arm_trajectory)

    def pub_trajectories(self, base_trajectory, arm_trajectory):
        # Create array message with the base trajectory information
        base_msg = Float64MultiArray()
        base_msg.data = base_trajectory.flatten().tolist()
        assert all(isinstance(val, float) for val in base_msg.data) # All elements must be floats
        # Define dimensions of the msg to reconstruct by the subscribers
        base_msg.layout.dim.append(MultiArrayDimension(label='rows', size=base_trajectory.shape[0], stride=base_trajectory.shape[1] * base_trajectory.shape[0]))
        base_msg.layout.dim.append(MultiArrayDimension(label='cols', size=base_trajectory.shape[1], stride=base_trajectory.shape[1]))

        # Publish base Trajectory 
        self.base_trajectory_publisher_.publish(base_msg)
        self.get_logger().info('Published base_target_xyz:"%s"' % base_msg.data)

        # Create array message with the arm trajectory information
        arm_msg = Float64MultiArray()
        arm_msg.data = arm_trajectory.flatten().tolist()
        assert all(isinstance(val, float) for val in arm_msg.data) # All elements must be floats
        # Define dimensions of the msg to reconstruct by the subscribers
        arm_msg.layout.dim.append(MultiArrayDimension(label='rows', size=arm_trajectory.shape[0], stride=arm_trajectory.shape[1] * arm_trajectory.shape[0]))
        arm_msg.layout.dim.append(MultiArrayDimension(label='cols', size=arm_trajectory.shape[1], stride=arm_trajectory.shape[1]))

        # Publish arm trajectory
        self.arm_trajectory_publisher_.publish(arm_msg)
        self.get_logger().info('Published arm_target_xyz:"%s"' % arm_msg.data)

def main(args=None):
    # start the motion planning node
    try:
        rclpy.init(args=args)
        motion_planner_node = MotionPlannerNode()

        rclpy.spin(motion_planner_node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if motion_planner_node is not None:
            motion_planner_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
            
        print("motion_planner Node has been shut down.")

if __name__ == "__main__":
    # Call Main Function
    main()