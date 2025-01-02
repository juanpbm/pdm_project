import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
import numpy as np
from urdfenvs.robots.generic_urdf import GenericUrdfReacher
from urdfenvs.urdf_common.urdf_env import UrdfEnv

from std_msgs.msg import Float64MultiArray, MultiArrayDimension
from std_msgs.msg import String

class MotionPlannerNode(Node):
    def __init__(self):
        super().__init__('motion_planner')
        self.trajectory_publisher_ = self.create_publisher(Float64MultiArray, 'trajectory', 10)
        self.subscription = self.create_subscription(
            String,
            'map',
            self.map_callback,
            10)

        # TODO: what other information or topics are needed?
        print("motion_planner Node has been created.")

    def map_callback(self, msg):
        self.get_logger().info('Got in Motion Planning Node sub: "%s"' % msg.data)

        # TODO: compute trajectory

        target_xyz = np.array([[1, 1, 0], [2, 2, 0], [1, 2, 0], [0, 0, 0]], dtype=float)
        msg = Float64MultiArray()

        # Flatten the array and assign to data
        msg.data = target_xyz.flatten().tolist()
        assert all(isinstance(val, float) for val in msg.data), "All elements must be floats"

        # Define dimensions
        msg.layout.dim.append(MultiArrayDimension(label='rows', size=target_xyz.shape[0], stride=target_xyz.shape[1] * target_xyz.shape[0]))
        msg.layout.dim.append(MultiArrayDimension(label='cols', size=target_xyz.shape[1], stride=target_xyz.shape[1]))

        self.trajectory_publisher_.publish(msg)
        self.get_logger().info("Published target_xyz array.")

    #Functions that use the motion planning class to compute the RRT

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