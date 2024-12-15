import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from std_msgs.msg import String

class MotionPlannerNode(Node):
    def __init__(self):
        super().__init__('motion_planner')
        self.publisher_ = self.create_publisher(String, 'trajectory', 10)
        self.subscription = self.create_subscription(
            String,
            'map',
            self.map_callback,
            10)

        # TODO: what other information or topics are needed?
        print("motion_planner Node has been created.")


    def map_callback(self, msg):
        self.get_logger().info('Got in Motion Planning Node sub: "%s"' % msg.data)

        # Compute trajectory

        msg_new = String()
        msg_new.data = 'Motion Planning has calculated trajectory'
        self.publisher_.publish(msg_new)
        self.get_logger().info('Publishing from Motion Planning Node: "%s"' % msg_new.data)

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