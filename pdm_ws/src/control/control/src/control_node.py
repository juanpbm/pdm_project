import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from std_msgs.msg import String

class ControlNode(Node):
    def __init__(self):
        super().__init__('control')
        self.publisher_ = self.create_publisher(String, 'cmd_vel', 10)
        self.subscription = self.create_subscription(
            String,
            'trajectory',
            self.trajectory_callback,
            10)

        # TODO: what other information or topics are needed?
        print("control Node has been created.")

    def trajectory_callback(self, msg):
        self.get_logger().info('Got in Control Node sub: "%s"' % msg.data)

        # Compute velocities

        msg_new = String()
        msg_new.data = 'Controller has calculated velocity'
        self.publisher_.publish(msg_new)
        self.get_logger().info('Publishing from control Node: "%s"' % msg_new.data)

    #Functions that use the motion planning class to compute the RRT

def main(args=None):
    # start the motion planning node
    try:
        rclpy.init(args=args)
        control_node = ControlNode()

        rclpy.spin(control_node)
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