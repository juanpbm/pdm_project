import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sim_env.src.robot_env import Albert_sym
import warnings
import gymnasium as gym

from std_msgs.msg import String

class PandaEnvNode(Node):
    def __init__(self):
        super().__init__('panda_env')
        self.publisher_ = self.create_publisher(String, 'map', 10)
        self.subscription = self.create_subscription(
            String,
            'cmd_vel',
            self.cmd_callback,
            10)

        # TODO: what other information or topics are needed?
        print("panda env Node has been created.")


    def cmd_callback(self, msg):
        self.get_logger().info('Got in panda_env Node sub: "%s"' % msg.data)

        # act on received command
        # get new info to publish 

        msg_new = String()
        msg_new.data = 'panda env has acted on cmd_vel '
        self.publisher_.publish(msg_new)
        self.get_logger().info('Publishing from panda_env Node: "%s"' % msg_new.data)

    #Functions that use the env class

def main(args=None):
    # start the panda_env node
    try:
        rclpy.init(args=args)
        panda_env_node = PandaEnvNode()

        albert_sym = Albert_sym(render=True)
        albert_sym.move()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if panda_env_node is not None:
            panda_env_node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
            
        print("motion_planner Node has been shut down.")

if __name__ == "__main__":
    # Call Main Function
    main()