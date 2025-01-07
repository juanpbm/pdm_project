import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
import numpy as np
from std_msgs.msg import Float64MultiArray, MultiArrayDimension, String
import cv2 as cv
from motion_planning.src.motion_planner import RRT

from ament_index_python.packages import get_package_share_directory
import os


class MotionPlannerNode(Node):
    def __init__(self):
        super().__init__('motion_planner')
        self.base_trajectory_publisher_ = self.create_publisher(Float64MultiArray, 'base_trajectory', 10)
        self.arm_trajectory_publisher_ = self.create_publisher(Float64MultiArray, 'arm_trajectory', 10)
        self.subscription = self.create_subscription(
            String,
            'map',
            self.map_callback,
            10)

        # TODO: what other information or topics are needed?
        print("motion_planner Node has been created.")

    def map_callback(self, msg):
        self.get_logger().info('Got in Motion Planning Node sub: "%s"' % msg.data)

        base_trajectory = self.temp_map_rrt()
        # Dummy trajectory. The computed trajectory should return something similar
        arm_trajectory = np.array([[0.6, 0, 0.5]])

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


    #Functions that use the motion planning class to compute the RRT

    def temp_map_rrt(self):
        file_path_img = os.path.join(os.path.dirname(get_package_share_directory('motion_planning')), 'motion_planning', 'resource', 'second.jpg')
        img = cv.imread(file_path_img)
        rrt = RRT()
        # Properties of an Image
        gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)
        ret,thresh = cv.threshold(gray,127,255,0)

        size = img.shape
        start = (50,50)
        end = (325,450)
        rad = 10

        img_print = img.copy()
        image = img.copy()

        kernel = np.ones((8, 8), np.uint8) 
    
        # Using cv2.erode() method  
        image = cv.erode(thresh, kernel, cv.BORDER_REFLECT)  

        # Draw a circle of red color of thickness -1 px 
        img_print = cv.circle(img_print, (start[1],start[0]), 3, (100,255,255), -1) 

        # Draw a circle of red color of thickness -1 px 
        img_print = cv.circle(img_print, (end[1],end[0]), rad, (255,0,0), 1) 

        [V_E, img_print] = rrt.RRT_star(np.squeeze(image), start, end,rad,size,img_print)
        V_E = np.asarray(V_E)
        for i in V_E:
            #print(V_E.shape)
            if(i.parent != (None,None)):
                img_print = cv.line(img_print, (i.child[1],i.child[0]), (i.parent[1],i.parent[0]), (255,0,255), 1)


        V_E_shortest = rrt.find_shortest([V_E[-1]],V_E,V_E[-1])

        V_E_shortest_smoothed = rrt.smooth_path(V_E_shortest, np.squeeze(image))
        message = []
        # Draw the smoothed path
        j = len(V_E_shortest_smoothed) - 1
        for i in range(len(V_E_shortest_smoothed) - 1):
            p1 = V_E_shortest_smoothed[i].child
            p2 = V_E_shortest_smoothed[i + 1].child
            if(i == 0):
                message.append([V_E_shortest_smoothed[j].child[0]/100, V_E_shortest_smoothed[j].child[1]/100, 0])

            message.append([V_E_shortest_smoothed[j-1].child[0]/100, V_E_shortest_smoothed[j-1].child[1]/100, 0])
            j = j-1

            img_print = cv.line(img_print, (p1[1], p1[0]), (p2[1], p2[0]), (0, 0, 255), 2)  # Smoothed path in yellow

        # print(message)
        # Display the Binary Image
        cv.imshow("Binary Image", img_print)
        cv.waitKey(0)
        cv.destroyAllWindows()
        return np.array(message)
    
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