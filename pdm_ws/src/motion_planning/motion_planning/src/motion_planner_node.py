import os
import cv2 as cv
from geometry_msgs.msg import Point
from motion_planning.src.motion_planner import RRT
import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, MultiArrayDimension, Int32
import matplotlib.pyplot as plt
from ament_index_python.packages import get_package_share_directory

class MotionPlannerNode(Node):
    def __init__(self):
        super().__init__('motion_planner')
        self.base_trajectory_publisher_ = self.create_publisher(Float64MultiArray, 'base_trajectory', 10)
        self.arm_trajectory_publisher_ = self.create_publisher(Float64MultiArray, 'arm_trajectory', 10)
        self.map_subscription_ = self.create_subscription(
            Float64MultiArray,
            'map',
            self.map_callback,
            10)
        self.goal_subscription_ = self.create_subscription(
            Point,
            'goal_pos',
            self.goal_callback,
            10)
        self.init_pos_subscription_ = self.create_subscription(
            Point,
            'init_pos',
            self.init_callback,
            10)

        self.new_state_subscription_ = self.create_subscription(
            Int32,
            'new_state',
            self.new_state_callback,
            10)
        self.base_pos_subscription = self.create_subscription(
            Point,
            'base_pos',
            self.base_pos_callback,
            10)
        
        self.state_ = 0

        # TODO: what other information or topics are needed?
        self.map = None
        self.base_trajectory = np.empty(0)
        self.arm_trajectory = np.empty(0)
        self.base_goal = np.empty(0)
        self.init_pos = np.empty(0)
        self.base_current_pos= np.empty(0)
        self.safe_arm_pos = np.array([[0.32, 0, 0.3]],dtype=float)
        self.get_logger().info("motion_planner Node has been created.")

    def map_callback(self, msg):
        self.get_logger().info('Got Map in Motion Planning Node sub')

        if (self.map is None):
            dims = msg.layout.dim
            if len(dims) == 0:
                self.get_logger().error('Received an array with no dimensions.')
                return
            shape = tuple(dim.size for dim in dims)
            self.map = np.array(msg.data).reshape(shape)
        if(self.map_ready and self.base_trajectory.size == 0):
            self.base_trajectory = self.map_rrt()
            self.get_logger().info('generated base_trajectory:"%s"' % self.base_trajectory)

    def base_pos_callback(self, msg):    
        # Recover position information as a 1D array
        self.base_current_pos = np.array([msg.x, msg.y, msg.z], dtype=float)
        self.get_logger().info('Got base pos in Control Node sub: "%s"' % self.base_current_pos)

    # Get the new environment from the environment
    def new_state_callback(self, msg):
        self.state_ = msg.data
        if(self.state_ == 1):
            while(self.base_trajectory == np.empty(0)):
                return
            # safe position
            self.arm_trajectory = self.safe_arm_pos


        elif(self.state_ == 3):
            # goal position

            # Add the offset as a new row to the trajectory
            self.arm_trajectory = np.vstack([np.array([0.32, 0, 0.6], dtype=float), np.array([self.arm_goal],dtype=float)])

        elif(self.state_ == 4):
            # safe position
            self.arm_trajectory = self.safe_arm_pos

        elif(self.state_ == 5):
            self.base_trajectory = self.base_trajectory[::-1]
            self.arm_trajectory = self.safe_arm_pos

        elif(self.state_ == 6):
            # drop position
            self.arm_trajectory = np.array([[0.4, 0, 0.1]])

        self.pub_trajectories()

    def goal_callback(self, msg):
        self.get_logger().info('Got goal pos in motion planner:"%s"' % msg)
        if(self.base_current_pos.size != 0):
            self.arm_goal = np.array([msg.x -self.base_current_pos[0], msg.y-self.base_current_pos[1], msg.z + 0.2], dtype=float)
        self.base_goal = np.array([msg.x -0.6, msg.y, 0], dtype=float)
    
    def init_callback(self, msg):
        self.get_logger().info('Got init pos in motion planner:"%s"' % msg)
        self.init_pos = np.array([msg.x, msg.y, msg.z], dtype=int)

    def pub_trajectories(self):
        # Create array message with the base trajectory information
        base_msg = Float64MultiArray()
        base_msg.data = self.base_trajectory.flatten().tolist()
        assert all(isinstance(val, float) for val in base_msg.data) # All elements must be floats
            # Define dimensions of the msg to reconstruct by the subscribers
        base_msg.layout.dim.append(MultiArrayDimension(label='rows', size=self.base_trajectory.shape[0], stride=self.base_trajectory.shape[1] * self.base_trajectory.shape[0]))
        base_msg.layout.dim.append(MultiArrayDimension(label='cols', size=self.base_trajectory.shape[1], stride=self.base_trajectory.shape[1]))

        # Publish base Trajectory 
        self.base_trajectory_publisher_.publish(base_msg)
        self.get_logger().info('Published base_target_xyz:"%s"' % base_msg.data)

        # Create array message with the arm trajectory information
        arm_msg = Float64MultiArray()
        arm_msg.data = self.arm_trajectory.flatten().tolist()
        assert all(isinstance(val, float) for val in arm_msg.data) # All elements must be floats
        # Define dimensions of the msg to reconstruct by the subscribers
        arm_msg.layout.dim.append(MultiArrayDimension(label='rows', size=self.arm_trajectory.shape[0], stride=self.arm_trajectory.shape[1] * self.arm_trajectory.shape[0]))
        arm_msg.layout.dim.append(MultiArrayDimension(label='cols', size=self.arm_trajectory.shape[1], stride=self.arm_trajectory.shape[1]))

        # Publish arm trajectory
        self.arm_trajectory_publisher_.publish(arm_msg)
        self.get_logger().info('Published arm_target_xyz:"%s"' % arm_msg.data)

    def map_rrt(self):
        image_array = self.map
        rrt = RRT()

        # Ensure the data type is uint8
        if image_array.dtype != np.uint8:
            img = (image_array * 255).astype(np.uint8)
        else:
            img = image_array.copy()

        # Get only a slice of the occupancy grid (number 1)
        slice_index = 1
        img = img[:,:,slice_index]

        # Flip, rotate and resize the image, and invert the colors, to adjust it to the requirements of the algorithm
        img = cv.flip(img,1)
        img = cv.rotate(img, cv.ROTATE_90_COUNTERCLOCKWISE)
        img = 255 - img

        img  = cv.resize(img, (200, 160), interpolation = cv.INTER_LINEAR)

        # Apply a threshold to the image to ensure that there is only black and white colors (0 and 255 values)
        ret,thresh = cv.threshold(img,254,255,0)

        # Variables to make the algorithm work
        size = img.shape
        start = (np.abs(self.init_pos[1].astype(int) ) * 10, self.init_pos[0].astype(int) * 10)
        end = (np.abs(self.base_goal[1].astype(int) ) * 10, self.base_goal[0].astype(int)  * 10)
        rad = 5
        done = False
        img_print = img.copy()
        image = img.copy()

        kernel = np.ones((6, 6), np.uint8) 
    
        # Using cv2.erode() method  
        image = cv.erode(thresh, kernel, cv.BORDER_REFLECT)  

        # Draw a circle of red color of thickness -1 px 
        img_print = cv.circle(img_print, (start[1],start[0]), 3, (100,255,255), -1) 

        # Draw a circle of red color of thickness -1 px 
        img_print = cv.circle(img_print, (end[1],end[0]), rad, (255,0,0), 1) 

        cv.imshow("Binary Image", image)
        cv.waitKey(0)
        cv.destroyAllWindows()
        [V_E, img_print, id] = rrt.RRT_star(np.squeeze(image), start, end,rad,size,img_print)
        print("HAS BEEN FOUND: " + str(id))
        V_E = np.asarray(V_E)
        for i in V_E:
            if(i.parent != (None,None)):
                img_print = cv.line(img_print, (i.child[1],i.child[0]), (i.parent[1],i.parent[0]), (255,0,255), 1)


        V_E_shortest = rrt.find_shortest([V_E[id]],V_E,V_E[id])

        V_E_shortest_smoothed = rrt.smooth_path(V_E_shortest, np.squeeze(image))
        message = []
        # Draw the smoothed path
        j = len(V_E_shortest_smoothed) - 1
        for i in range(len(V_E_shortest_smoothed) - 1):
            p1 = V_E_shortest_smoothed[i].child
            p2 = V_E_shortest_smoothed[i + 1].child
            
            if(i == 0):
                message.append([(V_E_shortest_smoothed[j].child[1]/10), -V_E_shortest_smoothed[j].child[0]/10, 0])

            points=np.linspace(np.array([V_E_shortest_smoothed[j].child[1]/10, -V_E_shortest_smoothed[j].child[0]/10, 0]),np.array([V_E_shortest_smoothed[j-1].child[1]/10, -V_E_shortest_smoothed[j-1].child[0]/10, 0]),num=22)
            message.extend([point.tolist() for point in points[1:]])
                
            j = j-1

            img_print = cv.line(img_print, (p1[1], p1[0]), (p2[1], p2[0]), (0, 0, 255), 2)  # Smoothed path in yellow

        message[-1][0]+=0.4

        img_path = os.path.join(os.path.dirname(get_package_share_directory('motion_planning')), 'motion_planning', 'resource', "RRT_star_result.png")
        fig = plt.figure(figsize=(3, 3))
        plt.imshow(img_print, cmap='gray')
        plt.title("RRT* Result")
        plt.axis('off')
        fig.savefig(img_path, dpi=fig.dpi)
       
        return np.array(message, dtype=float)
    
    def map_ready(self):
        # make sure that the map and positions are ready
        return self.map is not None and self.base_goal.size != 0 and self.init_pos.size != 0
    
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
            
        motion_planner_node.get_logger().info("motion_planner Node has been shut down.")

if __name__ == "__main__":
    # Call Main Function
    main()