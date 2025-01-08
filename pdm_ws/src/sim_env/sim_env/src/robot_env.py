import warnings
import gymnasium as gym
import matplotlib.pyplot as plt
import numpy as np
import pybullet as p
import cv2 as cv

from urdfenvs.robots.generic_urdf import GenericUrdfReacher
from urdfenvs.urdf_common.urdf_env import UrdfEnv
from mpscenes.obstacles.box_obstacle import BoxObstacle
from mpscenes.obstacles.sphere_obstacle import SphereObstacle

from urdfenvs.sensors.occupancy_sensor import OccupancySensor


class Panda_Sym:
    def __init__(self, render=True,
                 albert_pos=np.array([0.0, -3.0, 0.0]),
                 boundary_size=20, 
                 room_size = 6,
                 door_size = 1,
                 obstacle_size = 0.5):
        
        self.render = render
        self.boundary_size = boundary_size
        self.room_size = room_size
        self.door_size = door_size
        self.obstacle_size = obstacle_size
        self.height = 2
        self.albert_pos = [-(self.boundary_size/2 - self.room_size/2), -(self.boundary_size/2 - self.room_size/2), 0.0]
        self.init_pos = np.array([4.0,-14.0,0.0])
        self.env: UrdfEnv = None
        self.Gen_Env()
        img = self.capture_overhead_image(self.env)

        cv.imwrite("/home/zuleikarg/Desktop/Map.png", img)

        cv.imshow('image window', img)
        # add wait key. window waits until user presses a key
        cv.waitKey(0)
        #   and finally destroy/close all open windows
        cv.destroyAllWindows()

        history = self.run_point_robot_with_occupancy_sensor()
    
    def __del__(self):
        if self.env is not None:
            self.env.close()

    def capture_overhead_image(self, environment,width = 640, height = 480, fov = 60, near_val = 0.1, far_val = 100.0,
    camera_position = (10.0, -8.0, 14), target_position = (10.0, -8.0, 0), up_vector = (0, 2, 0)):
        view_matrix  = p.computeViewMatrix(cameraEyePosition=camera_position, cameraTargetPosition=target_position, cameraUpVector=up_vector)

        projection_matrix =p.computeProjectionMatrixFOV(
        fov=fov,
        aspect=float(width) / height,
        nearVal=near_val,
        farVal=far_val)

        _, _, rgb_image, _, _ = p.getCameraImage(
        width=width,
        height=height,
        viewMatrix=view_matrix,
        projectionMatrix=projection_matrix,
        renderer=p.ER_BULLET_HARDWARE_OPENGL)

        gray = cv.cvtColor(rgb_image, cv.COLOR_BGR2GRAY)
        ret,thresh = cv.threshold(gray,250,255,cv.THRESH_BINARY+cv.THRESH_OTSU)
        #print(thresh)
        return np.reshape(thresh, (height, width))[:, :]

# TRYING SOMETHING HERE

    def get_index_from_coordinates(self,point, mesh) -> tuple:
        distances = np.linalg.norm(mesh - point, axis=3)
        return np.unravel_index(np.argmin(distances), mesh.shape[:-1])

    def evaluate_occupancy(self, point, mesh, occupancy, resolution) -> int:
        index = list(self.get_index_from_coordinates(point, mesh))
        return occupancy[tuple(index)]

    def run_point_robot_with_occupancy_sensor(self,n_steps=10, render=False, obstacles=True, goal=True):
        
        # add sensor
        val = 80
        sensor = OccupancySensor(
            limits =  np.array([[0, 20], [0, -16], [0, 50/val]]),
            resolution = np.array([80, 64, 5], dtype=int),
            interval=100,
            plotting_interval=100,
        )

        self.env.add_sensor(sensor, [0])
        # Set spaces AFTER all components have been added.
        self.env.set_spaces()
        defaultAction = np.array([1.0,-1.0,0.0])
        pos0 = np.array([13.0,-14.0,0.0])
        vel0 = np.array([0.0, 0.0, 0.0])
        initial_observations = []
        self.ob, _ = self.env.reset(mount_positions=np.array([self.init_pos]), vel=vel0)
        initial_observations.append(self.ob)
        self.env.add_debug_shape(
            (0.2, -0.3, 0.0),
            (0.0, 0.0, 0.0, 1.0),
            size=[0.3],
            rgba_color=[0.0, 0.0, 0.0, 0.3],
        )
        point=[]
        action = np.zeros(self.env.n())
        history = []
        final_occupancy = None
        for _ in range(n_steps):
            action[:3] = defaultAction
            self.ob, *_ = self.env.step(action)
            point = np.append(self.ob['robot_0']['joint_state']['position'][0:2], 0.0)
            occupancy = self.ob['robot_0']['Occupancy']
            occupancy_eval = self.evaluate_occupancy(point, sensor.mesh(), occupancy, [0.2, 0.2, 1])
            print(occupancy_eval)
            final_occupancy = occupancy
            history.append(self.ob)

        #if(final_occupancy!= None):
        np.save("occupancy_grid.npy", final_occupancy)
        print("The occupancy map has been saved")
        slice_index = 1
        slice_data = final_occupancy[:,:,slice_index]

        plt.imshow(slice_data, cmap="gray", origin="lower")
        plt.title(f"Occupancy map")
        plt.show()


        self.env.close()
        return history





# TRYING SOMETHING BEFORE


    def Gen_Env(self):
        self.Gen_Panda()
        self.Gen_Boundary()
        self.Gen_Rooms()
        self.Gen_Room_Obstacles()
        self.Gen_Shelf()
        self.env = UrdfEnv(dt=0.01, robots=self.panda_robot, render=self.render, num_sub_steps=200,)

        for wall in self.boundary_walls:
            self.env.add_obstacle(wall)
        for wall in self.room1_walls:
            self.env.add_obstacle(wall)
        for wall in self.room2_walls:
            self.env.add_obstacle(wall)
        self.env.add_obstacle(self.obstacle_room1)
        self.env.add_obstacle(self.obstacle_room2)
        #self.env.add_obstacle(self.sphereObst1)
        self.env.add_obstacle(self.shelf1)
        #self.env.add_obstacle(self.shelf2)
        self.env.add_obstacle(self.shelf3)
        self.env.add_obstacle(self.shelf4)
        #self.env.add_obstacle(self.shelf5)
        self.env.add_obstacle(self.shelf6)
        #action = np.zeros(self.env.n())
        self.ob = self.env.reset(mount_positions=np.array([self.init_pos]))
        #self.ob, *_ = self.env.step(action) 

    def Gen_Panda(self):
        self.panda_robot = [
            GenericUrdfReacher(urdf="mobilePanda_with_gripper.urdf", mode="vel"),
        ]
    
    def Gen_Boundary(self):

        boundary_obs_dicts = [
            {
                'type': 'box', 
                'geometry': {
                    'position': [self.boundary_size, -16.0/2, 0.4], 'width': 16, 'height': self.height, 'length': 0.1
                },
                'high': {
                    'position' : [self.boundary_size, -16.0/2, 0.4],
                    'width': 16,
                    'height': self.height,
                    'length': 0.1,
                },
                'low': {
                    'position' : [self.boundary_size, -16.0/2, 0.4],
                    'width': 16,
                    'height': self.height,
                    'length': 0.1,
                },
            },
            {
                'type': 'box', 
                'geometry': {
                    'position': [self.boundary_size/2, 0.0, 0.4], 'width': 0.1, 'height': self.height, 'length': self.boundary_size
                },
                'high': {
                    'position' : [self.boundary_size/2, 0.0, 0.4],
                    'width': 0.1,
                    'height': self.height,
                    'length': self.boundary_size,
                },
                'low': {
                    'position' : [self.boundary_size/2, 0.0, 0.4],
                    'width': 0.1,
                    'height': self.height,
                    'length': self.boundary_size,
                },
            },
            {
                'type': 'box', 
                'geometry': {
                    'position': [self.boundary_size/2, -16, 0.4], 'width': 0.1, 'height': self.height, 'length': self.boundary_size
                },
                'high': {
                    'position' : [self.boundary_size/2, -16, 0.4],
                    'width': 0.1,
                    'height': self.height,
                    'length': self.boundary_size,
                },
                'low': {
                    'position' : [self.boundary_size/2, -16, 0.4],
                    'width': 0.1,
                    'height': self.height,
                    'length': self.boundary_size,
                },
            },
            {
                'type': 'box', 
                'geometry': {
                    'position': [0.0, -16.0/2, 0.4], 'width': 16, 'height': self.height, 'length': 0.1
                },
                'high': {
                    'position' : [0.0, -16.0/2, 0.4],
                    'width': 16,
                    'height': self.height,
                    'length': 0.1,
                },
                'low': {
                    'position' : [0.0, -16.0/2, 0.4],
                    'width': 16,
                    'height': 0.8,
                    'length': 0.1,
                },
            },
        ]
        self.boundary_walls = [BoxObstacle(name=f"wall_{i}", content_dict=obst_dict) for i, obst_dict in enumerate(boundary_obs_dicts)]
    
    def Gen_Rooms(self):
        room1_obs_dicts = [
            {
                'type': 'box', 
                "rgba": [0.0, 0.0, 1.0, 1.0],
                'geometry': {
                    'position': [self.boundary_size - self.room_size, 0.0 - (self.room_size-self.door_size)/2 +1/2, 0.4],
                    'width': self.room_size-self.door_size, 'height': self.height, 'length': 0.1
                },
                'high': {
                    'position' : [self.boundary_size - self.room_size+1/2, 0.0 - (self.room_size-self.door_size)/2 +1/2, 0.4],
                    'width': self.room_size-self.door_size, 'height': self.height, 'length': 0.1,
                },
                'low': {
                    'position' : [self.boundary_size - self.room_size+1/2, 0.0 - (self.room_size-self.door_size)/2 +1/2, 0.4],
                    'width': self.room_size-self.door_size, 'height': self.height, 'length': 0.1,
                },
            },
            {
                'type': 'box', 
                "rgba": [0.0, 0.0, 1.0, 1.0],
                'geometry': {
                    'position': [self.boundary_size - self.room_size/2 +2/2, 0.0 - self.room_size+self.door_size+2/2, 0.4],
                    'width': 0.1, 'height': self.height, 'length': self.room_size-4/2
                },
                'high': {
                    'position' : [self.boundary_size - self.room_size/2 +2/2, 0.0 - self.room_size+self.door_size+2/2, 0.4],
                    'width': 0.1, 'height': self.height, 'length': self.room_size-4/2,
                },
                'low': {
                    'position' : [self.boundary_size - self.room_size/2 +2/2, 0.0 - self.room_size+self.door_size+2/2, 0.4],
                    'width': 0.1, 'height': self.height, 'length': self.room_size-4/2,
                },
            },
        ]
        self.room1_walls = [BoxObstacle(name=f"wall_{i}", content_dict=obst_dict) for i, obst_dict in enumerate(room1_obs_dicts)]
        
        room2_obs_dicts = [
            {
                'type': 'box', 
                "rgba": [1.0, 0.0, 0.0, 1.0],
                'geometry': {
                    'position': [(self.room_size), -(16 - (self.room_size-self.door_size)/2)-1/2, 0.4],
                    'width': self.room_size-self.door_size, 'height': self.height, 'length': 0.1
                },
                'high': {
                    'position' : [(self.room_size), -(16 - (self.room_size-self.door_size)/2)-1/2, 0.4],
                    'width': self.room_size-self.door_size, 'height': self.height, 'length': 0.1,
                },
                'low': {
                    'position' : [(self.room_size), -(16 - (self.room_size-self.door_size)/2)-1/2, 0.4],
                    'width': self.room_size-self.door_size, 'height': self.height, 'length': 0.1,
                },
            },
            {
                'type': 'box', 
                "rgba": [1.0, 0.0, 0.0, 1.0],
                'geometry': {
                    'position': [self.room_size/2-2/2, -(16 - self.room_size + self.door_size)-2/2, 0.4],
                    'width': 0.1, 'height': self.height, 'length': self.room_size-4/2
                },
                'high': {
                    'position' : [self.room_size/2-2/2, -(16 - self.room_size + self.door_size)-2/2, 0.4],
                    'width': 0.1, 'height': self.height, 'length': self.room_size-4/2,
                },
                'low': {
                    'position' : [self.room_size/2-2/2, -(16 - self.room_size + self.door_size)-2/2, 0.4],
                    'width': 0.1, 'height': self.height, 'length': self.room_size-4/2,
                },
            },
        ]
        self.room2_walls = [BoxObstacle(name=f"wall_{i}", content_dict=obst_dict) for i, obst_dict in enumerate(room2_obs_dicts)]
        
    def Gen_Room_Obstacles(self):
    
        self.obstacle_room1 = BoxObstacle(name='obstacle1', content_dict=(
                {
                'type': 'box', 
                "rgba": [0.0, 1.0, 0.0, 1.0],
                'geometry': {
                    'position': [5.0, -5.0 , 0.4], 
                    'width': self.obstacle_size, 'height': self.obstacle_size, 'length': self.obstacle_size
                },
                'high': {
                    'position' : [5.0, -5.0 , 0.4],
                    'width': self.obstacle_size, 'height': self.obstacle_size, 'length': self.obstacle_size
                },
                'low': {
                    'position' : [5.0, -5.0 , 0.4],
                    'width': self.obstacle_size, 'height': self.obstacle_size, 'length': self.obstacle_size
                },
            }))
        
        self.obstacle_room2 = BoxObstacle(name='obstacle1', content_dict=(
                {
                'type': 'box', 
                "rgba": [0.0, 1.0, 0.0, 1.0],
                'geometry': {
                    'position': [17.0, -8.0, 0.4],
                    'width': self.obstacle_size, 'height': self.obstacle_size, 'length': self.obstacle_size
                },
                'high': {
                    'position' : [17.0, -8.0, 0.4],
                    'width': self.obstacle_size, 'height': self.obstacle_size, 'length': self.obstacle_size
                },
                'low': {
                    'position' : [17.0, -8.0, 0.4],
                    'width': self.obstacle_size, 'height': self.obstacle_size, 'length': self.obstacle_size
                },
            }))
        obst2Dict = {
        "type": "sphere",
        "rgba": [0.0, 1.0, 0.0, 1.0],
        "movable": False,
        "geometry": {"position": [0.0, 0.0, 0.35], "radius": 0.7},
        }
        self.sphereObst1 = SphereObstacle(name="sphere_2", content_dict=obst2Dict)

    def Gen_Shelf(self):
        self.shelf1 = BoxObstacle(name='obstacle1', content_dict=(
                {
                'type': 'box', 
                "rgba": [0.5, 0.5, 0.0, 1.0],
                'geometry': {
                    'position': [self.boundary_size/2 -2.0, -4.0, 0.4], 
                    'width':  4, 'height': self.height, 'length': self.obstacle_size+0.5
                },
                'high': {
                    'position' :  [self.boundary_size/2-2.0, -4.0, 0.4], 
                    'width': 4 , 'height': self.height, 'length': self.obstacle_size+0.5
                },
                'low': {
                    'position' :  [self.boundary_size/2-2.0, -4.0, 0.4], 
                    'width': 4 , 'height': self.height, 'length': self.obstacle_size+0.5
                },
            }))
        
        self.shelf2 = BoxObstacle(name='obstacle1', content_dict=(
                {
                'type': 'box', 
                "rgba": [0.5, 0.5, 0.0, 1.0],
                'geometry': {
                    'position': [self.boundary_size/2, -4.0, 0.4], 
                    'width':  4, 'height': self.height, 'length': self.obstacle_size+0.5
                },
                'high': {
                    'position' :  [self.boundary_size/2, -4.0, 0.4], 
                    'width': 4 , 'height': self.height, 'length': self.obstacle_size+0.5
                },
                'low': {
                    'position' :  [self.boundary_size/2, -4.0, 0.4], 
                    'width': 4 , 'height': self.height, 'length': self.obstacle_size+0.5
                },
            }))
        
        self.shelf3 = BoxObstacle(name='obstacle1', content_dict=(
                {
                'type': 'box', 
                "rgba": [0.5, 0.5, 0.0, 1.0],
                'geometry': {
                    'position': [self.boundary_size/2 +2.0, -4.0, 0.4], 
                    'width':  4, 'height': self.height, 'length': self.obstacle_size+0.5
                },
                'high': {
                    'position' :  [self.boundary_size/2+2.0, -4.0, 0.4], 
                    'width': 4 , 'height': self.height, 'length': self.obstacle_size+0.5
                },
                'low': {
                    'position' :  [self.boundary_size/2+2.0, 4.0, 0.4], 
                    'width': 4 , 'height': self.height, 'length': self.obstacle_size+0.5
                },
            }))
        
        self.shelf4 = BoxObstacle(name='obstacle1', content_dict=(
                {
                'type': 'box', 
                "rgba": [0.5, 0.5, 0.0, 1.0],
                'geometry': {
                    'position': [self.boundary_size/2 -2.0, -16/2-4.0, 0.4], 
                    'width':  4, 'height': self.height, 'length': self.obstacle_size+0.5
                },
                'high': {
                    'position' :  [self.boundary_size/2-2.0, -16/2-4.0, 0.4], 
                    'width': 4 , 'height': self.height, 'length': self.obstacle_size+0.5
                },
                'low': {
                    'position' :  [self.boundary_size/2-2.0, -16/2-4.0, 0.4], 
                    'width': 4 , 'height': self.height, 'length': self.obstacle_size+0.5
                },
            }))
        
        self.shelf5 = BoxObstacle(name='obstacle1', content_dict=(
                {
                'type': 'box', 
                "rgba": [0.5, 0.5, 0.0, 1.0],
                'geometry': {
                    'position': [self.boundary_size/2, -16/2-4.0, 0.4], 
                    'width':  4, 'height': self.height, 'length': self.obstacle_size+0.5
                },
                'high': {
                    'position' :  [self.boundary_size/2, -16/2-4.0, 0.4], 
                    'width': 4 , 'height': self.height, 'length': self.obstacle_size+0.5
                },
                'low': {
                    'position' :  [self.boundary_size/2, -16/2-4.0, 0.4], 
                    'width': 4 , 'height': self.height, 'length': self.obstacle_size+0.5
                },
            }))
        
        self.shelf6 = BoxObstacle(name='obstacle1', content_dict=(
                {
                'type': 'box', 
                "rgba": [0.5, 0.5, 0.0, 1.0],
                'geometry': {
                    'position': [self.boundary_size/2 +2.0, -16/2-4.0, 0.4], 
                    'width':  4, 'height': self.height, 'length': self.obstacle_size+0.5
                },
                'high': {
                    'position' :  [self.boundary_size/2+2.0, -16/2-4.0, 0.4], 
                    'width': 4 , 'height': self.height, 'length': self.obstacle_size+0.5
                },
                'low': {
                    'position' :  [self.boundary_size/2+2.0, -16/2-4.0, 0.4], 
                    'width': 4 , 'height': self.height, 'length': self.obstacle_size+0.5
                },
            }))
    
    def Get_Ob(self):
        # Get Current position
        return self.ob

    def move_panda(self, action):
        # Move Robot
        self.ob, *_ = self.env.step(action)