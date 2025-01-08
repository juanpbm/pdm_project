import warnings
import gymnasium as gym
import numpy as np
from urdfenvs.robots.generic_urdf import GenericUrdfReacher
from urdfenvs.urdf_common.urdf_env import UrdfEnv
from mpscenes.obstacles.box_obstacle import BoxObstacle
from mpscenes.obstacles.sphere_obstacle import SphereObstacle

class Panda_Sym:
    def __init__(self, render=True,
                 albert_pos=np.array([0.0, 0.0, 0.0]),
                 boundary_size=10, 
                 room_size = 3,
                 door_size = 1,
                 obstacle_size = 0.5):
        
        self.render = render
        self.boundary_size = boundary_size
        self.room_size = room_size
        self.door_size = door_size
        self.obstacle_size = obstacle_size
        self.height = 2
        self.albert_pos = [-(self.boundary_size/2 - self.room_size/2), -(self.boundary_size/2 - self.room_size/2), 0.0]
        self.env: UrdfEnv = None
        self.Gen_Env()
    
    def __del__(self):
        if self.env is not None:
            self.env.close()

    def Gen_Env(self):
        self.Gen_Panda()
        # self.Gen_Boundary()
        # self.Gen_Rooms()
        # self.Gen_Room_Obstacles()
        # self.Gen_Shelf()
        self.env = UrdfEnv(dt=0.01, robots=self.panda_robot, render=self.render, num_sub_steps=200,)

        # for wall in self.boundary_walls:
        #     self.env.add_obstacle(wall)
        # for wall in self.room1_walls:
        #     self.env.add_obstacle(wall)
        # for wall in self.room2_walls:
        #     self.env.add_obstacle(wall)
        # self.env.add_obstacle(self.obstacle_room1)
        # self.env.add_obstacle(self.sphereObst1)
        # self.env.add_obstacle(self.shelf1)
        # self.env.add_obstacle(self.shelf2)
        # self.env.add_obstacle(self.shelf3)
        # self.env.add_obstacle(self.shelf4)
        # self.env.add_obstacle(self.shelf5)
        # self.env.add_obstacle(self.shelf6)
        action = np.zeros(self.env.n())
        self.ob = self.env.reset()
        self.ob, *_ = self.env.step(action) 

    def Gen_Panda(self):
        self.panda_robot = [
            GenericUrdfReacher(urdf="mobilePanda_with_gripper.urdf", mode="vel"),
        ]
    
    def Gen_Boundary(self):

        boundary_obs_dicts = [
            {
                'type': 'box', 
                'geometry': {
                    'position': [self.boundary_size/2.0, 0.0, 0.4], 'width': self.boundary_size, 'height': self.height, 'length': 0.1
                },
                'high': {
                    'position' : [self.boundary_size/2.0, 0.0, 0.4],
                    'width': self.boundary_size,
                    'height': self.height,
                    'length': 0.1,
                },
                'low': {
                    'position' : [self.boundary_size/2.0, 0.0, 0.4],
                    'width': self.boundary_size,
                    'height': self.height,
                    'length': 0.1,
                },
            },
            {
                'type': 'box', 
                'geometry': {
                    'position': [0.0, self.boundary_size/2.0, 0.4], 'width': 0.1, 'height': self.height, 'length': self.boundary_size
                },
                'high': {
                    'position' : [0.0, self.boundary_size/2.0, 0.4],
                    'width': 0.1,
                    'height': self.height,
                    'length': self.boundary_size,
                },
                'low': {
                    'position' : [0.0, self.boundary_size/2.0, 0.4],
                    'width': 0.1,
                    'height': self.height,
                    'length': self.boundary_size,
                },
            },
            {
                'type': 'box', 
                'geometry': {
                    'position': [0.0, -self.boundary_size/2.0, 0.4], 'width': 0.1, 'height': self.height, 'length': self.boundary_size
                },
                'high': {
                    'position' : [0.0, -self.boundary_size/2.0, 0.4],
                    'width': 0.1,
                    'height': self.height,
                    'length': self.boundary_size,
                },
                'low': {
                    'position' : [0.0, -self.boundary_size/2.0, 0.4],
                    'width': 0.1,
                    'height': self.height,
                    'length': self.boundary_size,
                },
            },
            {
                'type': 'box', 
                'geometry': {
                    'position': [-self.boundary_size/2.0, 0.0, 0.4], 'width': self.boundary_size, 'height': self.height, 'length': 0.1
                },
                'high': {
                    'position' : [-self.boundary_size/2.0, 0.0, 0.4],
                    'width': self.boundary_size,
                    'height': self.height,
                    'length': 0.1,
                },
                'low': {
                    'position' : [-self.boundary_size/2.0, 0.0, 0.4],
                    'width': self.boundary_size,
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
                    'position': [self.boundary_size/2 - self.room_size, self.boundary_size/2 - (self.room_size-self.door_size)/2, 0.4],
                    'width': self.room_size-self.door_size, 'height': self.height, 'length': 0.1
                },
                'high': {
                    'position' : [self.boundary_size/2 - self.room_size, self.boundary_size/2 - (self.room_size-self.door_size)/2, 0.4],
                    'width': self.room_size-self.door_size, 'height': self.height, 'length': 0.1,
                },
                'low': {
                    'position' : [self.boundary_size/2 - self.room_size, self.boundary_size/2 - (self.room_size-self.door_size)/2, 0.4],
                    'width': self.room_size-self.door_size, 'height': self.height, 'length': 0.1,
                },
            },
            {
                'type': 'box', 
                "rgba": [0.0, 0.0, 1.0, 1.0],
                'geometry': {
                    'position': [self.boundary_size/2 - self.room_size/2, self.boundary_size/2 - self.room_size, 0.4], 'width': 0.1, 'height': self.height, 'length': self.room_size
                },
                'high': {
                    'position' : [self.boundary_size/2 - self.room_size/2, self.boundary_size/2 - self.room_size, 0.4],
                    'width': 0.1, 'height': self.height, 'length': self.room_size,
                },
                'low': {
                    'position' : [self.boundary_size/2 - self.room_size/2, self.boundary_size/2 - self.room_size, 0.4],
                    'width': 0.1, 'height': self.height, 'length': self.room_size,
                },
            },
        ]
        self.room1_walls = [BoxObstacle(name=f"wall_{i}", content_dict=obst_dict) for i, obst_dict in enumerate(room1_obs_dicts)]
        
        room2_obs_dicts = [
            {
                'type': 'box', 
                "rgba": [1.0, 0.0, 0.0, 1.0],
                'geometry': {
                    'position': [-(self.boundary_size/2 - self.room_size), -(self.boundary_size/2 - (self.room_size-self.door_size)/2), 0.4], 'width': self.room_size-self.door_size, 'height': self.height, 'length': 0.1
                },
                'high': {
                    'position' : [-(self.boundary_size/2 - self.room_size), -(self.boundary_size/2 - (self.room_size-self.door_size)/2), 0.4],
                    'width': self.room_size-self.door_size, 'height': self.height, 'length': 0.1,
                },
                'low': {
                    'position' : [-(self.boundary_size/2 - self.room_size), -(self.boundary_size/2 - (self.room_size-self.door_size)/2), 0.4],
                    'width': self.room_size-self.door_size, 'height': self.height, 'length': 0.1,
                },
            },
            {
                'type': 'box', 
                "rgba": [1.0, 0.0, 0.0, 1.0],
                'geometry': {
                    'position': [-(self.boundary_size/2 - (self.room_size)/2), -(self.boundary_size/2 - self.room_size), 0.4], 'width': 0.1, 'height': self.height, 'length': self.room_size
                },
                'high': {
                    'position' : [-(self.boundary_size/2 - (self.room_size)/2), -(self.boundary_size/2 - self.room_size), 0.4],
                    'width': 0.1, 'height': self.height, 'length': self.room_size,
                },
                'low': {
                    'position' : [-(self.boundary_size/2 - (self.room_size)/2), -(self.boundary_size/2 - self.room_size), 0.4],
                    'width': 0.1, 'height': self.height, 'length': self.room_size,
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
                    'position': [self.boundary_size/2 - self.room_size/2, self.boundary_size/2 - self.room_size/2, 0.4], 
                    'width': self.obstacle_size, 'height': self.obstacle_size, 'length': self.obstacle_size
                },
                'high': {
                    'position' : [self.boundary_size/2 - self.room_size/2, self.boundary_size/2 - self.room_size/2, 0.4],
                    'width': self.obstacle_size, 'height': self.obstacle_size, 'length': self.obstacle_size
                },
                'low': {
                    'position' : [self.boundary_size/2 - self.room_size/2, self.boundary_size/2 - self.room_size/2, 0.4],
                    'width': self.obstacle_size, 'height': self.obstacle_size, 'length': self.obstacle_size
                },
            }))
        
        self.obstacle_room2 = BoxObstacle(name='obstacle1', content_dict=(
                {
                'type': 'box', 
                "rgba": [0.0, 1.0, 0.0, 1.0],
                'geometry': {
                    'position': [-(self.boundary_size/2 - self.room_size/2), -(self.boundary_size/2 - self.room_size/2), 0.4],
                    'width': self.obstacle_size, 'height': self.obstacle_size, 'length': self.obstacle_size
                },
                'high': {
                    'position' : [-(self.boundary_size/2 - self.room_size/2), -(self.boundary_size/2 - self.room_size/2), 0.4],
                    'width': self.obstacle_size, 'height': self.obstacle_size, 'length': self.obstacle_size
                },
                'low': {
                    'position' : [-(self.boundary_size/2 - self.room_size/2), -(self.boundary_size/2 - self.room_size/2), 0.4],
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
                    'position': [-((self.boundary_size/2) - 3), ((self.boundary_size/2) - 3), 0.4], 
                    'width': 3 , 'height': self.height, 'length': self.obstacle_size
                },
                'high': {
                    'position' :  [-((self.boundary_size/2) - 3), ((self.boundary_size/2) - 3), 0.4], 
                    'width': 3 , 'height': self.height, 'length': self.obstacle_size
                },
                'low': {
                    'position' :  [-((self.boundary_size/2) - 3), ((self.boundary_size/2) - 3), 0.4], 
                    'width': 3 , 'height': self.height, 'length': self.obstacle_size
                },
            }))
        
        self.shelf2 = BoxObstacle(name='obstacle1', content_dict=(
                {
                'type': 'box', 
                "rgba": [0.5, 0.5, 0.0, 1.0],
                'geometry': {
                    'position': [-((self.boundary_size/2) - 2), ((self.boundary_size/2) - 3), 0.4], 
                    'width': 3 , 'height': self.height, 'length': self.obstacle_size
                },
                'high': {
                    'position' :  [-((self.boundary_size/2) - 2), ((self.boundary_size/2) - 3), 0.4], 
                    'width': 3 , 'height': self.height, 'length': self.obstacle_size
                },
                'low': {
                    'position' :  [-((self.boundary_size/2) - 2), ((self.boundary_size/2) - 3), 0.4], 
                    'width': 3 , 'height': self.height, 'length': self.obstacle_size
                },
            }))
        
        self.shelf3 = BoxObstacle(name='obstacle1', content_dict=(
                {
                'type': 'box', 
                "rgba": [0.5, 0.5, 0.0, 1.0],
                'geometry': {
                    'position': [-((self.boundary_size/2) - 1), ((self.boundary_size/2) - 3), 0.4], 
                    'width': 3 , 'height': self.height, 'length': self.obstacle_size
                },
                'high': {
                    'position' :  [-((self.boundary_size/2) - 1), ((self.boundary_size/2) - 3), 0.4], 
                    'width': 3 , 'height': self.height, 'length': self.obstacle_size
                },
                'low': {
                    'position' :  [-((self.boundary_size/2) - 1), ((self.boundary_size/2) - 3), 0.4], 
                    'width': 3 , 'height': self.height, 'length': self.obstacle_size
                },
            }))
        
        self.shelf4 = BoxObstacle(name='obstacle1', content_dict=(
                {
                'type': 'box', 
                "rgba": [0.5, 0.5, 0.0, 1.0],
                'geometry': {
                    'position': [((self.boundary_size/2) - 3), -((self.boundary_size/2) - 3), 0.4], 
                    'width': 3 , 'height': self.height, 'length': self.obstacle_size
                },
                'high': {
                    'position' :  [((self.boundary_size/2) - 3), -((self.boundary_size/2) - 3), 0.4], 
                    'width': 3 , 'height': self.height, 'length': self.obstacle_size
                },
                'low': {
                    'position' :  [((self.boundary_size/2) - 3), -((self.boundary_size/2) - 3), 0.4], 
                    'width': 3 , 'height': self.height, 'length': self.obstacle_size
                },
            }))
        
        self.shelf5 = BoxObstacle(name='obstacle1', content_dict=(
                {
                'type': 'box', 
                "rgba": [0.5, 0.5, 0.0, 1.0],
                'geometry': {
                    'position': [((self.boundary_size/2) - 2), -((self.boundary_size/2) - 3), 0.4], 
                    'width': 3 , 'height': self.height, 'length': self.obstacle_size
                },
                'high': {
                    'position' :  [((self.boundary_size/2) - 2), -((self.boundary_size/2) - 3), 0.4], 
                    'width': 3 , 'height': self.height, 'length': self.obstacle_size
                },
                'low': {
                    'position' :  [((self.boundary_size/2) - 2), -((self.boundary_size/2) - 3), 0.4], 
                    'width': 3 , 'height': self.height, 'length': self.obstacle_size
                },
            }))
        
        self.shelf6 = BoxObstacle(name='obstacle1', content_dict=(
                {
                'type': 'box', 
                "rgba": [0.5, 0.5, 0.0, 1.0],
                'geometry': {
                    'position': [((self.boundary_size/2) - 1), -((self.boundary_size/2) - 3), 0.4], 
                    'width': 3 , 'height': self.height, 'length': self.obstacle_size
                },
                'high': {
                    'position' :  [((self.boundary_size/2) - 1), -((self.boundary_size/2) - 3), 0.4], 
                    'width': 3 , 'height': self.height, 'length': self.obstacle_size
                },
                'low': {
                    'position' :  [((self.boundary_size/2) - 1), -((self.boundary_size/2) - 3), 0.4], 
                    'width': 3 , 'height': self.height, 'length': self.obstacle_size
                },
            }))
    
    def Get_Ob(self):
        # Get Current position
        return self.ob

    def move_panda(self, action):
        # Move Robot
        self.ob, *_ = self.env.step(action)