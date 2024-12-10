import warnings
import gymnasium as gym
import numpy as np
from urdfenvs.robots.generic_urdf.generic_diff_drive_robot import GenericDiffDriveRobot
from urdfenvs.urdf_common.urdf_env import UrdfEnv
from mpscenes.obstacles.box_obstacle import BoxObstacle
from multiprocessing import Process, Pipe
from urdfenvs.keyboard_input.keyboard_input_responder import Responder
from pynput.keyboard import Key
from mpscenes.obstacles.sphere_obstacle import SphereObstacle

class Albert_sym:
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
        self.Gen_Albert(self.albert_pos)
        self.Gen_Boundary()
        self.Gen_Rooms()
        self.Gen_Room_Obstacles()
        self.Gen_Shelf()
        self.env = UrdfEnv(dt=0.01, robots=self.albert_robot, render=self.render)

        for wall in self.boundary_walls:
            self.env.add_obstacle(wall)
        for wall in self.room1_walls:
            self.env.add_obstacle(wall)
        for wall in self.room2_walls:
            self.env.add_obstacle(wall)
        self.env.add_obstacle(self.obstacle_room1)
        self.env.add_obstacle(self.sphereObst1)
        self.env.add_obstacle(self.shelf1)
        self.env.add_obstacle(self.shelf2)
        self.env.add_obstacle(self.shelf3)
        self.env.add_obstacle(self.shelf4)
        self.env.add_obstacle(self.shelf5)
        self.env.add_obstacle(self.shelf6)
        ob = self.env.reset(
            pos=np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0, 0.0, 0, 0]))

    def Gen_Albert(self, albert_pos):
        self.albert_robot = [GenericDiffDriveRobot(
                 urdf="albert.urdf",
                 mode="vel",
                 actuated_wheels=["wheel_right_joint", "wheel_left_joint"],
                 castor_wheels=["rotacastor_right_joint", "rotacastor_left_joint"],
                 wheel_radius = 0.08,
                 wheel_distance = 0.494,
                 spawn_offset = albert_pos,
                 spawn_rotation = 0,
                 facing_direction = '-y',
                 )]
    
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
        
    def move(self):
        ob = self.env.reset(
            pos=np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0, 0.0, 0, 0.0])
        )
        print(f"Initial observation : {ob}")
        history = []
        action = np.zeros(self.env.n())
        while(True):
            
            x = input()
            if (x == 'w'):
                action[0] += 1
            elif (x == 's'):
                action[0] += -1
            elif (x == 'd'):
                action[1] += 1 
            elif (x == 'a'):
                action[1] += -1
            elif (x == 'e'):
                action[2] = 1
            elif (x == 'r'):
                action[3] = 1
            elif (x == 't'):
                action[4] = 1
            elif (x == 'y'):
                action[5] = 1
            elif (x == 'u'):
                action[6] = 1
            elif (x == 'i'):
                action[7] = 1
            elif (x == 'o'):
                action[8] = 1
            elif (x == 'p'):
                action[9] = 1
            elif (x == 'q'):
                break
            else:
                print('continue')
            print(action)
            ob, *_ = self.env.step(action)
            history.append(ob)
        self.env.close()
        return history
        
if __name__ == "__main__":
    show_warnings = False
    warning_flag = "default" if show_warnings else "ignore"
    with warnings.catch_warnings():
        warnings.filterwarnings(warning_flag)
        albert_sym = Albert_sym(render=True)
        albert_sym.move()

