import warnings
import gymnasium as gym
import numpy as np

from urdfenvs.robots.generic_urdf import GenericUrdfReacher
from urdfenvs.urdf_common.urdf_env import UrdfEnv
from mpscenes.obstacles.box_obstacle import BoxObstacle
from mpscenes.obstacles.sphere_obstacle import SphereObstacle

from urdfenvs.sensors.occupancy_sensor import OccupancySensor
import json
import os
from ament_index_python.packages import get_package_share_directory

class Panda_Sym:
    def __init__(self, render=True):
        
        self.render = render
        self.init_pos = np.array([0,0,0])
        self.env: UrdfEnv = None
        self.map_number = np.random.randint(1, 5)

        self.Gen_Env()
        self.run_point_robot_with_occupancy_sensor()
    
    def __del__(self):
        if self.env is not None:
            self.env.close()

    def get_index_from_coordinates(self,point, mesh) -> tuple:
        distances = np.linalg.norm(mesh - point, axis=3)
        return np.unravel_index(np.argmin(distances), mesh.shape[:-1])

    def evaluate_occupancy(self, point, mesh, occupancy, resolution) -> int:
        index = list(self.get_index_from_coordinates(point, mesh))
        return occupancy[tuple(index)]

    def run_point_robot_with_occupancy_sensor(self, n_steps=10):
        
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
        for _ in range(n_steps):
            action[:3] = defaultAction
            self.ob, *_ = self.env.step(action)
            point = np.append(self.ob['robot_0']['joint_state']['position'][0:2], 0.0)
            occupancy = self.ob['robot_0']['Occupancy']
            occupancy_eval = self.evaluate_occupancy(point, sensor.mesh(), occupancy, [0.2, 0.2, 1])

    def Gen_Env(self):
        self.Get_Map_Info()
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
        self.env.add_obstacle(self.shelf1)
        self.env.add_obstacle(self.shelf2)
        self.env.add_obstacle(self.shelf3)
        self.env.add_obstacle(self.shelf4)
        if(self.map_number > 2):
            self.env.add_obstacle(self.shelf5)
            self.env.add_obstacle(self.shelf6)
        self.ob = self.env.reset(mount_positions=np.array([self.init_pos]))

    def Get_Map_Info(self):

        json_path = os.path.join(os.path.dirname(get_package_share_directory('sim_env')), 'sim_env', 'resource', f"map{self.map_number}.json")
        with open(json_path, 'r') as file:
            map_data = json.load(file)

        # Extract information into variables
        self.init_pos = map_data.get('panda_init_pos', [])
        self.boundary_obs_dicts = map_data.get('boundary_obs_dicts', [])
        self.blue_room_dicts = map_data.get('blue_room_dicts', [])
        self.red_room_dicts = map_data.get('red_room_dicts', [])
        self.obstacles = map_data.get('obstacles', [])
        self.shelves = map_data.get('shelves', [])

    def Gen_Panda(self):
        self.panda_robot = [
            GenericUrdfReacher(urdf="mobilePanda_with_gripper.urdf", mode="vel"),
        ]
    
    def Gen_Boundary(self):
        self.boundary_walls = [BoxObstacle(name=f"wall_{i}", content_dict=obst_dict) for i, obst_dict in enumerate(self.boundary_obs_dicts)]
    
    def Gen_Rooms(self):

        #BLUE ROOM
        self.room1_walls = [BoxObstacle(name=f"wall_{i}", content_dict=obst_dict) for i, obst_dict in enumerate(self.blue_room_dicts)]
        
        #RED ROOM
        self.room2_walls = [BoxObstacle(name=f"wall_{i}", content_dict=obst_dict) for i, obst_dict in enumerate(self.red_room_dicts)]
        
    def Gen_Room_Obstacles(self):
    
        self.obstacle_room1 = BoxObstacle(name='obstacle1', content_dict=(self.obstacles[0]))
        
        self.obstacle_room2 = BoxObstacle(name='obstacle1', content_dict=(self.obstacles[1]))

    def Gen_Shelf(self):
        self.shelf1 = BoxObstacle(name='obstacle1', content_dict=(self.shelves[0]))
        
        self.shelf2 = BoxObstacle(name='obstacle1', content_dict=(self.shelves[1]))
        
        self.shelf3 = BoxObstacle(name='obstacle1', content_dict=(self.shelves[2]))
        
        self.shelf4 = BoxObstacle(name='obstacle1', content_dict=(self.shelves[3]))

        if(self.map_number > 2):
            self.shelf5 = BoxObstacle(name='obstacle1', content_dict=(self.shelves[4]))
            
            self.shelf6 = BoxObstacle(name='obstacle1', content_dict=(self.shelves[5]))
    
    def Get_Ob(self):
        # Get Current position
        return self.ob

    def move_panda(self, action):
        # Move Robot
        self.ob, *_ = self.env.step(action)