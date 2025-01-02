import warnings
import gymnasium as gym
import numpy as np
from urdfenvs.robots.generic_urdf import GenericUrdfReacher
from urdfenvs.urdf_common.urdf_env import UrdfEnv
from mpscenes.obstacles.box_obstacle import BoxObstacle
from multiprocessing import Process, Pipe
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
        return self.ob
    
    def move(self):

        self.ob = self.env.reset(
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
            self.ob, *_ = self.env.step(action)
            history.append(self.ob)
        self.env.close()
        return history

    def move_panda(self, action):
        self.ob, *_ = self.env.step(action)

    # def move_panda(self, n_steps=10000, render=False, goal=True, obstacles=True):

    #     action = np.zeros(self.env.n())
    #     # action[0] = 0.1
    #     # action[1] = 0.1
    #     # action[3] = 1
    #     # action[1] = 0.1
    #     # action[arm_joints[7]] = 1
    #     # action[arm_joints[5]]=0
    #     ob = self.env.reset()
    #     print(f"Initial observation : {ob}")
    #     urdf_path="/home/jose/anaconda3/envs/PDM/lib/python3.10/site-packages/robotmodels/mobilePanda/urdf/mobilePanda_with_gripper.urdf"
    #     robot = URDF.load(urdf_path)
        
    #     range_joints=range(len(robot.actuated_joints)-3)
    #     joints=[]
    #     print(robot.actuated_joints[0])
    #     for x in range_joints:
    #         print("X.",x)
    #         joint=robot.actuated_joints[x+3]
    #         joints.append(joint)
    #         # joint_type = joints[x].joint_type
    #         # joint_limits = joints[x].limit
    #         # print(f"{joints[x].name:<20} {joint_type:<15} {x:<10}")
    #         # print(f"Lower Limit: {joint_limits.lower:<15} Upper Limit: {joint_limits.upper:<10}")
    #         # print(np.round(joints[x].origin,5))

    #     ob, *_ = self.env.step(action) 
    #     print(np.round(ob['robot_0']['joint_state']['position'],2))
    #     print(np.round(ob['robot_0']['joint_state']['position'][3:-2],4))
    #     current_xyz=compute_forward_kinematics(joints, np.round(ob['robot_0']['joint_state']['position'][3:-2],4))
    #     print("End Position")
    #     print(np.round(current_xyz))
    #     ja=compute_jacobian(joints, np.round(ob['robot_0']['joint_state']['position'][3:-2],4), current_xyz)
    #     print("Jacobian")
    #     print(ja)
    #     robot.show()
    #     history = []
    #     target_xyz = np.array([0.8, 0, 0.5])
    #     max_velocity = 0.5
    #     # print(f"Intial velocity: {ob['robot_0']['joint_state']['velocity']}")
    #     for i in range(n_steps):
    #         # if (int(i / 100)) % 2 == 0:
    #         #     action[11] = -0.01
    #         #     action[10] = -0.01
    #         # else:
    #         #     action[11] = 0.01
    #         #     action[10] = 0.01
    #         if (np.linalg.norm(target_xyz - current_xyz) > 0.01):  # Loop until close to target
    #             # Step 1: Compute desired Cartesian velocity (proportional control for simplicity)
    #             error_xyz = target_xyz - current_xyz
    #             desired_velocity_xyz = 1.0 * error_xyz  # Proportional gain (1.0)

    #             if np.linalg.norm(desired_velocity_xyz) > max_velocity:
    #                 desired_velocity_xyz = desired_velocity_xyz / np.linalg.norm(desired_velocity_xyz) * max_velocity
    #             desired_velocity = np.hstack((desired_velocity_xyz, np.zeros(3))) 
    #             # Step 2: Compute Jacobian at current joint positions
    #             J= compute_jacobian(joints, np.round(ob['robot_0']['joint_state']['position'][3:-2],4), current_xyz)

    #             # Step 3: Compute joint velocities
    #             print("Jacobian")
    #             print(J)
    #             print("Error")
    #             print(desired_velocity)
    #             joint_velocities = np.linalg.pinv(J) @ desired_velocity # Use pseudoinverse to solve

    #             # Step 4: Send joint velocities to the robot
    #             for x in range(len(joints)-2):
    #                 # print("X.",x)
    #                 action[x+3]=joint_velocities[x]
                    

    #             # Step 5: Update current joint positions and end-effector position
                
                
    #         else:
    #             for x in range(len(joints)-2):
    #                 # print("X.",x)
    #                 action[x+3]=0

    #         ob, *_ = env.step(action) 
    #         current_xyz = compute_forward_kinematics(joints, np.round(ob['robot_0']['joint_state']['position'][3:-2],4))  # FK to get xyz    
    #         # for x in range(len(action)):
    #         #     if (np.abs(np.round(ob['robot_0']['joint_state']['velocity'][x],4)) > 0.1):
    #         #         action[x]=-ob['robot_0']['joint_state']['velocity'][x]
    #         #     else:
    #         #         action[x]=0
    #         # print(f"Velocity: {np.round(ob['robot_0']['joint_state']['position'],2)}")
    #         # if (ob['robot_0']['joint_state']['velocity'][arm_joints[0]]>0.5):
    #         #     action[arm_joints[0]] = 0.0
    #         history.append(ob)
    #     env.close()

        

        # return history