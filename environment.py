import numpy as np

from urdfenvs.robots.generic_urdf import GenericUrdfReacher
from urdfenvs.urdf_common.urdf_env import UrdfEnv
from mpscenes.obstacles.sphere_obstacle import SphereObstacle
from mpscenes.obstacles.box_obstacle import BoxObstacle
import pybullet as p
import cv2
import matplotlib.pyplot as plt

wall_length = 10

def generate_occupancy_grid(image, grid_resolution=0.1):
    """
    Generates an occupancy grid from an overhead image.

    Parameters:
        image (np.ndarray): Overhead RGB image of the environment.
        grid_resolution (float): Resolution of the grid in meters per cell.

    Returns:
        np.ndarray: Occupancy grid with values (-1: unknown, 0: free, 100: occupied).
    """
    # Convert the image to grayscale
    # gray_image = cv2.cvtColor(image, cv2.COLOR_RGB2GRAY)
    print(image[500,500])
    print(image.shape)
    # Blanco 255 x3
    # Azul [206 213 223]
    plt.imshow(image)
    plt.show()

    # Threshold the image to classify areas
    # _, obstacle_mask = cv2.threshold(gray_image, 100, 255, cv2.THRESH_BINARY)

    # Determine grid size based on image dimensions and resolution
    grid_width = image.shape[1]
    grid_height= image.shape[0]
    occupancy_grid = np.full((grid_height, grid_width), -1, dtype=int)  # Initialize as unknown (-1)

    # Map obstacle mask to grid values
    for i in range(grid_height):
        for j in range(grid_width):
            if (np.array_equal(image[i, j], np.array([255, 255, 255])) 
                or np.array_equal(image[i, j], np.array([206, 213, 223])) 
                or np.array_equal(image[i, j], np.array([277, 277, 277]))
                or np.array_equal(image[i, j], np.array([154, 177, 213]))):
                # Black -> obstacle
                occupancy_grid[i, j] = 0  # Free
            else:  # White -> free space
                occupancy_grid[i, j] = 100  # Ocupied
    print(occupancy_grid.shape)
    return occupancy_grid


def visualize_occupancy_grid(occupancy_grid):
    """
    Visualizes the occupancy grid using matplotlib.

    Parameters:
        occupancy_grid (np.ndarray): Occupancy grid to visualize.
    """
    plt.imshow(occupancy_grid, cmap="gray_r", vmin=0, vmax=100)
    plt.colorbar(label="Occupancy")
    plt.title("Occupancy Grid")
    plt.show()


def capture_overhead_image(
    environment,
    width: int = 640,
    height: int = 480,
    fov: float = 60,
    near_val: float = 0.1,
    far_val: float = 100.0,
    camera_position: tuple = (0, 0, 10),
    target_position: tuple = (0, 0, 0),
    up_vector: tuple = (0, 1, 0),
) -> np.ndarray:
    """
    Captures an overhead image of the environment.

    Parameters:
        width (int): Width of the image in pixels.
        height (int): Height of the image in pixels.
        fov (float): Field of view of the camera in degrees.
        near_val (float): Near clipping plane distance.
        far_val (float): Far clipping plane distance.
        camera_position (tuple): Position of the camera.
        target_position (tuple): Point the camera is looking at.
        up_vector (tuple): Up vector for the camera orientation.

    Returns:
        np.ndarray: RGB image array.
    """
    view_matrix = p.computeViewMatrix(
        cameraEyePosition=camera_position,
        cameraTargetPosition=target_position,
        cameraUpVector=up_vector,
    )
    projection_matrix =p.computeProjectionMatrixFOV(
        fov=fov,
        aspect=float(width) / height,
        nearVal=near_val,
        farVal=far_val,
    )
    _, _, rgb_image, _, _ = p.getCameraImage(
        width=width,
        height=height,
        viewMatrix=view_matrix,
        projectionMatrix=projection_matrix,
        renderer=p.ER_BULLET_HARDWARE_OPENGL,
    )
    return np.reshape(rgb_image, (height, width, 4))[:, :, :3]


wall_obstacles_dicts = [
    {
        'type': 'box', 
         'geometry': {
             'position': [wall_length/2.0, 0.0, 0.4], 'width': wall_length, 'height': 0.8, 'length': 0.1
        },
        'high': {
            'position' : [wall_length/2.0, 0.0, 0.4],
            'width': wall_length,
            'height': 0.8,
            'length': 0.1,
        },
        'low': {
            'position' : [wall_length/2.0, 0.0, 0.4],
            'width': wall_length,
            'height': 0.8,
            'length': 0.1,
        },
    },
    {
        'type': 'box', 
         'geometry': {
             'position': [0.0, wall_length/2.0, 0.4], 'width': 0.1, 'height': 0.8, 'length': wall_length
        },
        'high': {
            'position' : [0.0, wall_length/2.0, 0.4],
            'width': 0.1,
            'height': 0.8,
            'length': wall_length,
        },
        'low': {
            'position' : [0.0, wall_length/2.0, 0.4],
            'width': 0.1,
            'height': 0.8,
            'length': wall_length,
        },
    },
    {
        'type': 'box', 
         'geometry': {
             'position': [0.0, -wall_length/2.0, 0.4], 'width': 0.1, 'height': 0.8, 'length': wall_length
        },
        'high': {
            'position' : [0.0, -wall_length/2.0, 0.4],
            'width': 0.1,
            'height': 0.8,
            'length': wall_length,
        },
        'low': {
            'position' : [0.0, -wall_length/2.0, 0.4],
            'width': 0.1,
            'height': 0.8,
            'length': wall_length,
        },
    },
    {
        'type': 'box', 
         'geometry': {
             'position': [-wall_length/2.0, 0.0, 0.4], 'width': wall_length, 'height': 0.8, 'length': 0.1
        },
        'high': {
            'position' : [-wall_length/2.0, 0.0, 0.4],
            'width': wall_length,
            'height': 0.8,
            'length': 0.1,
        },
        'low': {
            'position' : [-wall_length/2.0, 0.0, 0.4],
            'width': wall_length,
            'height': 0.8,
            'length': 0.1,
        },
    },
]

wall_obstacles = [BoxObstacle(name=f"wall_{i}", content_dict=obst_dict) for i, obst_dict in enumerate(wall_obstacles_dicts)]

def run_mobile_reacher(n_steps=10000, render=False, goal=True, obstacles=True):
    robots = [
         GenericUrdfReacher(urdf="mobilePanda_with_gripper.urdf", mode="vel"),
    ]
    
    
    env: UrdfEnv = UrdfEnv(
        dt=0.01,
          robots=robots, 
          render=render, num_sub_steps=200,
    )
    

    shelf2 = BoxObstacle(name='obstacle1', content_dict=(
                {
                'type': 'box', 
                "rgba": [0.5, 0.5, 0.0, 1.0],
                "movable": False,
                'geometry': {
                    'position': [2, 0.15, 0.15], 
                    'width': 1, 'height': 0.3, 'length': 1
                },
                'high': {
                    'position': [2, 0.15, 0.15], 
                    'width': 1, 'height': 0.3, 'length': 1
                },
                'low': {
                    'position': [2, 0.15, 0.15], 
                    'width': 1, 'height': 0.3, 'length': 1
                },
            }))
    
    env.add_obstacle(shelf2)
    for i in range(len(wall_obstacles)):
        env.add_obstacle(wall_obstacles[i])
    

 

    action = np.zeros(env.n())
    
    ob = env.reset()

    print(f"Initial observation : {ob}")


    history = []
    ob, *_ = env.step(action)
    # obstacles=env.get_obstacles()
    
    image = capture_overhead_image(
        env,
    width=800,
    height=600,
    camera_position=(0, 0, 20),
    target_position=(0, 0, 0)
    )

    # Visualize or save the image using matplotlib or OpenCV
    # plt.savefig("synthetic_environment.png")
    occupancy_grid = generate_occupancy_grid(image)

    # Visualize the occupancy grid
    visualize_occupancy_grid(occupancy_grid)
        
    env.close()
    return history


if __name__ == "__main__":
    run_mobile_reacher(render=True)

    

