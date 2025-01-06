import numpy as np
import matplotlib.pyplot as plt
import time
from urdfenvs.robots.generic_urdf import GenericUrdfReacher
from urdfenvs.urdf_common.urdf_env import UrdfEnv

velocity_limit=2.5
"This python code should receive all the waypoints necessary to compute the trajectory to this variable"
target_xyz = np.array([[1, 1, 0],[2,2,0],[0,2,0],[0,0,0]])

# NEW NEW NEW

def trajectory_planning(via_points,V_max, desired_dt):
    # Extract x and y coordinates of via-points
    x = via_points[:, 0]
    y = via_points[:, 1]
    n = len(x)

    # Calculate distances between via-points
    distances = np.sqrt(np.diff(x)**2 + np.diff(y)**2)

    # Calculate minimum time intervals based on maximum velocity
    min_time_intervals = distances / V_max

    # Ensure cumulative time intervals
    times = np.zeros(n)
    times[1:] = np.cumsum(min_time_intervals)

    # Step lengths in time
    h = np.diff(times)

    # Solve for the coefficients of the cubic spline
    A = np.zeros((n, n))
    bx = np.zeros(n)
    by = np.zeros(n)

    # Natural spline conditions
    A[0, 0] = 1
    A[-1, -1] = 1

    # Setting up the system of equations for x and y coordinates
    for i in range(1, n-1):
        A[i, i-1] = h[i-1]
        A[i, i] = 2 * (h[i-1] + h[i])
        A[i, i+1] = h[i]
        bx[i] = 3 * ((x[i+1] - x[i]) / h[i] - (x[i] - x[i-1]) / h[i-1])
        by[i] = 3 * ((y[i+1] - y[i]) / h[i] - (y[i] - y[i-1]) / h[i-1])

    # Solve for the second derivatives
    second_derivatives_x = np.linalg.solve(A, bx)
    second_derivatives_y = np.linalg.solve(A, by)

    # Calculate the spline coefficients for x and y
    splines_x = []
    splines_y = []
    for i in range(n-1):
        ax = (second_derivatives_x[i+1] - second_derivatives_x[i]) / (6 * h[i])
        bx = second_derivatives_x[i] / 2
        cx = (x[i+1] - x[i]) / h[i] - (h[i] * (2 * second_derivatives_x[i] + second_derivatives_x[i+1])) / 6
        dx = x[i]

        ay = (second_derivatives_y[i+1] - second_derivatives_y[i]) / (6 * h[i])
        by = second_derivatives_y[i] / 2
        cy = (y[i+1] - y[i]) / h[i] - (h[i] * (2 * second_derivatives_y[i] + second_derivatives_y[i+1])) / 6
        dy = y[i]

        splines_x.append((ax, bx, cx, dx))
        splines_y.append((ay, by, cy, dy))

    # Generate points along the spline
    total_time = times[-1] 
    time_new =np.arange(0,total_time, desired_dt)
    x_new = np.zeros_like(time_new)
    y_new = np.zeros_like(time_new)

    coordinates_trajectory=[]
    for j in range(len(time_new)):
        for i in range(n-1):
            if times[i] <= time_new[j] <= times[i+1]:
                dt = time_new[j] - times[i]
                ax, bx, cx, dx = splines_x[i]
                ay, by, cy, dy = splines_y[i]
                x_new[j] = ax * dt**3 + bx * dt**2 + cx * dt + dx
                y_new[j] = ay * dt**3 + by * dt**2 + cy * dt + dy
                coordinates_trajectory.append([x_new[j],y_new[j]])

    return np.array(coordinates_trajectory)
# END OF NEW


def run_mobile_reacher(n_steps=10000, render=False, goal=True, obstacles=True):
    robots = [
        GenericUrdfReacher(urdf="mobilePanda_with_gripper.urdf", mode="vel"),
    ]
    env: UrdfEnv = UrdfEnv(
        dt=0.01, robots=robots, render=render, num_sub_steps=200,
    )
    action = np.zeros(env.n())
    ob = env.reset()

    ob, *_ = env.step(action) 
    
    current_xyz = np.round(ob['robot_0']['joint_state']['position'][:3],4)

    
    history = []
    
  
    action_to_send=[0,0,0]

    V_max = 10 # Maximum velocity (units per time unit)

    # NEW NEW NEW
    dt=0.01
    trajectory_xyz=np.vstack((current_xyz,target_xyz))
    coordinates_trajectory=trajectory_planning(trajectory_xyz,V_max, dt)
    Kp=5
    Kd=2
    e_prev=[0,0]
    time_prev=0
    # END OF NEW
    
    
    for i in range(n_steps):
        
        # NEW NEW NEW
        if (len(coordinates_trajectory)>i): 
           
            error_xyz = coordinates_trajectory[i] - current_xyz[:2]
            Derivative_error=Kd*(error_xyz - e_prev)/(i+1 - time_prev)
            desired_velocity_xyz = Kp*error_xyz + Derivative_error
            action_to_send = desired_velocity_xyz

            e_prev = error_xyz
            time_prev = i
            
        elif(len(coordinates_trajectory)+100>i):   
            error_xyz = coordinates_trajectory[-1] - current_xyz[:2]
            Derivative_error=Kd*(error_xyz - e_prev)/(i+1 - time_prev)
            desired_velocity_xyz = Kp*error_xyz + Derivative_error
            action_to_send = desired_velocity_xyz

            e_prev = error_xyz
            time_prev = i

        # END OF NEW   
        
        else:
            action_to_send = 0
            
        
        "Action is the variable with the taregt velocities for the robot"
        action[:2]=action_to_send
        "This is what actually gives the command for the robot to move, so I suppose this is what this node should emmit"
        ob, *_ = env.step(action) 
        current_xyz = np.round(ob['robot_0']['joint_state']['position'][:3],4)  
        history.append(ob)

      
    env.close()

    

    return history


if __name__ == "__main__":
    run_mobile_reacher(render=True)