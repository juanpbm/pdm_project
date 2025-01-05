import numpy as np
def trajectory_planning(via_points,V_max, n_steps):
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
    time_new = np.linspace(times[0], times[-1], n_steps)
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