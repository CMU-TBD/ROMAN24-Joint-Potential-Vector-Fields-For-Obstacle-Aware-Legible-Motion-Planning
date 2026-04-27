import numpy as np
import matplotlib.pyplot as plt

# Parameters
m = 1.0  # Mass of the robot
k_goal = 5  # Spring constant for the goal
k_obstacle = 8  # Spring constant for the obstacle
b = 0.1  # Damping coefficient
d0 = 2.0  # Effective range of the repulsive force
dt = 0.1  # Time step
goal = np.array([10.0, 10.0])  # Goal position
obstacle = np.array([5.0, 5.0])  # Obstacle position
robot_pos = np.array([0.0, 0.0])  # Initial robot position
v = np.array([0.0, 0.0])  # Initial velocity

def calculate_force(robot_pos, goal, obstacle, v):
    # Attractive force towards the goal
    f_goal = -k_goal * (robot_pos - goal)
    
    # Distance to the obstacle
    dist_to_obstacle = np.linalg.norm(robot_pos - obstacle)
    
    # Repulsive force from the obstacle
    if dist_to_obstacle < d0:
        f_obstacle = k_obstacle * (1/dist_to_obstacle - 1/d0) * (1/dist_to_obstacle**2) * (robot_pos - obstacle)
    else:
        f_obstacle = np.array([0, 0])
    
    # Damping force
    f_damping = -b * v
    
    # Net force
    f_net = f_goal + f_obstacle + f_damping
    return f_net

# Simulation loop
num_steps = 200
positions = [robot_pos.copy()]

for _ in range(num_steps):
    f_net = calculate_force(robot_pos, goal, obstacle, v)
    a = f_net / m
    v += a * dt
    robot_pos += v * dt
    positions.append(robot_pos.copy())

# Convert positions to a Numpy array for plotting
positions = np.array(positions)

# Plotting
plt.figure(figsize=(8, 8))
plt.plot(positions[:, 0], positions[:, 1], label='Robot Path')
plt.scatter(*goal, color='g', label='Goal')
plt.scatter(*obstacle, color='r', label='Obstacle')
plt.xlabel('X Position')
plt.ylabel('Y Position')
plt.title('2D Robot Navigation')
plt.legend()
plt.grid(True)
plt.show()
