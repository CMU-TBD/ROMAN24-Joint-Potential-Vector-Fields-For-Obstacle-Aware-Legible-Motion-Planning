import numpy as np
import matplotlib.pyplot as plt

# Define the start, goal, and obstacles
start_pos = np.array([0, 0])
goal_pos = np.array([10, 10])
obstacles = [np.array([5, 5])]  # List of obstacle positions
obstacle_radius = 0.5
step_size = 0.1
attraction_coeff = 10.0
repulsion_coeff = 20.0
influence_radius = 2.0
max_iterations = 10000  # Maximum number of iterations to prevent infinite loop
goal_threshold = 0.1  # Distance from goal to consider as reached

def calculate_attractive_force(position):
    force = attraction_coeff * (goal_pos - position)
    return force

def calculate_repulsive_force(position):
    force = np.zeros(2)
    for obstacle in obstacles:
        vec_to_obstacle = position - obstacle
        distance = np.linalg.norm(vec_to_obstacle)
        if distance < influence_radius:
            force -= repulsion_coeff * (1 / distance - 1 / influence_radius) * (1 / distance**2) * (vec_to_obstacle / distance)
    return force

def update_position(position):
    total_force = calculate_attractive_force(position) + calculate_repulsive_force(position)
    # Normalize the total force vector if it's not zero to avoid division by zero
    if np.linalg.norm(total_force) > 0:
        new_position = position + (total_force / np.linalg.norm(total_force)) * step_size
    else:
        new_position = position
    return new_position

# Initialize the path with the start position
path = [start_pos]
current_pos = start_pos
iterations = 0

# Iterate until the robot reaches the goal or maximum iterations reached
while np.linalg.norm(goal_pos - current_pos) > goal_threshold and iterations < max_iterations:
    current_pos = update_position(current_pos)
    path.append(current_pos)
    iterations += 1

# Plotting the path and the environment
path = np.array(path)
plt.figure(figsize=(8, 8))
plt.plot(path[:, 0], path[:, 1], label='Path')
plt.scatter(start_pos[0], start_pos[1], color='green', label='Start')
plt.scatter(goal_pos[0], goal_pos[1], color='red', label='Goal')
for obstacle in obstacles:
    circle = plt.Circle(obstacle, obstacle_radius, color='black', fill=True)
    plt.gca().add_artist(circle)
plt.xlabel('X')
plt.ylabel('Y')
plt.title('Path Planning with APF')
plt.legend()
plt.grid(True)
plt.axis('equal')
plt.show()
