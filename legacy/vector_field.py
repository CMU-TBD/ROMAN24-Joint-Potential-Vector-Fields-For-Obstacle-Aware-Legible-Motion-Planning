import numpy as np
import matplotlib.pyplot as plt

def circular_force_field(obstacle_center, field_strength, effect_radius, X, Y):
    """
    Generate a circular force field around an obstacle, contained within a specified radius.
    """
    dx = X - obstacle_center[0]
    dy = Y - obstacle_center[1]
    distance = np.sqrt(dx**2 + dy**2)
    
    # Initialize force vectors
    U = np.zeros_like(X)
    V = np.zeros_like(Y)
    
    # Apply force only within the effect radius
    within_effect = distance <= effect_radius
    U[within_effect] = -field_strength * dy[within_effect] / distance[within_effect]**2
    V[within_effect] = field_strength * dx[within_effect] / distance[within_effect]**2
    
    return U, V

def attractive_potential(goal, strength, X, Y):
    """
    Generate an attractive potential towards the goal.
    """
    dx = X - goal[0]
    dy = Y - goal[1]
    U = -strength * dx
    V = -strength * dy
    return U, V

def simulate_path(start_point, goal, obstacle_center, field_strength, goal_strength, effect_radius, grid_size, resolution, max_steps=1000, step_size=0.1):
    """
    Simulate the agent's path in the combined potential field.
    """
    x = np.linspace(0, grid_size[0], resolution)
    y = np.linspace(0, grid_size[1], resolution)
    X, Y = np.meshgrid(x, y)
    
    # Generate the vector fields
    U_obstacle, V_obstacle = circular_force_field(obstacle_center, field_strength, effect_radius, X, Y)
    U_goal, V_goal = attractive_potential(goal, goal_strength, X, Y)
    
    # Combine the fields
    U_total = U_obstacle + U_goal
    V_total = V_obstacle + V_goal
    
    # Simulate the path
    path = [np.array(start_point)]
    for _ in range(max_steps):
        current_pos = path[-1]
        ix = np.argmin(np.abs(x - current_pos[0]))
        iy = np.argmin(np.abs(y - current_pos[1]))
        dx = U_total[iy, ix]
        dy = V_total[iy, ix]
        next_pos = current_pos + np.array([dx, dy]) * step_size
        path.append(next_pos)
        if np.linalg.norm(next_pos - np.array(goal)) < step_size:  # Check if goal is reached
            break
    
    return np.array(path), X, Y, U_total, V_total

# Parameters
start_point = (10, 10)
goal = (90, 90)
obstacle_center = (50, 50)
field_strength = 1000
goal_strength = 5
effect_radius = 20  # Radius of effect around the obstacle
grid_size = (100, 100)
resolution = 100

# Simulate the path
path, X, Y, U_total, V_total = simulate_path(start_point, goal, obstacle_center, field_strength, goal_strength, effect_radius, grid_size, resolution)

# Plotting
plt.figure(figsize=(10, 10))
plt.quiver(X, Y, U_total, V_total, alpha=0.5)
plt.plot(path[:, 0], path[:, 1], 'r-', label='Path')
plt.plot(start_point[0], start_point[1], 'go', label='Start')
plt.plot(goal[0], goal[1], 'bo', label='Goal')
plt.plot(obstacle_center[0], obstacle_center[1], 'rx', label='Obstacle')
plt.xlim(0, grid_size[0])
plt.ylim(0, grid_size[1])
plt.legend()
plt.title('Path Planning with Circular Obstacle Avoidance within a Limited Region')
plt.show()
