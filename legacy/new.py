import matplotlib.pyplot as plt
import numpy as np

# Constants, positions, and functions as previously defined
k_att = 1
k_rep_base = 100
D0 = 5
goal_pos = np.array([40, 40])
decoy_pos = np.array([10, 40])
start_pos = np.array([25, 0])
grid_size = 50

def distance(pos1, pos2):
    return np.linalg.norm(pos1 - pos2)

def attractive_potential(pos):
    d_goal = distance(pos, goal_pos)
    return 0.5 * k_att * d_goal**2

def repulsive_potential(pos):
    d_decoy = distance(pos, decoy_pos)
    d_goal = distance(pos, goal_pos)
    if d_decoy < D0:
        k_rep = k_rep_base / (d_goal**2 + 1)
        return 0.5 * k_rep * ((1/d_decoy - 1/D0)**2)
    else:
        return 0

def total_potential(pos):
    return attractive_potential(pos) + repulsive_potential(pos)

# Path planning function as previously defined
def plan_path(start_pos, goal_pos, step_size=0.5, max_steps=1000):
    path = [start_pos]
    current_pos = np.array(start_pos, dtype=float)
    
    for _ in range(max_steps):
        grad_x = -(total_potential(current_pos + np.array([step_size, 0])) - total_potential(current_pos - np.array([step_size, 0]))) / (2*step_size)
        grad_y = -(total_potential(current_pos + np.array([0, step_size])) - total_potential(current_pos - np.array([0, step_size]))) / (2*step_size)
        grad = np.array([grad_x, grad_y])
        
        next_pos = current_pos - step_size * grad / np.linalg.norm(grad)
        current_pos = next_pos
        path.append(current_pos)
        
        if distance(current_pos, goal_pos) < step_size:
            break
    
    return np.array(path)

# Compute path
path = plan_path(start_pos, goal_pos)

# Grid of points for quiver plot
x_range = np.linspace(0, grid_size, 20)
y_range = np.linspace(0, grid_size, 20)
X, Y = np.meshgrid(x_range, y_range)
U, V = np.zeros(X.shape), np.zeros(Y.shape)

for i in range(X.shape[0]):
    for j in range(X.shape[1]):
        pos = np.array([X[i, j], Y[i, j]])
        grad_x = -(total_potential(pos + np.array([0.01, 0])) - total_potential(pos - np.array([0.01, 0]))) / 0.02
        grad_y = -(total_potential(pos + np.array([0, 0.01])) - total_potential(pos - np.array([0, 0.01]))) / 0.02
        U[i, j], V[i, j] = grad_x, grad_y  # Negative gradient for force direction

# Plotting
fig, ax = plt.subplots(figsize=(10, 10))
ax.quiver(X, Y, U, V, color='lightgray')  # Quiver plot for potential field forces
ax.plot(path[:, 0], path[:, 1], '-o', label='Path', markersize=4, color='blue')  # Path
ax.plot(goal_pos[0], goal_pos[1], 'ro', label='Actual Goal')  # Actual goal
ax.plot(decoy_pos[0], decoy_pos[1], 'bx', label='Decoy Goal')  # Decoy goal
ax.plot(start_pos[0], start_pos[1], 'g^', label='Start Position')  # Start position

plt.xlim(0, grid_size)
plt.ylim(0, grid_size)
plt.xlabel('X position')
plt.ylabel('Y position')
plt.title('Path Planning with Quiver Arrows Showing Forces')
plt.legend()
plt.grid(True)
plt.show()
