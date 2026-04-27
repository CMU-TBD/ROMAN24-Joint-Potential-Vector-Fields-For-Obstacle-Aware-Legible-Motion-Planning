# import numpy as np
# import matplotlib.pyplot as plt
# from matplotlib.animation import FuncAnimation, Animation
# from matplotlib.patches import Circle
# from scipy.interpolate import CubicSpline
# from scipy.spatial.distance import euclidean

# # Global variables
# grid_width = 100
# grid_height = 100
# start = (10, 10)
# goal = (90, 90)
# path_steps = 50
# sampling_radius = 10
# N_samples = 5
# obstacle_time_step = 10
# animation_interval = 500
# marker_size = 5

# def calculate_distance(point1, point2):
#     return euclidean(point1, point2)

# def sample_points_in_circle(center, radius, N):
#     angles = np.random.uniform(0, 2 * np.pi, N)
#     r = radius * np.sqrt(np.random.uniform(0, 1, N))
#     x = center[0] + r * np.cos(angles)
#     y = center[1] + r * np.sin(angles)
    
#     x = np.clip(x, 0, grid_width)
#     y = np.clip(y, 0, grid_height)
    
#     return list(zip(x, y))

# def generate_spline_path(start, goal, steps):
#     control_points = np.array([start, [(start[0] + goal[0]) / 2, (start[1] + goal[1]) / 2 + 20], goal])
#     x = control_points[:, 0]
#     y = control_points[:, 1]
    
#     cs_x = CubicSpline(np.linspace(0, 1, len(x)), x)
#     cs_y = CubicSpline(np.linspace(0, 1, len(y)), y)
    
#     t = np.linspace(0, 1, steps)
#     path_x = cs_x(t)
#     path_y = cs_y(t)
    
#     return list(zip(path_x, path_y))

# def generate_avoidance_path(start, goal, obstacle):
#     control_points = np.array([start, [(start[0] + goal[0]) / 2, (start[1] + goal[1]) / 2 + 20], goal])
#     obstacle_x, obstacle_y = obstacle
#     obstacle_point = [(start[0] + obstacle_x) / 2, (start[1] + obstacle_y) / 2]
#     control_points = np.insert(control_points, 2, obstacle_point, axis=0)
#     x = control_points[:, 0]
#     y = control_points[:, 1]
    
#     cs_x = CubicSpline(np.linspace(0, 1, len(x)), x)
#     cs_y = CubicSpline(np.linspace(0, 1, len(y)), y)
    
#     t = np.linspace(0, 1, path_steps)
#     path_x = cs_x(t)
#     path_y = cs_y(t)
    
#     return list(zip(path_x, path_y))

# path = generate_spline_path(start, goal, path_steps)

# fig, ax = plt.subplots()
# plt.subplots_adjust(bottom=0.2)
# ax.set_xlim(0, grid_width)
# ax.set_ylim(0, grid_height)
# ax.plot(*start, 'go', markersize=marker_size, label='Start')
# ax.plot(*goal, 'ro', markersize=marker_size, label='Goal')

# samples, = ax.plot([], [], 'bx', label='Sampled Points')
# traveled_path, = ax.plot([], [], 'r.', markersize=marker_size - 1)
# current_point, = ax.plot([], [], 'co', markersize=marker_size, label='Current Point')
# next_step_mark, = ax.plot([], [], 'ms', markersize=marker_size, label='Next Step')
# obstacle_point, = ax.plot([], [], 'kp', markersize=marker_size, label='Obstacle')
# nearest_point, = ax.plot([], [], 'g*', markersize=marker_size, label='Nearest to Obstacle')
# sampling_circle = Circle((0, 0), sampling_radius, color='red', fill=False, alpha=0.5)
# ax.add_artist(sampling_circle)
# ax.plot(*zip(*path), 'y-', alpha=0.5, linewidth=2, label='Planned Path')

# def init():
#     samples.set_data([], [])
#     traveled_path.set_data([], [])
#     current_point.set_data([], [])
#     next_step_mark.set_data([], [])
#     obstacle_point.set_data([], [])
#     nearest_point.set_data([], [])
#     sampling_circle.center = (0, 0)
#     return samples, traveled_path, current_point, next_step_mark, obstacle_point, nearest_point, sampling_circle

# def animate(i):
#     if i < len(path):
#         traveled_path.set_data(*zip(*path[:i+1]))
#         current_point.set_data(*path[i])
    
#     next_step = i + 1
#     if next_step < len(path):
#         sampled_points = sample_points_in_circle(path[next_step], sampling_radius, N_samples)
#         samples_x, samples_y = zip(*sampled_points)
#         samples.set_data(samples_x, samples_y)
#         sampling_circle.center = path[next_step]
#         next_step_mark.set_data(*path[next_step])
#         if i == obstacle_time_step:
#             obstacle = sampled_points[0]
#             obstacle_point.set_data(*obstacle)
#             distances = [calculate_distance(obstacle, pt) for pt in sampled_points[1:]]
#             nearest_idx = np.argmin(distances)
#             nearest = sampled_points[nearest_idx + 1]
#             nearest_point.set_data(*nearest)

#     else:
#         samples.set_data([], [])
#         next_step_mark.set_data([], [])
#         sampling_circle.center = (0, 0)

#     return samples, traveled_path, current_point, next_step_mark, obstacle_point, nearest_point, sampling_circle


# anim = FuncAnimation(fig, animate, init_func=init, frames=path_steps + 20, interval=animation_interval, blit=True)

# plt.legend()
# plt.show()




import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.patches import Circle, Arc, Polygon
from scipy.interpolate import CubicSpline
from scipy.spatial.distance import euclidean

# Global variables
# np.random.seed(0)
grid_width = 100
grid_height = 100
start = (10, 10)
goal = (90, 90)
path_steps = 50
sampling_radius = 10
N_samples = 5
obstacle_time_step = 10
animation_interval = 750
marker_size = 5
sector_angle = 90

def draw_sector(ax, center, radius, angle, sector_angle):
    """
    Draw a sector centered at 'center', with 'radius', and an orientation of 'angle'.
    The sector spans 'sector_angle' degrees around the central line.
    """
    # Remove previous sector if it exists
    for artist in ax.findobj(match=Polygon):
        artist.remove()

    # Calculate sector's bounding points
    start_angle_rad = np.radians(angle - sector_angle / 2)
    end_angle_rad = np.radians(angle + sector_angle / 2)

    # Define the arc points
    arc_angles = np.linspace(start_angle_rad, end_angle_rad, num=50)
    arc_points = [(center[0] + radius * np.cos(a), center[1] + radius * np.sin(a)) for a in arc_angles]

    # Points for the sector polygon
    sector_points = [center] + arc_points + [center]

    # Create and add the sector polygon to the axes
    sector_polygon = Polygon(sector_points, fill=False, edgecolor='red', alpha=0.5)
    ax.add_patch(sector_polygon)

def calculate_angle(point1, point2):
    """
    Calculate the angle (in degrees) of the line connecting point1 and point2 relative to the x-axis.
    """
    dx = point2[0] - point1[0]
    dy = point2[1] - point1[1]
    return np.degrees(np.arctan2(dy, dx))

def calculate_distance(point1, point2):
    return euclidean(point1, point2)

def sample_points_on_sector(center, radius, N, sector_angle=sector_angle):
    # Calculate the number of points to sample on the arc and the straight lines
    N_arc = N // 3
    N_lines = N - N_arc
    N_line_each = N_lines // 2

    # Sample points on the arc
    arc_angles = np.random.uniform(-sector_angle / 2, sector_angle / 2, N_arc) * (np.pi / 180)
    arc_x = center[0] + radius * np.cos(arc_angles)
    arc_y = center[1] + radius * np.sin(arc_angles)

    # Sample points on the straight lines
    # Line 1 (starting from the center to the edge of the sector)
    line1_distances = np.random.uniform(0, radius, N_line_each)
    line1_angle = -sector_angle / 2 * (np.pi / 180)
    line1_x = center[0] + line1_distances * np.cos(line1_angle)
    line1_y = center[1] + line1_distances * np.sin(line1_angle)

    # Line 2
    line2_distances = np.random.uniform(0, radius, N_line_each)
    line2_angle = sector_angle / 2 * (np.pi / 180)
    line2_x = center[0] + line2_distances * np.cos(line2_angle)
    line2_y = center[1] + line2_distances * np.sin(line2_angle)

    # Combine all points
    x = np.concatenate((arc_x, line1_x, line2_x))
    y = np.concatenate((arc_y, line1_y, line2_y))

    return list(zip(x, y))

def sample_points_in_circle(center, radius, N):
    angles = np.random.uniform(0, 2 * np.pi, N)
    r = radius * np.sqrt(np.random.uniform(0, 1, N))
    x = center[0] + r * np.cos(angles)
    y = center[1] + r * np.sin(angles)
    
    x = np.clip(x, 0, grid_width)
    y = np.clip(y, 0, grid_height)
    
    return list(zip(x, y))

def generate_spline_path(start, goal, steps):
    control_points = np.array([start, [(start[0] + goal[0]) / 2, (start[1] + goal[1]) / 2 + 20], goal])
    x = control_points[:, 0]
    y = control_points[:, 1]
    
    cs_x = CubicSpline(np.linspace(0, 1, len(x)), x)
    cs_y = CubicSpline(np.linspace(0, 1, len(y)), y)
    
    t = np.linspace(0, 1, steps)
    path_x = cs_x(t)
    path_y = cs_y(t)
    
    return list(zip(path_x, path_y))

def generate_avoidance_path(start, goal, obstacle):
    # Adjusted to generate a clear avoidance path for each obstacle
    dx = (goal[0] - start[0]) * 0.2
    dy = (goal[1] - start[1]) * 0.2
    control_points = np.array([start, [obstacle[0] +  dx, obstacle[1] + dy], goal])
    x = control_points[:, 0]
    y = control_points[:, 1]

    cs_x = CubicSpline(np.linspace(0, 1, len(x)), x)
    cs_y = CubicSpline(np.linspace(0, 1, len(y)), y)

    t = np.linspace(0, 1, path_steps)
    path_x = cs_x(t)
    path_y = cs_y(t)

    return list(zip(path_x, path_y))

path = generate_spline_path(start, goal, path_steps)

fig, ax = plt.subplots()
plt.subplots_adjust(bottom=0.2)
ax.set_xlim(0, grid_width)
ax.set_ylim(0, grid_height)
ax.plot(*start, 'go', markersize=marker_size, label='Start')
ax.plot(*goal, 'ro', markersize=marker_size, label='Goal')

samples, = ax.plot([], [], 'bx', label='Sampled Points')
traveled_path, = ax.plot([], [], 'r.', markersize=marker_size - 1)
current_point, = ax.plot([], [], 'co', markersize=marker_size, label='Current Point')
next_step_mark, = ax.plot([], [], 'ms', markersize=marker_size, label='Next Step')
obstacle_point, = ax.plot([], [], 'kp', markersize=marker_size, label='Obstacle')
nearest_point, = ax.plot([], [], 'g*', markersize=marker_size, label='Nearest to Obstacle')
sampling_circle = Circle((0, 0), sampling_radius, color='red', fill=False, alpha=0.5)
ax.add_artist(sampling_circle)
ax.plot(*zip(*path), 'y-', alpha=0.5, linewidth=2, label='Planned Path')

# List to store hypothetical path lines for resetting
hypothetical_paths_lines = []

def init():
    # path = generate_spline_path(start, goal, path_steps)
    samples.set_data([], [])
    traveled_path.set_data([], [])
    current_point.set_data([], [])
    next_step_mark.set_data([], [])
    obstacle_point.set_data([], [])
    nearest_point.set_data([], [])
    sampling_circle.center = (0, 0)
    for line in hypothetical_paths_lines:
        line.remove()  # Remove lines from the plot
    hypothetical_paths_lines.clear()
    # global hypothetical_paths_lines, path
    # path = generate_spline_path(start, goal, path_steps)

    return samples, traveled_path, current_point, next_step_mark, obstacle_point, nearest_point, sampling_circle

def animate(i):
    global hypothetical_paths_lines, path
    if i == len(path) - 1:
        path = generate_spline_path(start, goal, path_steps)
    

    if i < len(path) - 1:
        traveled_path.set_data(*zip(*path[:i+1]))
        current_point.set_data(*path[i])

    next_step = i + 1 if i + 1 < len(path) else i
    direction_angle = calculate_angle(path[i], path[next_step])
    # draw_sector(ax, next_step, sampling_radius, 0, sector_angle)
    draw_sector(ax, path[i], sampling_radius, direction_angle, sector_angle)
    sampled_points = sample_points_in_circle(path[next_step], sampling_radius, N_samples)
    samples_x, samples_y = zip(*sampled_points)
    samples.set_data(samples_x, samples_y)
    sampling_circle.center = path[next_step]
    next_step_mark.set_data(*path[next_step])

    # Clear previous hypothetical paths
    for line in hypothetical_paths_lines:
        line.remove()
    hypothetical_paths_lines.clear()

    # Generate and display hypothetical paths for each sampled point
    hp_paths = []
    for sp in sampled_points:
        hypothetical_path = generate_avoidance_path(path[i + 1], goal, sp)
        hp_paths.append(hypothetical_path)
        hp_line, = ax.plot(*zip(*hypothetical_path), '--', linewidth=1, alpha=0.5)
        hypothetical_paths_lines.append(hp_line)

    # Generate a random obstacle in the sampling area at the specified time step
    if i == obstacle_time_step:
        obstacle = (np.random.uniform(path[next_step][0] - sampling_radius, path[next_step][0] + sampling_radius),
                    np.random.uniform(path[next_step][1] - sampling_radius, path[next_step][1] + sampling_radius))
        obstacle_point.set_data(*obstacle)
        # Calculate and mark the nearest sampled point to the obstacle
        distances = [calculate_distance(obstacle, pt) for pt in sampled_points]
        nearest_idx = np.argmin(distances)
        nearest = sampled_points[nearest_idx]
        nearest_point.set_data(*nearest)

        # Select the avoidance path that avoids the nearest sampled point to the obstacle
        # avoidance_path = hp_paths[nearest_idx]
        # traveled_path.set_data(*zip(*avoidance_path))
        # path = avoidance_path  # Update the path to the avoidance path



    return samples, traveled_path, current_point, next_step_mark, obstacle_point, nearest_point, sampling_circle, *hypothetical_paths_lines



anim = FuncAnimation(fig, animate, init_func=init, frames=len(path), interval=animation_interval, blit=False)

plt.legend()
plt.show()
