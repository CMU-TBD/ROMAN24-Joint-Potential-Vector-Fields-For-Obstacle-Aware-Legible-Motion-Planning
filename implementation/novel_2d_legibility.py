import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import CubicSpline
from math import exp
from matplotlib.animation import FuncAnimation
from scipy.integrate import quad
from scipy.interpolate import interp1d

# import new_leb


x = np.arange(0,80/10,1, dtype=float) #80
y = np.arange(0,80/10,1, dtype=float)
X, Y = np.meshgrid(x,y)
# obstacles = [[40.0, 20.0],[52.0, 40.0]]
# obstacles = [[48.0, 30.0],[56.0, 30.0]]
# obstacles = [[52.5, 31.0]]
# obstacles = [[65.0, 15.0], [20.0, 20.0]]
# obstacles = [[41.0,20.0],[15.0,40.0]]
# obstacles = [[15.0,35.0]]
obstacles=[]
goal = [40.0/10,60.0/10] # 40 60
goal_decoy = [20.0/10,60.0/10] # 20 60
# goal_decoy = [40.0,60.0]
# goal = [20.0,60.0]
s_obs = 20.0 # 20
r_obs = 1.0 # 1
s_goal = 5.0 # 5
r_goal = 5.0 / 2.0 # 5/2
r_decoy = 5.0 / 2.0 # 5/2
start_point = np.array([30.0/10,0.0])  #30
# print(np.hypot(52.0-goal_decoy[0],44.0-goal_decoy[1]))
# start_point = np.array([52.0,44.0]) 
s_decoy = np.sqrt((start_point[0]-goal_decoy[0])**2 + (start_point[1]-goal_decoy[1])**2)
ka = 1.0 # 12 or 15 20
kp = 0.2 # 2.0
ko = 0.8 # 0.8
n = 5 # 5

s_obs = 10 # 10
step_size = 0.001 #0.0001
thresh = r_goal
field_strength = 50 # 50

def add_goal(X, Y, s_goal, r_goal, loc):

  delx = np.zeros_like(X)
  dely = np.zeros_like(Y)
  for i in range(len(x)):
    for j in range(len(y)):
      d= np.sqrt((loc[0]-X[i][j])**2 + (loc[1]-Y[i][j])**2)
      theta = np.arctan2(loc[1]-Y[i][j], loc[0] - X[i][j])
      delx[i][j] = ka * d * np.cos(theta)
      dely[i][j] = ka * d * np.sin(theta)
  return delx, dely

def add_goal_decoy(X, Y, s_decoy, r_decoy, loc):
  delx = np.zeros_like(X)
  dely = np.zeros_like(Y)
  for i in range(len(x)):
    for j in range(len(y)):
      d = np.sqrt((loc[0]-X[i][j])**2 + (loc[1]-Y[i][j])**2)
      d_goal = np.sqrt((goal[0]-X[i][j])**2 + (goal[1]-Y[i][j])**2)
      theta = np.arctan2(loc[1]-Y[i][j], loc[0] - X[i][j])
      if d< r_decoy:
        delx[i][j] = -1 * np.sign(np.cos(theta)) * 10 # 10
        dely[i][j] = -1 * np.sign(np.sin(theta)) * 10
      else:
        if d <= 0.0001: # 0.0001
          d = 0.0001
        delx[i][j] = -kp * ((1.0/d) - (1.0/s_decoy)) * (1.0/d)**2 * np.cos(theta) * d_goal**n
        dely[i][j] = -kp * ((1.0/d) - (1.0/s_decoy)) * (1.0/d)**2 * np.sin(theta) * d_goal**n
  
  return delx, dely


def circular_force_field(X, Y, obstacle_center, field_strength, effect_radius, direction=1.0):
    """
    Generate a circular force field around an obstacle, contained within a specified radius.
    """
    # Counter clockwise is positive, clockwise is negative
    U = np.zeros_like(X)
    V = np.zeros_like(Y)
    for i in range(len(x)):
      for j in range(len(y)):
        dx = X[i][j] - obstacle_center[0]
        dy = Y[i][j] - obstacle_center[1]
        distance = np.sqrt(dx**2 + dy**2)
        d_goal = np.sqrt((goal[0]-X[i][j])**2 + (goal[1]-Y[i][j])**2)
        d_o_g = np.sqrt((goal[0]-obstacle_center[0])**2 + (goal[1]-obstacle_center[1])**2)
        if distance <= 0.01:
          distance = 0.01
        
        if distance > effect_radius or d_o_g > d_goal:
        # if distance > effect_radius:
          U[i][j] = 0
          V[i][j] = 0
        else:
          if direction == 1.0 or direction == -1.0:
            U[i][j] = -direction * field_strength * dy / distance**2 * d_goal**ko
            V[i][j] = direction * field_strength * dx / distance**2 * d_goal**ko
          else:
            U[i][j] = 0
            V[i][j] = 0
          # else: # Clockwise
          #   U[i][j] = direction * field_strength * dy / distance**2 * d_goal**ko
          #   V[i][j] = -direction * field_strength * dx / distance**2 * d_goal**ko
    
    return U, V

def all_goals(X,Y,s_goal,r_goal,s_decoy,r_decoy, loc_goal, loc_decoy):
  delx_g, dely_g = add_goal(X,Y,s_goal,r_goal,loc_goal)
  delx_d, dely_d = add_goal_decoy(X,Y,s_decoy,r_decoy,loc_decoy)
  total_force_x, total_force_y = delx_g + delx_d, dely_g + dely_d
  # total_force_x, total_force_y = delx_g + delx_d, dely_g + dely_d
  # if np.hypot(obstacles[0],obstacles[1]) <= 10.0 * r_obs:
  #   obstacles = [[(obstacles[0][0] + obstacles[1][0]) / 2.0, (obstacles[0][1] + obstacles[1][1]) / 2.0]]
  #   u1, v1 = circular_force_field(obstacles[0], field_strength, 2.0*s_obs, 'cc',X,Y)
  #   total_force_x, total_force_y = total_force_x + u1 + u2, total_force_y + v1 + v2

  return total_force_x, total_force_y

def all_goals_potential_field(X,Y,s_goal,r_goal,s_decoy,r_decoy, loc_goal, loc_decoy):
  delx_g, dely_g = add_goal(X,Y,s_goal,r_goal,loc_goal)
  # delx_d, dely_d = add_goal_decoy(X,Y,s_decoy,r_decoy,loc_decoy)
  total_force_x, total_force_y = delx_g, dely_g
  # total_force_x, total_force_y = delx_g + delx_d, dely_g + dely_d
  # if np.hypot(obstacles[0],obstacles[1]) <= 10.0 * r_obs:
  #   obstacles = [[(obstacles[0][0] + obstacles[1][0]) / 2.0, (obstacles[0][1] + obstacles[1][1]) / 2.0]]
  #   u1, v1 = circular_force_field(obstacles[0], field_strength, 2.0*s_obs, 'cc',X,Y)
  #   total_force_x, total_force_y = total_force_x + u1 + u2, total_force_y + v1 + v2

  return total_force_x, total_force_y


total_force_x, total_force_y = all_goals(X,Y,s_goal,r_goal,s_decoy,r_decoy,goal,goal_decoy)
u, v = [], []

def goals_plus_obstacle(X,Y,total_force_x, total_force_y, obstacle, direction):
  u_temp, v_temp = circular_force_field(X,Y,obstacle, field_strength, s_obs, direction)
  total_force_x, total_force_y = total_force_x + u_temp, total_force_y + v_temp
  u.append(u_temp/10) # /1
  v.append(v_temp/10) # /1
  return total_force_x, total_force_y

def add_obstacle(X, Y , delx, dely, goal, obstacle):
  s = s_obs

  # generating obstacle with random sizes
  r = r_obs

  # generating random location of the obstacle 
  # obstacle = random.sample(range(0, 50), 2)
  for i in range(len(x)):
    for j in range(len(y)):
      
      d_goal = np.sqrt((goal[0]-X[i][j])**2 + ((goal[1]-Y[i][j]))**2)
      d_obstacle = np.sqrt((obstacle[0]-X[i][j])**2 + (obstacle[1]-Y[i][j])**2)
      #print(f"{i} and {j}")
      theta_goal= np.arctan2(goal[1] - Y[i][j], goal[0]  - X[i][j])
      theta_obstacle = np.arctan2(obstacle[1] - Y[i][j], obstacle[0]  - X[i][j])
      if d_obstacle < r:
        delx[i][j] = -1*np.sign(np.cos(theta_obstacle))*5 +0
        dely[i][j] = -1*np.sign(np.cos(theta_obstacle))*5  +0
      elif d_obstacle>r+s:
        delx[i][j] += 0 -(50 * s *np.cos(theta_goal))
        dely[i][j] += 0 - (50 * s *np.sin(theta_goal))
      elif d_obstacle<r+s :
        delx[i][j] += -150 *(s+r-d_obstacle)* np.cos(theta_obstacle)
        dely[i][j] += -150 * (s+r-d_obstacle)*  np.sin(theta_obstacle) 
      if d_goal <r+s:
        if delx[i][j] != 0:
          delx[i][j]  += (50 * (d_goal-r) *np.cos(theta_goal))
          dely[i][j]  += (50 * (d_goal-r) *np.sin(theta_goal))
        else:
          
          delx[i][j]  = (50 * (d_goal-r) *np.cos(theta_goal))
          dely[i][j]  = (50 * (d_goal-r) *np.sin(theta_goal))
          
      if d_goal>r+s:
        if delx[i][j] != 0:
          delx[i][j] += 50* s *np.cos(theta_goal)
          dely[i][j] += 50* s *np.sin(theta_goal)
        else:
          
          delx[i][j] = 50* s *np.cos(theta_goal)
          dely[i][j] = 50* s *np.sin(theta_goal) 
      if d_goal<r:
          delx[i][j] = 0
          dely[i][j] = 0
   
  return delx, dely


"""
  Args:
    X =  2D array of the Points on X-axis
    Y =  2D array of the Points on Y-axis 
    delx = Usual meaning
    dely = Usual Meaninig
    obj = String to tell is the object on the map is Goal or the Obstacle
    fig = Matplotlib figure
    ax = Axis of the figure
    loc = Location of the object
    r = Size of the object
    i = Number of the Object
    color = coloer of the object
    start_goal = starting point of the robot, default = (0,0)
  Returns:
    ax = axis of the figure
  This function plot the quiver plot, draw the goal/ obstacle at the given location
  whith given color and text.  
"""
def plot_graph_g(X, Y, delx, dely,obj_g, obj_d, obj_o, fig, ax, loc_g, loc_d, loc_o, r_goal, r_decoy, r_obs, color_g, color_d, color_o, u,v, start_point):
  # ax.quiver(X, Y, delx, dely)
  ax.set_xlim(0,7)
  ax.set_ylim(0,7)
  ax.add_patch(plt.Circle(loc_g, r_goal, color=color_g, label="Real Goal"))
  ax.add_patch(plt.Circle(loc_d, r_decoy, color=color_d, label="Other Goal"))
  # ax.annotate(obj_g, xy=loc_g, fontsize=10, ha="center")
  # ax.annotate(obj_d, xy=loc_d, fontsize=10, ha="center")
  # if loc_o != [] and u != []:
  #   for i, loc in enumerate(loc_o):

  # ax.add_patch(plt.Circle([4.1, 2.0], r_obs, color=color_o, label="Obstacle"))
  # # ax.annotate(obj_o, xy=loc, fontsize=10, ha="center")
  # # print(u[0])
  # ax.quiver(X,Y,u[0],v[0], scale=900)
  # ax.add_patch(plt.Circle(loc_o[1], r_obs, color=color_o))
  # # ax.annotate(obj_o, xy=loc, fontsize=10, ha="center")
  # ax.quiver(X,Y,u[1],v[1], scale=500)

  # ax.set_title(f'Robot path with {i} obstacles ')
  ax.set_title("Robot Paths with " + str(len(obstacles)) + " obstacle(s)", fontsize=20)
  ax.plot(start_point[0], start_point[1], 'o', color='gray', markersize=20, label='Start Point')
  
  
  return ax

def plot_graph_g_switch(X, Y, delx, dely,obj_g, obj_d, obj_o, fig, ax, loc_g, loc_d, loc_o, r_goal, r_decoy, r_obs, color_g, color_d, color_o, u,v, start_point, switch_point):
  # ax.quiver(X, Y, delx, dely)
  ax.add_patch(plt.Circle(loc_g, r_goal, color=color_g, label="Real Goal"))
  ax.add_patch(plt.Circle(loc_d, r_decoy, color=color_d, label="Other Goal"))
  # ax.annotate(obj_g, xy=loc_g, fontsize=10, ha="center")
  # ax.annotate(obj_d, xy=loc_d, fontsize=10, ha="center")
  # if loc_o != [] and u != []:
  #   for i, loc in enumerate(loc_o):
      # ax.add_patch(plt.Circle(loc, r_obs, color=color_o, label="Obstacle"))
  #     ax.annotate(obj_o, xy=loc, fontsize=10, ha="center")
  #     # ax.quiver(X,Y,u[i],v[i], scale=5000)
  # # ax.set_title(f'Robot path with {i} obstacles ')
  # ax.set_title("Robot path with " + str(len(loc_o)) + " obstacle(s)")
  ax.set_title("Robot path with " + str(0) + " obstacle(s) Switch Goal", fontsize=20)
  ax.plot(start_point[0], start_point[1], 'o', color='gray', markersize=20, label='Start Point')
  ax.plot(switch_point[0], switch_point[1], 'r*', markersize=20, label='Switching Point')
  
  return ax  


def plot_graph_d(X, Y, delx, dely,obj, fig, ax, loc,r, color,start_goal=np.array([[0,0]])):
  
  ax.quiver(X, Y, delx, dely)
  # ax.add_patch(plt.Circle(loc, r, color=color))
  # ax.set_title(f'Robot path with {i} obstacles ')
  ax.set_title(f'Robot path with obstacles ')
  ax.annotate(obj, xy=loc, fontsize=10, ha="center")
  return ax

def plot_graph_o(X, Y, delx, dely,obj, fig, ax, locs,r, color,start_goal=np.array([[0,0]])):

  ax.quiver(X, Y, delx, dely)
  for loc in locs:
    ax.add_patch(plt.Circle(loc, r, color=color))
    ax.annotate(obj, xy=loc, fontsize=10, ha="center")
  # ax.set_title(f'Robot path with {i} obstacles ')
  ax.set_title(f'Robot path with obstacles ')
  return ax



# def add_obstacle(X, Y , delx, dely, goal, obstacles):

#   # generating obstacle with random sizes
# #   r = random.randint(5,25)/10

#   # generating random location of the obstacle 
#   # obstacle = random.sample(range(0, 50), 2)
#   # obstacle = [25,20]
#   for obstacle in obstacles:
#     for i in range(len(x)):
#       for j in range(len(y)):
        
#         d_goal = np.sqrt((goal[0]-X[i][j])**2 + ((goal[1]-Y[i][j]))**2)
#         d_obstacle = np.sqrt((obstacle[0]-X[i][j])**2 + (obstacle[1]-Y[i][j])**2)
#         #print(f"{i} and {j}")
#         theta_goal= np.arctan2(goal[1] - Y[i][j], goal[0]  - X[i][j])
#         theta_obstacle = np.arctan2(obstacle[1] - Y[i][j], obstacle[0]  - X[i][j])
#         if d_obstacle < r:
#           delx[i][j] = -1*np.sign(np.cos(theta_obstacle))*5 +0
#           dely[i][j] = -1*np.sign(np.cos(theta_obstacle))*5  +0
#         elif d_obstacle>r+s:
#           delx[i][j] += 0 -(50 * s *np.cos(theta_goal))
#           dely[i][j] += 0 - (50 * s *np.sin(theta_goal))
#         elif d_obstacle<r+s :
#           delx[i][j] += -150 *(s+r-d_obstacle)* np.cos(theta_obstacle)
#           dely[i][j] += -150 * (s+r-d_obstacle)*  np.sin(theta_obstacle) 
#         if d_goal <r+s:
#           if delx[i][j] != 0:
#             delx[i][j]  += (50 * (d_goal-r) *np.cos(theta_goal))
#             dely[i][j]  += (50 * (d_goal-r) *np.sin(theta_goal))
#           else:
            
#             delx[i][j]  = (50 * (d_goal-r) *np.cos(theta_goal))
#             dely[i][j]  = (50 * (d_goal-r) *np.sin(theta_goal))
            
#         if d_goal>r+s:
#           if delx[i][j] != 0:
#             delx[i][j] += 50* s *np.cos(theta_goal)
#             dely[i][j] += 50* s *np.sin(theta_goal)
#           else:
            
#             delx[i][j] = 50* s *np.cos(theta_goal)
#             dely[i][j] = 50* s *np.sin(theta_goal) 
#         if d_goal<r:
#             delx[i][j] = 0
#             dely[i][j] = 0
   
#   return delx, dely, obstacles, r


def heading_eq(line_point1, line_point2, point):
    """
    Check if two points lie on the same side of a line defined by two points.
    
    Parameters:
    - line_point1, line_point2: Tuples (x, y) defining two points on the line.
    - point1, point2: Tuples (x, y) for the points to check.
    
    Returns:
    - True if both points lie on the same side of the line or one of them is on the line.
    - False if they lie on opposite sides.
    """
    x1, y1 = line_point1
    x2, y2 = line_point2
    
    # Calculate the coefficients A, B, C of the line equation
    A = y2 - y1
    B = x1 - x2
    C = (x2 * y1) - (x1 * y2)

    return A * point[0] + B * point[1] + C

def find_point_at_distance(path, obstacle_point, D):
    """
    Find the first point on a path that is exactly D units from an obstacle point.

    Parameters:
    - path: List of tuples [(x, y)] representing points on the path.
    - obstacle_point: Tuple (x, y) representing the obstacle point.
    - D: The distance from the obstacle point.

    Returns:
    - The point (x, y) on the path that is exactly D units from the obstacle,
      or None if no such point exists.
    """
    if obstacle_point is not None:
      for i, point in enumerate(path):
          distance = ((point[0] - obstacle_point[0]) ** 2 + (point[1] - obstacle_point[1]) ** 2) ** 0.5
          if distance < D:  # Using a small tolerance to account for floating-point arithmetic
              return path[i], path[i-1], i
    return None, None, None



fig, ax = plt.subplots(figsize = (10,10))
# delx, dely =add_goal(X, Y,s_goal, r_goal , goal)
# delx, dely =add_goal_decoy(X, Y,s_decoy, r_decoy , goal_decoy)

# delx, dely = circular_force_field([40.0,40.0],10000,20,'cc',X,Y)
# print(delx)

# plot_graph_g(X, Y, delx, dely , 'Goal', 'Decoy Goal', 'Obstacle', goal, goal_decoy,obstacles, fig, ax, r_goal, r_decoy,r_obs, 'b', 'r','g')

# plot_graph_d(X, Y, delx, dely , 'Goal Decoy',fig, ax, goal_decoy, r_goal, 'r' )
# plot_graph_d(X, Y, delx, dely , 'Obs',fig, ax, [40.0, 40.0], 20, 'r' )
# delx, dely, locs, r = add_obstacle(X,Y, delx,dely,goal,obstacles)
# plot_graph_o(X, Y, delx, dely , 'Obstacle',fig, ax, locs, r ,'m')


# ax.plot(path[:, 0], path[:, 1], 'r-', linewidth=4, label='Path from Potential Field')

# directions = []

# Detect obstacle here
# obstacles = [[40.0, 20.0]]
# obstacles = [[40.0, 20.0],[55.0, 40.0]]

def obstacle_direction(path_zero,obstacle, s_obs, goal, goal_decoy, ax):
  line_point_1, line_point_2, contact_index = find_point_at_distance(path_zero, obstacle,s_obs)
  if contact_index is not None:
    # ax.plot(line_point_1[0], line_point_1[1], 'o')
    # ax.plot(line_point_2[0], line_point_2[1], 'o')
    # ax.axline((line_point_1[0], line_point_1[1]), (line_point_2[0], line_point_2[1]), label='Heading')

    value_o = heading_eq(line_point_1,line_point_2,obstacle)
    value_g = heading_eq(line_point_1, line_point_2, goal)
    value_d = heading_eq(line_point_1,line_point_2,goal_decoy)

    # if value_o >= 0.0:
    #   if value_g >= 0.0 and value_d >= 0.0:
    #     direction = 1
    #   else:
    #     direction = -1
    # else:
    #   if value_g >= 0.0 and value_d >= 0.0:
    #     direction = -1
    #   else:
    #     direction = 1
    distance = np.linalg.norm(np.cross(line_point_2-line_point_1, line_point_1-obstacle))/np.linalg.norm(line_point_2-line_point_1)
    # distance_goal = np.hypot(line_point_1[0]-goal[0], line_point_1[1]-goal[1])
    # distance_decoy = np.hypot(line_point_1[0]-goal_decoy[0], line_point_1[1]-goal_decoy[1])
    # print(distance)
    # if value_g >= 0.0:
    #   side = -1
    # print(value_g)
    if value_o * value_g <= 0.0: # Obstacle and goal in opposite sides of the line
      # if distance < r_obs:
      #   direction = 1
      # else:
      direction = -1
    else:
      if distance < 1.0*r_obs:   #2.0 * r_obs
        direction = -1
      else:
        direction = 1
    return obstacle, direction * np.sign(value_g), line_point_1, contact_index
  else:
    direction = 0.0
    return obstacle, direction, None, None

def get_path_from_field(start_point, goal, goal_decoy, delx, dely, step_size=step_size, thresh=thresh, flag=False):
    path = [np.array(start_point)]  # Initialize path with starting point
    step = 0
    # for _ in range(max_steps):
    path_goal_distance = np.hypot(path[-1][0] - goal[0], path[-1][1] - goal[1])
    while path_goal_distance >= thresh:
        step += 1
        current_pos = path[-1]
        # Convert current position to indices within the field arrays
        ix, iy = int(current_pos[0]), int(current_pos[1])
        
        # Ensure the indices are within the bounds of the vector field
        if ix < 0 or iy < 0 or ix >= delx.shape[1] or iy >= delx.shape[0]:
            break  # Stop if the next step is outside the field
        
        # Get the vector components at the current position
        dx, dy = delx[iy, ix], dely[iy, ix]
        
        # Calculate the next position based on the vector field
        next_pos = current_pos + np.array([dx, dy]) * step_size
        
        # Add the next position to the path
        path.append(next_pos)
        # A = y2 - y1
        # B = x1 - x2
        # C = (x2 * y1) - (x1 * y2)
        # if step > 1:
        #   A1, B1, C1 = path[-1][1] - path[-2][1], path[-1][0] - path[-2][0], path[-2][0]*path[-1][1] - path[-1][0]*path[-2][1]
        #   A2, B2, C2 = path[-2][1] - path[-3][1], path[-2][0] - path[-3][0], path[-3][0]*path[-2][1] - path[-2][0]*path[-3][1]
        #   angle = np.arctan2(abs( (A2*B1 - A1*B2) / (A1*A2 + B1*B2) ))
        #   if angle >= np.pi / 4:


        # print(step)
        path_goal_distance = np.hypot(path[-1][0] - goal[0], path[-1][1] - goal[1])
        # print(path_goal_distance)
        switch_point = None
        
        if flag == True:
          if path_goal_distance <= 25.0:
            switch_point = path[-1]
            # last_point = path[-1]
            print("Switch: ", switch_point)
            # plt.plot(switch_point[0], switch_point[1], 'ro', label="Switch")
            break

        else:
          if path_goal_distance <= thresh:
            # print("Converging step: ", step)
            # print("Final position error: ", path_goal_distance)
            # last_point = path[-1]

            break
  
    last_point = path[-1]
    return np.array(path), last_point, switch_point



def new_path(delx, dely, goal, goal_decoy, original_path, obstacle, last_point, flag,ax):
    obstacle, direction, new_start_point, contact_index = obstacle_direction(original_path, obstacle, s_obs, goal, goal_decoy, ax)
    delx, dely = goals_plus_obstacle(X,Y,delx, dely, obstacle,direction)
    new_path, last_point, switch_point = get_path_from_field(start_point,goal,goal_decoy,delx,dely, step_size, thresh, flag)


  # if flag == True:
  #   delx, dely = all_goals(X,Y,s_decoy,r_decoy,s_goal,r_goal,goal_decoy,goal)
  #   new_path, flag = get_path_from_field(start_point,goal_decoy,goal,delx,dely, flag)
  #   contact_index = None
    return delx, dely, new_path, contact_index, last_point, switch_point

# def obstacle_avoidance(delx, dely, previous_path, obstacle, goal, goal_decoy):
#   obstacle, direction, new_start_point, contact_index = obstacle_direction(previous_path, obstacle, s_obs)
#   delx, dely = goals_plus_obstacle(X,Y,delx,dely,obstacle,direction)
#   new_path, flag, _ = get_path_from_field(start_point,goal,goal_decoy,delx,dely, step_size, thresh, flag)
#   return delx, dely, new_path, contact_index, flag

# def switch_goal(new_goal, previous_goal, new_s, previous_s, new_r, previous_r, start_point, flag):
#   # delx = np.zeros_like(X)
#   # dely = np.zeros_like(Y)
#   delx, dely = all_goals(X,Y,new_s,new_r,previous_s,previous_r,new_goal,previous_goal)
#   new_path, last_point, switch_point = get_path_from_field(start_point,new_goal,previous_goal,delx, dely, step_size, thresh, flag)
#   return delx, dely, new_path, last_point, switch_point

def part_1(goal, goal_decoy, s_goal, r_goal, s_decoy, r_decoy, start_point, ax):
  
  flag = False
  s_decoy = np.sqrt((start_point[0]-goal_decoy[0])**2 + (start_point[1]-goal_decoy[1])**2)
  delx_zero, dely_zero = all_goals(X,Y,s_goal,r_goal,s_decoy,r_decoy,goal,goal_decoy)
  # delx_zero, dely_zero = all_goals(X,Y,s_goal/10,r_goal/10,s_decoy/10,r_decoy/10,goal,goal_decoy)
  path_zero, last_point, switch_point = get_path_from_field(start_point,goal,goal_decoy,delx_zero,dely_zero, step_size, thresh, flag=False)

  path = path_zero
  delx, dely = delx_zero, dely_zero

# delx, dely, path, contact_index, last_point = new_path(delx, dely, path, obstacles, last_point)

# delx, dely, path_2, flag, last_point = switch_goal(goal,goal_decoy,s_goal,s_decoy,r_goal,r_decoy, last_point)

# path = path + path_2


# if np.hypot(last_point,goal) > r_goal:
#   goal, goal_decoy = goal_decoy, goal
#   s_goal,r_goal,s_decoy,r_decoy = s_decoy, r_decoy, s_goal, r_goal
#   delx, dely, path, contact_index, flag = switch_goal(goal,goal_decoy,s_goal,s_decoy,r_goal,r_decoy)

  if obstacles is not None:
    for obstacle in obstacles:
      delx, dely, path, contact_index, last_point, switch_point = new_path(delx, dely, goal, goal_decoy, path,obstacle, last_point, False, ax) # Change to True if switch goals
      # delx, dely, path, contact_index, flag = obstacle_avoidance(delx, dely, path, obstacle, goal, goal_decoy)

  # plot_graph_g(X,Y,delx,dely,'Goal', 'Decoy Goal', 'Obstacle', fig, ax, goal, goal_decoy, obstacles, r_goal, r_decoy, r_obs, 'b', 'r','g', u, v)
  # print("Done part 1")

  return delx, dely, path_zero, path, switch_point,ax

def part_2(goal, goal_decoy, s_goal, r_goal, s_decoy, r_decoy, start_point, ax):
  
  flag = False
  s_decoy = np.sqrt((start_point[0]-goal_decoy[0])**2 + (start_point[1]-goal_decoy[1])**2)
  delx_zero, dely_zero = all_goals(X,Y,s_goal,r_goal,s_decoy,r_decoy,goal,goal_decoy)
  path_zero, last_point, switch_point = get_path_from_field(start_point,goal,goal_decoy,delx_zero,dely_zero, step_size, thresh, flag=False)

  path = path_zero
  delx, dely = delx_zero, dely_zero

# delx, dely, path, contact_index, last_point = new_path(delx, dely, path, obstacles, last_point)

# delx, dely, path_2, flag, last_point = switch_goal(goal,goal_decoy,s_goal,s_decoy,r_goal,r_decoy, last_point)

# path = path + path_2


# if np.hypot(last_point,goal) > r_goal:
#   goal, goal_decoy = goal_decoy, goal
#   s_goal,r_goal,s_decoy,r_decoy = s_decoy, r_decoy, s_goal, r_goal
#   delx, dely, path, contact_index, flag = switch_goal(goal,goal_decoy,s_goal,s_decoy,r_goal,r_decoy)

  # if obstacles is not None:
  #   for obstacle in obstacles:
  #     delx, dely, path, contact_index, last_point, switch_point = new_path(delx, dely, path,obstacle, last_point, False,ax)
  #     # delx, dely, path, contact_index, flag = obstacle_avoidance(delx, dely, path, obstacle, goal, goal_decoy)
  # delx, dely, path, contact_index, last_point, switch_point = new_path(delx, dely, goal, goal_decoy, path,[30.0,40.0], last_point, False, ax)

  # plot_graph_g(X,Y,delx,dely,'Goal', 'Decoy Goal', 'Obstacle', fig, ax, goal, goal_decoy, obstacles, r_goal, r_decoy, r_obs, 'b', 'r','g', u, v)
  # print("Done part 2")

  return delx, dely, path_zero, path, switch_point,ax




fig, ax = plt.subplots(figsize = (10,10))


delx, dely, path_zero, path_ours, switch_point,ax = part_1(goal, goal_decoy, s_goal/10, r_goal/10, s_decoy/10, r_decoy/10, start_point, ax)
# path_ours = path_ours[::int(np.ceil(len(path_ours) / 65))]
plot_graph_g(X/10,Y/10,delx,dely,'Goal', 'Decoy Goal', 'Obstacle', fig, ax, loc_g=[4.0,6.0], loc_d=[2.0,6.0], loc_o=[[4.1,2.0],[1.5,4.0]], r_goal=r_goal/10, r_decoy=r_decoy/10, r_obs=r_obs/10, color_g='b', color_d='r',color_o='k', u=u, v=v, start_point=[3.0,0.0])
# plot_graph_g(X,Y,delx,dely,'Goal', 'Decoy Goal', 'Obstacle', fig, ax, goal, goal_decoy, obstacles, r_goal=r_goal/10, r_decoy=r_decoy/10, r_obs=r_obs/10, color_g='b', color_d='r',color_o='k', u=u, v=v, start_point=[3.0,0.0])
# graph_1, = ax.plot(path_ours[:,0]/10, path_ours[:,1]/10, '-',color='b', linewidth=4, label="Ours")
# graph_1, = ax.plot(path_ours[:,0], path_ours[:,1], '-',color='b', linewidth=4, label="Ours")
path_zero = np.loadtxt("path.txt")
# path_zero /= 10
# graph_5, = ax2.plot(path_zero[:,0]/10, path_zero[:,1]/10, '--', color='b', linewidth=4, label="Ours \nw/o obstacle-aware")
# graph_5, = ax2.plot(path_zero[:,0]/10, path_zero[:,1]/10, '--', color='b', linewidth=4, label="Ours \nw/o Goal Switch")
graph_5, = ax.plot(path_zero[:,0]/10, path_zero[:,1]/10, '-', color='b', linewidth=4, label="Path")
# # # np.savetxt("path_1obs.txt", path_ours)

# path_pf = np.loadtxt('path_pf_1obs.txt',dtype=float)
# path_pf /= 10
# path_pf_00 = np.loadtxt('path_pf.txt',dtype=float)
# path_pf_00 /= 10
# path_pf = path_pf[::int(np.ceil(len(path_pf) / 30))]
# idx = np.round(np.linspace(0, len(path_pf) - 1, 65)).astype(int)
# path_pf = path_pf[idx]
# path_pf_00 = path_pf_00[idx]
# print(len(path_pf))
# graph_2, = ax.plot(path_pf[:,0], path_pf[:,1], '-', color='maroon', linewidth=4, label="Potential Field")
# graph_20, = ax2.plot(path_pf_00[:,0], path_pf_00[:,1], '--', color='maroon', linewidth=4, label="Potential Field \nw/o obstacle-aware")
# graph_20, = ax2.plot(path_pf_00[:,0], path_pf_00[:,1], '--', color='maroon', linewidth=4, label="Potential Field \nw/o Goal Switch")

# # # delx, dely, path_zero, path_pf, switch_point,ax = part_1_pf(goal, goal_decoy, s_goal, r_goal, s_decoy, r_decoy, start_point, ax)
# # # path_pf = path_pf[::int(np.ceil(len(path_pf) / 30))]
# # # graph_2, = ax.plot(path_pf[:,0], path_pf[:,1], 'm-', linewidth=4, label="Potential Field")
# # # # np.savetxt("path_pf_.txt", path_pf)

# baseline_anca = np.loadtxt('path_anca_1obs.txt',dtype=float)
# baseline_anca /= 10
# baseline_anca_00 = np.loadtxt('path_anca.txt',dtype=float)
# baseline_anca_00 /= 10
# baseline_anca = baseline_anca[::int(np.ceil(len(baseline_anca) / 30))]
# graph_0, = ax.plot(baseline_anca[:,0], baseline_anca[:,1], '-',color='orange', linewidth=4, label="Legible Baseline")
# graph_00, = ax2.plot(baseline_anca_00[:,0], baseline_anca_00[:,1], '--',color='orange', linewidth=4, label="Legible Baseline \nw/o obstacle-aware")
# graph_00, = ax2.plot(baseline_anca_00[:,0], baseline_anca_00[:,1], '--',color='orange', linewidth=4, label="Legible Baseline \nw/o Goal Switch")

# plt.xticks(fontsize=15)
# plt.yticks(fontsize=15)


# path_pf = np.loadtxt('path_pf_1obs.txt',dtype=float)
# # path_pf = path_pf[::int(np.ceil(len(path_pf) / 30))]
# idx = np.round(np.linspace(0, len(path_pf) - 1, 30)).astype(int)
# path_pf = path_pf[idx]
# print(len(path_pf))
# graph_2, = ax.plot(path_pf[:,0], path_pf[:,1], 'm-', linewidth=4, label="Potential Field")








# ax2.set_xlim(0,7)
# ax2.set_ylim(0,7)
# goal, goal_decoy = goal_decoy, goal
# s_goal, r_goal, s_decoy, r_decoy = s_decoy, r_decoy, s_goal, r_goal
# delx, dely, path_zero, path_2, _,ax2 = part_2(goal, goal_decoy, s_goal, r_goal, s_decoy, r_decoy, switch_point, ax2)
# # print(path_2)

# plot_graph_g_switch(X/10,Y/10,delx,dely,'Goal', 'Decoy Goal', 'Obstacle', fig2, ax2, [2.0,6.0], [4.0,6.0], [[15.0,35.0]], r_goal/10, r_decoy/10, r_obs/10, 'b', 'r','k', u, v, [3.0,0.0], [4.722721749, 3.641198184])

# # path_ours /= 10
# # path_2 /= 10

# temp3 = np.loadtxt("path_pf_switch_final.txt")
# temp3 /= 10
# # idx = np.round(np.linspace(0, len(temp3) - 1, 15)).astype(int)
# # temp3 = temp3[idx]
# # path_total_pf = np.vstack((path_ours,temp3))
# path_total_pf = temp3
# graph_3, = ax2.plot(path_total_pf[:,0], path_total_pf[:,1], '-',color='maroon', linewidth=4, label="Potential Field")

# temp4 = np.loadtxt("path_anca_switch_final.txt")
# temp4 /= 10
# # path_total_anca = np.vstack((path_ours,temp4))
# path_total_anca = temp4
# graph_4, = ax2.plot(path_total_anca[:,0], path_total_anca[:,1], '-', color='orange', linewidth=4, label="Legible Baseline")
# # np.savetxt("path_switch_later.txt",path_2)

# # idx = np.round(np.linspace(0, len(path_2) - 1, 15)).astype(int)
# # path_2 = path_2[idx]
# # print(len(path_2))
# # path_total_ours = np.vstack((path_ours,path_2))
# # print(path_ours)
# # graph_2, = ax2.plot(path_total_ours[:,0], path_total_ours[:,1], 'b-', linewidth=4, label="Ours")
# path_total_ours = np.loadtxt("path_switch.txt")
# path_total_ours /= 10
# graph_2, = ax2.plot(path_total_ours[:,0], path_total_ours[:,1], 'b-', linewidth=4, label="Ours")

# ax2.plot(5.28011507655277, 3.72413793103493, 'r*', markersize=20, label='')
# ax2.plot(0.3609299999999996089e+01, 0.3513400000000000745e+01, 'r*', markersize=20, label='')

# plt.xticks(fontsize=15)
# plt.yticks(fontsize=15)



# Switch point:  [47.22721749 36.41198184]

# if switch_flag == True:
#   # goal, goal_decoy = goal_decoy, goal
#   # s_goal, r_goal, s_decoy, r_decoy = s_decoy, r_decoy, s_goal, r_goal
#   delx, dely = all_goals(X,Y,s_decoy,r_decoy,s_goal,r_goal,goal_decoy,goal)
#   path, switch_flag = get_path_from_field(start_point,goal_decoy,goal,delx,dely,switch_flag)

  

# plot_graph_g(X,Y,delx,dely,'Goal', 'Decoy Goal', 'Obstacle', fig, ax, goal, goal_decoy, obstacles, r_goal, r_decoy, r_obs, 'b', 'r','g', u, v)

# ax.plot(path_zero[:, 0], path_zero[:, 1], 'g-', linewidth=3, label='Path with no Obstacles')


# # ax.axline((path[500,0], path[500,1]), (path[499,0], path[499,1]), label='Heading')
# graph_1, = ax.plot([], [], 'r-', linewidth=4, label="New Path")
# x_final_1 = path[:,0]
# y_final_1 = path[:,1]

# graph_2, = ax.plot([], [], 'g-', linewidth=4, label="Original Path")
# x_final_2 = path_zero[:,0]
# y_final_2 = path_zero[:,1]

# # # line, = ax.plot([],[],linewidth=2, label='Heading')

# # # def init():
# # #   line.set_data([],[])
# # #   return line,

# def animate_1(i):
#     graph_1.set_data(x_final_1[:i+1], y_final_1[:i+1])
#     graph_2.set_data(x_final_2[:i+1], y_final_2[:i+1])
#     # if i >= 2:
#     #   # line.set_data([path[i,0], path[i-1,0]], [path[i,1], path[i-1,1]])
#     #   line, = ax.plot([path[i,0], path[i-1,0]], [path[i,1], path[i-1,1]], label="Heading")
#     #   # line = ax.plot([x_final_1[i], y_final_1[i]], [x_final_1[i-1], y_final_1[i-1]], label="Heading")
#     return graph_1, graph_2


# ani = FuncAnimation(fig, animate_1, frames=len(path_zero), interval=1)


# ax.streamplot(X,Y,delx,dely, color='red', start_points=[start_point],linewidth=4, cmap='auto')
# print(path)
# print(type(path))
# np.savetxt('path.txt', path)
# with open("path.txt", 'w') as output:
#     for row in path:
#         output.write(str(row) + '\n')

# test = []

# # file = open("path.txt", "r")
# # for line in file.readlines():
# #   test.append(line.split(' '))
# test = np.loadtxt('path.txt',dtype=float)

# print(np.array(test).shape)

# graph_1, = ax.plot(test[:,0], test[:,1], 'b-', linewidth=4, label="Path")


plt.legend(loc='lower left', fontsize=14)
# plt.savefig("Robot Paths with " + str(0) + " Obstacle(s) Switch")
# plt.savefig("Example Path")
# plt.savefig("Robot Paths with " + str(len(obstacles)) + " Obstacle(s)")
# plt.savefig("Environment Setup")
plt.show()