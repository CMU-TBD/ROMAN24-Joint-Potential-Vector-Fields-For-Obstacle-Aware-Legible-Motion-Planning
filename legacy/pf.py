
import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import CubicSpline
from math import exp
from matplotlib.animation import FuncAnimation
from scipy.integrate import quad
from scipy.interpolate import interp1d

# import new_leb

x = np.arange(-0,70,1)
y = np.arange(-0,70,1)


# Goal is at (40,40) 
goal = [40,60]

#obstacle is at(25,25)
obstacle = [41,20]
# obstacle = [0,0]
X, Y = np.meshgrid(x,y)

delx = np.zeros_like(X)
dely = np.zeros_like(Y)

"""
    Inside the nested loop, distance from each point to the goal and ostacle is 
    calculated, Similarly angles are calculated. Then I simply used the formula give and 
    superimposed it to the Goals potential field.Also
    α = 50
    β = 50
    s = 15
    r = 2
"""
s_obs = 10
r_obs = 1
r_goal= 2.5
goal_decoy = [20,60]
goal, goal_decoy = goal_decoy, goal

step_size = 0.001 #0.0001
thresh = r_goal
field_strength = 50 # 50

for i in range(len(x)):
  for j in range(len(y)):
    
    # finding the goal distance and obstacle distance
    d_goal = np.sqrt((goal[0]-X[i][j])**2 + ((goal[1]-Y[i][j]))**2)
    d_obstacle = np.sqrt((obstacle[0]-X[i][j])**2 + (obstacle[1]-Y[i][j])**2)
    #print(f"{i} and {j}")

    #finding theta correspoding to the goal and obstacle 
    theta_goal= np.arctan2(goal[1] - Y[i][j], goal[0]  - X[i][j])
    theta_obstacle = np.arctan2(obstacle[1] - Y[i][j], obstacle[0]  - X[i][j])

    # if d_obstacle < r_obs:
    #   delx[i][j] = np.sign(np.cos(theta_obstacle)) +0
    #   dely[i][j] = np.sign(np.cos(theta_obstacle))  +0
    # elif d_obstacle>1+s_obs:
    #   delx[i][j] = 0 +(200 * s_obs *np.cos(theta_obstacle))
    #   dely[i][j] = 0 + (200 * s_obs *np.sin(theta_goal))
    # elif d_obstacle<1+s_obs :
    #   delx[i][j] = -200 *(s_obs+r_obs-d_obstacle)* np.cos(theta_obstacle)
    #   dely[i][j] = -200 * (s_obs+r_obs-d_obstacle)*  np.sin(theta_obstacle) 

    # if d_goal <r+s:
    #   if delx[i][j] != 0:
    delx[i][j]  += (20 * d_goal *np.cos(theta_goal))
    dely[i][j]  += (20 * d_goal *np.sin(theta_goal))
    #   else:
    #     delx[i][j]  = (50 * (d_goal-r) *np.cos(theta_goal))
    #     dely[i][j]  = (50 * (d_goal-r) *np.sin(theta_goal))
        
    # if d_goal>r+s:
    #   if delx[i][j] != 0:
    #     delx[i][j] += 50* s *np.cos(theta_goal)
    #     dely[i][j] += 50* s *np.sin(theta_goal)
    #   else:
    #     delx[i][j] = 50* s *np.cos(theta_goal)
    #     dely[i][j] = 50* s *np.sin(theta_goal)

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
        print(path_goal_distance)
        switch_point = None
        
        if flag == True:
          if path_goal_distance <= 25.0:
            switch_point = path[-1]
            # last_point = path[-1]
            print("Switch: ", switch_point)
            plt.plot(switch_point[0], switch_point[1], 'ro', label="Switch")
            break

        else:
          if path_goal_distance <= thresh:
            # print("Converging step: ", step)
            # print("Final position error: ", path_goal_distance)
            # last_point = path[-1]

            break
  
    last_point = path[-1]
    return np.array(path), last_point, switch_point

fig, ax = plt.subplots(figsize=(10,10))

seek_points = np.array([[3.609299999999996089e+01, 3.513400000000000745e+01]]) 
ax.plot(seek_points[0][0], seek_points[0][1], 'o', color='gray', markersize=20, label='Start Point')
path, last_point, _ = get_path_from_field(seek_points[0],goal,goal_decoy,delx,dely,step_size,thresh)
np.savetxt("path_pf_switch_final.txt",path)
ax.plot(path[:,0],path[:,1])

# ax.streamplot(X,Y,delx,dely, start_points=seek_points)
ax.quiver(X, Y, delx, dely)
ax.add_patch(plt.Circle((obstacle[0], obstacle[1]), r_obs, color='y'))
ax.add_patch(plt.Circle((goal[0], goal[1]), r_goal, color='m'))

ax.annotate("Obstacle", xy=(obstacle[0], obstacle[1]), fontsize=8, ha="center")
ax.annotate("Goal", xy=(goal[0], goal[1]), fontsize=10, ha="center")

ax.set_title('Path taken by the Robot ')

plt.show()