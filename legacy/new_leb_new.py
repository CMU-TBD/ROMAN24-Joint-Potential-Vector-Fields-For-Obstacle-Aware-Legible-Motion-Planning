import copy
import decimal
import numpy as np
# import autograd.numpy as np
import math
import pandas as pd
import matplotlib.pylab as plt

# import utility_environ_descrip 		as resto
# import utility_path_segmentation 	as chunkify


# FUNCTIONS FOR CALCULATING FEATURES OF PATHS
# SUCH AS VISIBLIITY, LEGIBILITY, PATH_LENGTH, and ENVELOPE
np.random.seed(0)

F_JDIST 				= 'JDIST'
F_JHEADING_EXPONENTIAL 	= 'JHEAD_EXPON'
F_JHEADING_QUADRATIC 	= 'JHEAD_QUADR'
F_JHEADING 				= 'JHEAD'
F_SUM_DIST_EXPON 		= 'SUM_DIST_EXPON'
F_MIN_DIST_EXPON 		= 'MIN_DIST_EXPON'

LEGIBILITY_METHOD		= 'l_method'

PROB_INDEX_DIST = 0
PROB_INDEX_HEADING = 1
div = 1
# start_point = [30.0/10,0.0]  #30
# goal = [40.0/10,60.0/10] # 40 60
# goal_decoy = [20.0/10,60.0/10] # 20 60
# start_point = [3.0,0.0]  #30
# goal = [4.0,6.0] # 40 60
# goal_decoy = [2.0,6.0] # 20 60
# print(get_dist(start_point,goal))
# print(start_point,goal,goal_decoy)
# goal = [4.0, 6.0]
# goal_decoy = [2.0, 6.0]
# start_point = [3.0, 0.0]
# goals = [goal,goal_decoy]

def get_dist(p0, p1):
  p0_x, p0_y = p0
  p1_x, p1_y = p1

  min_distance = np.sqrt((p0_x-p1_x)**2 + (p0_y-p1_y)**2)
  return min_distance

# print(get_dist(start_point,goal))

# normal = get_dist(start_point,goal) + get_dist(start_point, goal_decoy) + get_dist(goal, goal_decoy)

def f_path_cost(path):
  cost = 0
  for i in range(len(path) - 1):
    # normal = get_dist(path[i],goal_decoy) / get_dist(path[i],goal)
    # normal = 1
    cost = cost + f_cost(path[i], path[i + 1]) # i+1???
    # normal = f_cost(path[i],goal) + f_cost(path[i],goal_decoy) + f_cost(goal,goal_decoy)
    # normal = get_dist(path[i],goal) + get_dist(path[i],goal_decoy) + get_dist(goal,goal_decoy)
    # normal = 1


  return cost


def get_min_direct_path_cost_between(p0, p1):
  dist = get_dist(p0, p1)
  
  # dt = 0.025
  dt = 1.0
  
  cost_chunk = dt * dt
  num_chunks = int()

  leftover = dist - (dt*num_chunks)
  cost = (num_chunks * cost_chunk) + 0.5*(leftover*leftover) * dt

  # normal = get_dist(p0,goal) + get_dist(p0,goal_decoy) + get_dist(goal,goal_decoy)
  # normal = 1
  # normal = f_cost(p0,goal) + f_cost(p0,goal_decoy) + f_cost(goal,goal_decoy)
  # normal = get_min_direct_path_cost_between(p0,goal) + get_min_direct_path_cost_between(p0,goal_decoy) + get_min_direct_path_cost_between(goal,goal_decoy)
  # normal = get_dist(p0,goal_decoy) / get_dist(p0,goal)

  return cost # cost

def get_min_direct_path_length(r, p0, p1):
	return get_dist(p0, p1)

def get_costs_along_path(path):
  output = []
  ci = 0
  csf = 0
  for pi in range(len(path)):
    # print(pi, ci)
    # print(path[ci], path[pi])
    cst = f_cost(path[ci], path[pi])
    csf = csf + cst
    log = (path[pi], csf)
    ci = pi
    output.append(log)
    
  return output

def dist(p0, p1):
	return np.sqrt((p0[0] - p1[0])**2 + (p0[1] - p1[1])**2)

def f_cost(t1, t2):
  a = dist(t1, t2)
  # return a
  return 0.5*np.abs(a * a)
  # return 0.5 * a * a

def unnormalized_prob_goal_given_path(pt, goal, path,start_point, is_og=True):
  # decimal.getcontext().prec = 60
  # is_og = False

  if is_og:
    c1 = decimal.Decimal(f_path_cost(path))
    # c1 = decimal.Decimal()
    # c1 = decimal.Decimal(get_path_length(path)[1])
    # print('not first')
  else:
    c1 = decimal.Decimal(get_min_direct_path_cost_between(start_point, pt))	
    # c1 = decimal.Decimal(0.5 * ((start_point[0]-pt[0])**2 + (start_point[1]-pt[1])**2))
    # print('first')
    # c1 = decimal.Decimal(0.0)
  # print('c1',c1)
  # print(c1)
  
  c2 = decimal.Decimal(get_min_direct_path_cost_between(pt, goal))
  # c2 = decimal.Decimal(0.5 * ((pt[0]-goal[0])**2 + (pt[1]-goal[1])**2))
  c3 = decimal.Decimal(get_min_direct_path_cost_between(start_point, goal))
  # c3 = decimal.Decimal(0.5 * ((start_point[0]-goal[0])**2 + (start_point[1]-goal[1])**2))

  # print('c2', c2)
  # print('c3', c3)
  a = np.exp((-c1-c2)) 
  b = np.exp(-c3)

  ratio 		= a / b
  # print('ratio', ratio)

  # if math.isnan(ratio):
  #   ratio = 0

  return ratio * decimal.Decimal(0.5)# * decimal.Decimal(0.01)


def prob_goal_given_path(pt, goal, goals, path,start_point, is_og=True):
  # entry = []

  g_array = []
  g_target = 0
  for g in goals:
    p_raw = unnormalized_prob_goal_given_path(pt, g, path,start_point, is_og=is_og)
    g_array.append(p_raw)
    # if g == goal:
    if get_dist(g,goal) <= 1e-5:
      # print('target val ' + str(p_raw))
      g_target = p_raw

  if(sum(g_array) == 0):
    print("weird g_array")
    return decimal.Decimal(1.0)
  
  # print('1', g_array[0] / sum(g_array))
  # print('2', g_array[1] / sum(g_array))

  return decimal.Decimal(g_target / (sum(g_array)))

def f_exp_single_normalized(t, pt, aud, path):
	# if this is the omniscient case, return the original equation
	# if len(aud) == 0 and path is not None:
  return float(len(path) - t + 1)
		# return float(len(path) - t)
	# elif len(aud) == 0:
	# 	# print('ping')
  # return 1.0

	# if in the (x, y) OR (x, y, t) case we can totally 
	# still run this equation
	# val = get_visibility_of_pt_w_observers(pt, aud, normalized=True)
	# return val

def f_path_length(t1, t2):
	a = dist(t1, t2)
	return a

def get_path_length(path):
  total = 0
  output = [0]

  for i in range(len(path) - 1):
    link_length = f_path_length(path[i], path[i + 1])
    total = total + link_length
    output.append(total)

  return output, total



def f_legibility(goal, goals, path, start_point, p_list=[], aud=[], f_function=f_exp_single_normalized):
	# if f_function is None:
	# 	f_function = f_exp_single_normalized

	# if exp_settings is not None:
	# 	FLAG_is_denominator = exp_settings['is_denominator']
	# else:
	# 	FLAG_is_denominator = True
	
	# if f_function is None and FLAG_is_denominator:
	# 	f_function = f_exp_single
	# elif f_function is None:
	# 	f_function = f_exp_single_normalized
  f_function = f_exp_single_normalized
  # print('path len', len(path))
  # print('path end', path[-1])

  # legibility = decimal.Decimal(0)
  # # if path is None or len(path) == 0:
  # #   print('return 0')
  # #   return 0
  # if len(path) == 0:
  #   legibility = decimal.Decimal(prob_goal_given_path(start_point,goal, goals, path,is_og=False))

  

  
  divisor = decimal.Decimal(0)
  total_dist = decimal.Decimal(0)

  # if exp_settings is not None and 'lambda' in exp_settings and exp_settings['lambda'] != '':
  #   LAMBDA = decimal.Decimal(exp_settings['lambda'])
  #   epsilon = decimal.Decimal(exp_settings['epsilon'])
  # else:
  #   # TODO verify this
  LAMBDA = 1.0
  # epsilon = 1.0
  epsilon = 1e-5

  # start = path[0]
  total_cost = decimal.Decimal(0)
  legibility = decimal.Decimal(0)
  # legibility += decimal.Decimal(0.5)
  # if len(path) == 0 or path is None:
  #   legibility += decimal.Decimal(0.5)
  #   print('legibility 0', legibility)
  # if len(path) == 0:
  #   # legibility = decimal.Decimal(prob_goal_given_path(start_point,goal, goals, path,is_og=False))
  #   legibility += decimal.Decimal(0.5)
    # total_cost =  - decimal.Decimal(LAMBDA)*total_cost

  # else:
  if len(path) != 0:
    aug_path = get_costs_along_path(path[:])

    path_length_list, length_of_total_path = get_path_length(path[:])
    length_of_total_path = decimal.Decimal(length_of_total_path)
    # print('path length', length_of_total_path)

    # delta_x = decimal.Decimal(1.0) 
    delta_x = length_of_total_path / len(aug_path)
    

    t = 1
    p_n = path[0]
    divisor = decimal.Decimal(epsilon)
    numerator = decimal.Decimal(0.0)
    # print('t', t) 

    f_log = []
    p_log = []



    for pt, cost_to_here in aug_path:
      f = decimal.Decimal(f_function(t, pt, aud, path))
      # print(f)


      # Get this probability from all the available signals
      # probs_array_goal_given_signals = prob_array_goal_given_signals(r, p_n, pt, goal, goals, cost_to_here, exp_settings)
      # if dist(start,pt) <= 1e-5:
      #   prob_goal_signals_fused = prob_goal_given_path(pt,goal, goals, path,is_og=False)

      # else:
      prob_goal_signals_fused = prob_goal_given_path(pt,goal, goals, path,start_point, is_og=True)
      # if len(p_list) == 0:
      #   p_list.append(prob_goal_signals_fused)
      # else:
      #   if prob_goal_signals_fused > p_list[-1]:
      #     p_list.append(prob_goal_signals_fused)
      # p_list.append(prob_goal_signals_fused)
      # print(prob_goal_signals_fused)
      # print(prob_goal_signals_fused)
      
        # t = t + 1
      # print('prob', prob_goal_signals_fused)
      

      # # combine them according to the exp settings
      # prob_goal_signals_fused = prob_overall_fuse_signals(probs_array_goal_given_signals, r, p_n, pt, goal, goals, cost_to_here, exp_settings)


      # Then do the normal methods of combining them
      f_log.append(float(f))
      p_log.append(prob_goal_signals_fused)
      # print('p_log', p_log)


      if len(aud) == 0: # FLAG_is_denominator or 
        numerator += (prob_goal_signals_fused * f)# * delta_x
        divisor += f #* delta_x
        # print('numerator', numerator)
        # print('divisor', divisor)
        # print('t', t)
        # print('nnumerator', divisor)
        # print(prob_goal_signals_fused)
      else:
        numerator += (prob_goal_signals_fused * f )# * delta_x)
        divisor += decimal.Decimal(1.0) #* delta_x
      # print('divisor', divisor)
      # print('t', t)

      # print(numerator/divisor)
      # print('numerator', numerator)
      # print('divisor', divisor)

      t = t + 1
      # print('next t', t)
      total_cost += decimal.Decimal(f_cost(p_n, pt))
      # print('total cost', total_cost)
      p_n = pt
      


    if divisor == 0:
      legibility = 0
    else:
      legibility = (numerator / divisor)
      # print('divisor', divisor)
      # print('t', t)
      # print('legibility',legibility)
      # print('prob',prob_goal_signals_fused)
    total_cost =  - decimal.Decimal(LAMBDA)*total_cost
    # print('total cost outside', total_cost)
    # print('\n')
    # overall = legibility + total_cost
    # print(legibility)


    if legibility > 1.0 or legibility < 0:
      print("BAD L ==> " + str(legibility))
      # r.get_obs_label(aud)
      # goal_index = r.get_goal_index(goal)
      # category = r.get_obs_label(aud)
      # bug_counter[goal_index, category] += 1

    # elif (legibility == 1):
      # goal_index = r.get_goal_index(goal)
      # category = r.get_obs_label(aud)
      # bug_counter[goal_index, category] += 1

      # print(len(aud))
      # if exp_settings['kill_1'] == True:
      #   overall = 0.0

    

  return legibility,p_log, length_of_total_path #overall


# normal = get_dist(start_point,goal) + get_dist(start_point, goal_decoy) + get_dist(goal, goal_decoy)



# start_point = [30.0/10,0.0]  #30
goal = [40.0/10,60.0/10] # 40 60
goal_decoy = [20.0/10,60.0/10] # 20 60
goals = [goal,goal_decoy]


path_anca = np.loadtxt('path_anca_1obs.txt',dtype=float)
# print(len(path_anca))
path_anca /= 10
test = np.loadtxt('path_anca.txt',dtype=float)
# print(len(path_anca))
test /= 10

path_pf = np.loadtxt('path_pf_1obs.txt',dtype=float)
path_pf /= 10
path = np.loadtxt('path_1obs.txt',dtype=float)
path /= 10

# path_pf = path_pf[::10]
# # print(len(path_pf))
# path = path[::10]


# min_idx = np.min([len(path_anca), len(path_pf), len(path)])
min_idx = np.min([len(path_anca), len(path_pf), len(path), len(test)])

# goal, goal_decoy = goal_decoy, goal
# start_point = [4.722721749, 3.641198184]
# min_idx = 5
# print(min_idx)

idx_anca = np.round(np.linspace(0, len(path_anca) - 1, min_idx)).astype(int)
idx_pf = np.round(np.linspace(0, len(path_pf) - 1, min_idx)).astype(int)
idx = np.round(np.linspace(0, len(path) - 1, min_idx)).astype(int)
idx_test = np.round(np.linspace(0, len(test) - 1, min_idx)).astype(int)

path_anca = path_anca[idx_anca]
path_pf = path_pf[idx_pf]
path = path[idx]
test = test[idx_test]

start_point = path[0]
start_point_pf = path_pf[0]
start_point_anca = path_anca[0]
start_point_test = test[0]


# plt.plot(path_anca[:,0],path_anca[:,1],'-o')
# plt.plot(test[:,0],test[:,1],'-o')
# plt.plot(path_pf[:,0],path_pf[:,1],'-o')
# plt.plot(path[:,0],path[:,1],'-o')
# print(path_anca[-1])
# print(path_pf[-1])
# print(path[-1])

time = np.arange(min_idx)

l_s_anca = []
l_d_anca = []
l_s_pf =[]
l_d_pf = []
l_s =[]
l_d = []
p_s_anca = []
p_d_anca = []
p_s_pf =[]
p_d_pf = []
p_s =[]
p_d = []
l_s_test = []
for i in range(0,min_idx):
  l_s_anca.append(f_legibility(goal,goals,path_anca[:i+1],start_point_anca)[0])
  l_s_test.append(f_legibility(goal,goals,test[:i+1],start_point_test)[0])
#   l_d_anca.append(f_legibility(goal_decoy,goals,path_anca[:i+1],start_point_anca)[0])
#   l_s_pf.append(f_legibility(goal,goals,path_pf[:i+1],start_point_pf)[0])
#   l_d_pf.append(f_legibility(goal_decoy,goals,path_pf[:i+1],start_point_pf)[0])
#   l_s.append(f_legibility(goal,goals,path[:i+1],start_point)[0])
#   l_d.append(f_legibility(goal_decoy,goals,path[:i+1],start_point)[0])

# print(l_s_anca[:6])
# print(l_s_test[:6])
# print(test[:6])
# print(path_anca[:6])
# thresh = -1
# p_s_anca = f_legibility(goal,goals,path_anca[:thresh],p_s_anca)[1]
# p_d_anca = f_legibility(goal_decoy,goals,path_anca[:thresh],p_d_anca)[1]
# p_s_pf = f_legibility(goal, goals,path_pf[:thresh],p_s_pf)[1]
# p_d_pf = f_legibility(goal_decoy,goals,path_pf[:thresh],p_d_pf)[1]
# p_s = f_legibility(goal,goals,path[:thresh],p_s)[1]
# p_d = f_legibility(goal_decoy,goals,path[:thresh],p_d)[1]

# # l, p_list = f_legibility(goal,goals,path_anca[:],[], f_exp_single_normalized, p_list=p_list)
# # l, p_list_d = f_legibility(goal_decoy,goals,path_anca[:],[], f_exp_single_normalized, p_list=p_list_d)
# # plt.plot(p_list[:30],'-',label)
# # plt.plot(p_list_d[:30],'-o')

# plt.plot(p_s_anca,'-o',color='orange',linewidth=3,label="Legible Baseline")
# plt.plot(p_d_anca,'-o',color='orange',linewidth=3, label="Legible Baseline - Other Goal")
# plt.plot(p_s_pf,'-o',color='maroon', linewidth=3,label="Potential Field")
# plt.plot(p_d_pf,'-o',color='maroon',linewidth=3,label="Potential Field - Other Goal")
# plt.plot(p_s,'-o', color='blue',linewidth=3,label="Ours")
# plt.plot(p_d,'-o',color='blue',linewidth=3,label="Ours - Other Goal")
# plt.legend(fontsize=12)
# plt.xlabel("Time",fontsize=15)
# plt.xticks(fontsize=13)
# plt.yticks(fontsize=13)
# plt.ylim(0.5,1)
# plt.ylabel("Probability",fontsize=15)
# plt.title("Probability over time - With Obstacle", fontsize=15)


### Path length
# thresh = -1
# len_s_anca = f_legibility(goal,goals,path_anca[:i+1],start_point_anca)[2]
# len_s_pf = f_legibility(goal,goals,path_pf[:i+1],start_point_pf)[2]
# len_s = f_legibility(goal,goals,path[:i+1],start_point)[2]
# print(len_s_anca,len_s_pf,len_s)
# len_s_anca = get_path_length(path_anca)[1]
# len_s_pf = get_path_length(path_pf)[1]
# len_s = get_path_length(path)[1]
# print(len_s_anca,len_s_pf,len_s)

# length_y = [len_s_pf, len_s_anca,len_s]
# length_x = ['Potential Field','Legible Baseline', 'Ours']
# # plt.bar(length_x,length_y)
# plt.figure(figsize=(8,8))
# plt.bar(length_x[0],length_y[0],width=0.7,edgecolor='black',color='maroon')
# plt.bar(length_x[1],length_y[1],width=0.7,edgecolor='black',color='orange')
# plt.bar(length_x[2],length_y[2],width=0.7,edgecolor='black',color='blue')
# plt.xlabel('\nMethod',fontsize=16)
# plt.ylabel('Path Length\n',fontsize=16)
# plt.xticks(fontsize=13)
# plt.yticks(fontsize=13)
# plt.title('Path Length - Goal Switch',fontsize=15)

# # N = 3
# len_pf = [6.09,6.30,2.97]
# len_anca = [7.19,7.48,4.60]
# len_ours = [6.63,6.76,3.88]
# r = np.arange(3)
# width = 0.25

# plt.figure(figsize=(8,8))

# plt.bar(r,len_pf,color='maroon', width=width, edgecolor='black',label="Potential Field")
# plt.bar(r + width,len_anca,color='orange', width=width, edgecolor='black',label="Legible Baseline")
# plt.bar(r + 2 * width,len_ours,color='blue', width=width, edgecolor='black',label="Ours")

# plt.xlabel("Case", fontsize=20) 
# plt.ylabel("Total Path Length",fontsize=20) 
# plt.title("Total Path Length in all cases",fontsize=20) 

# plt.xticks(r + width,['No Obstacle','With Obstacle','Goal Switch'],fontsize=15) 
# plt.yticks(fontsize=15) 
# plt.legend(fontsize=13) 



def auc(x_values, y_values):
    """
    Computes the area under the curve defined by the x and y values using the trapezoidal rule.
    - x_values: A list or array-like object containing the x values.
    - y_values: A list or array-like object containing the y values corresponding to the x values.
    
    Output:s
    - The area under the curve.
    """
    if len(x_values) != len(y_values):
        raise ValueError("x_values and y_values must have the same length")

    area = decimal.Decimal(0)
    for i in range(1, len(x_values)):
        height = (y_values[i - 1] + y_values[i]) / decimal.Decimal(2.0)
        width = x_values[i] - x_values[i - 1]
        area += width * height

    return area / 2


# print('AUC for Anca: {}'.format(auc(time,l_s_anca)-auc(time,l_d_anca)))
# print('AUC for PF: {}'.format(auc(time,l_s_pf)-auc(time,l_d_pf)))
# print('AUC for Ours: {}'.format(auc(time,l_s)-auc(time,l_d)))

# thresh = -1
# # plt.figure(figsize=(8,8))
# plt.plot(time[:thresh],l_s_anca[:thresh],'-',color='orange',linewidth=2,label="Legible Baseline")
# # plt.plot(time[:thresh],l_d_anca[:thresh],'-o',color='orange',linewidth=3, label="Legible Baseline - Other Goal")
# plt.plot(time[:thresh],l_s_pf[:thresh],'-',color='maroon', linewidth=2,label="Potential Field")
# # plt.plot(time[:thresh],l_d_pf[:thresh],'-o',color='maroon',linewidth=3,label="Potential Field - Other Goal")
# plt.plot(time[:thresh],l_s[:thresh],'-', color='blue',linewidth=2,label="Ours")
# # plt.plot(time[:thresh],l_d[:thresh],'-o',color='blue',linewidth=3,label="Ours - Other Goal")
# plt.legend(fontsize=13,loc='lower right')
# plt.xlabel("Time",fontsize=15)
# plt.xticks(fontsize=15)
# plt.yticks(fontsize=15)
# plt.ylim(0.5,1)
# plt.ylabel("Legibility",fontsize=17)
# plt.title("Legibility scores over time - Goal Switch", fontsize=17)


# print('Path Length Anca ', get_path_length(path_anca)[1])
# print('Path Length PF ', get_path_length(path_pf)[1])
# print('Path Length Ours ', get_path_length(path)[1])


# ratio = unnormalized_prob_goal_given_path(path_test[-1],goal, path_test, True)
# # print(ratio)
# prob = prob_goal_given_path(path_test[-1], goal, goals, path_test)
# print(prob)
# l_gs = []
# l_ds = []
# time = []
# uniform_g = np.random.uniform(low=0.0, high=1.0)

# prob_final = prob * decimal.Decimal(uniform_g)
# goal = [4.0, 6.0]
# goal_decoy = [2.0, 6.0]
# start_point = [3.0, 0.0]


# for i in range(len(path_test)):
#   l_g = unnormalized_prob_goal_given_path(path_test[i], goal)
#   l_d = unnormalized_prob_goal_given_path(path_test[i], goal_decoy)
#   sum = np.sqrt(l_g**2 + l_d**2)
#   l_gs.append(l_g/sum)
#   l_ds.append(l_d/sum)
#   time.append(i)


# plt.plot(time,l_gs)
# plt.savefig('Legibility No Obstacle Switch')
# plt.savefig('Total Path Length')
plt.show()