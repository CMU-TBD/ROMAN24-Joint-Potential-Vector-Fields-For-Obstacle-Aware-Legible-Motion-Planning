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

# start_point = [30.0,0.0]  #30
start_point = [47.22721749, 36.41198184]
goal = [20.0,60.0] # 40 60
goal_decoy = [40.0,60.0] # 20 60
# goal = [4.0, 6.0]
# goal_decoy = [2.0, 6.0]
# start_point = [3.0, 0.0]
goals = [goal,goal_decoy]

def get_dist(p0, p1):
  p0_x, p0_y = p0
  p1_x, p1_y = p1

  min_distance = np.sqrt((p0_x-p1_x)**2 + (p0_y-p1_y)**2)
  return min_distance

def f_path_cost(path):
	cost = 0
	for i in range(len(path) - 1):
		cost = cost + f_cost(path[i], path[i + 1]) # i+1???

	return cost


def get_min_direct_path_cost_between(p0, p1):
  dist = get_dist(p0, p1)
  
  dt = 0.025 # 0.025
  
  cost_chunk = dt * dt
  num_chunks = int()

  leftover = dist - (dt*num_chunks)
  cost = (num_chunks * cost_chunk) + (leftover*leftover)

  

  return cost

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
	return np.abs(a * a)
  # return np.abs(a)

def unnormalized_prob_goal_given_path(pt, goal, path, is_og):
  decimal.getcontext().prec = 60
  is_og = True

  if is_og:
    c1 = decimal.Decimal(f_path_cost(path))
  else:
    c1 = decimal.Decimal(get_min_direct_path_cost_between(start_point, pt))	

  
  c2 = decimal.Decimal(get_min_direct_path_cost_between(pt, goal))
  c3 = decimal.Decimal(get_min_direct_path_cost_between(start_point, goal))

  # print(c2)
  # print(c3)
  a = np.exp((-c1 + -c2))
  b = np.exp(-c3)
  # print(a)
  # print(b)

  ratio 		= a / b

  if math.isnan(ratio):
    ratio = 0

  return ratio

def prob_goal_given_path(pt, goal, goals, path):
	# entry = []

	g_array = []
	g_target = 0
	for g in goals:
		p_raw = unnormalized_prob_goal_given_path(pt, g, path, is_og=True)
		g_array.append(p_raw)
		if g == goal:
			# print('target val ' + str(p_raw))
			g_target = p_raw

	if(sum(g_array) == 0):
		print("weird g_array")
		return decimal.Decimal(1.0)

	# print(g_array)

	return decimal.Decimal(g_target / (sum(g_array)))

def f_exp_single_normalized(t, pt, aud, path):
	# if this is the omniscient case, return the original equation
	# if len(aud) == 0 and path is not None:
  return float(len(path) - t + 1)
  # return float(len(path) * np.exp(-t))
		# return float(len(path) - t)
	# elif len(aud) == 0:
	# 	# print('ping')
	# 	return 1.0

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

def f_legibility(goal, goals, path, aud=[], f_function=f_exp_single_normalized):
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

  if path is None or len(path) == 0:
    return 0

  legibility = decimal.Decimal(0)
  divisor = decimal.Decimal(0)
  total_dist = decimal.Decimal(0)

  # if exp_settings is not None and 'lambda' in exp_settings and exp_settings['lambda'] != '':
  #   LAMBDA = decimal.Decimal(exp_settings['lambda'])
  #   epsilon = decimal.Decimal(exp_settings['epsilon'])
  # else:
  #   # TODO verify this
  LAMBDA = 1.0
  epsilon = 1.0

  start = path[0]
  total_cost = decimal.Decimal(0)
  aug_path = get_costs_along_path(path)

  path_length_list, length_of_total_path = get_path_length(path)
  length_of_total_path = decimal.Decimal(length_of_total_path)

  delta_x = decimal.Decimal(1.0) #length_of_total_path / len(aug_path)

  t = 1
  p_n = path[0]
  divisor = decimal.Decimal(epsilon)
  numerator = decimal.Decimal(0.0)

  f_log = []
  p_log = []

  for pt, cost_to_here in aug_path:
    f = decimal.Decimal(f_function(t, pt, aud, path))

    # Get this probability from all the available signals
    # probs_array_goal_given_signals = prob_array_goal_given_signals(r, p_n, pt, goal, goals, cost_to_here, exp_settings)
    prob_goal_signals_fused = prob_goal_given_path(pt,goal, goals, path)

    # # combine them according to the exp settings
    # prob_goal_signals_fused = prob_overall_fuse_signals(probs_array_goal_given_signals, r, p_n, pt, goal, goals, cost_to_here, exp_settings)


    # Then do the normal methods of combining them
    f_log.append(float(f))
    p_log.append(prob_goal_signals_fused)


    if len(aud) == 0: # FLAG_is_denominator or 
      numerator += (prob_goal_signals_fused * f) # * delta_x)
      divisor += f #* delta_x
    else:
      numerator += (prob_goal_signals_fused * f) # * delta_x)
      divisor += decimal.Decimal(1.0) #* delta_x

    t = t + 1
    total_cost += decimal.Decimal(f_cost(p_n, pt))
    p_n = pt

  if divisor == 0:
    legibility = 0
  else:
    legibility = (numerator / divisor)

  total_cost =  - decimal.Decimal(LAMBDA)*total_cost
  # overall = legibility + total_cost


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

  

  return legibility #overall






path = []
# file = open("path.txt", "r")
# for line in file.readlines():
#   line = np.fromstring(line)
#   path.append(line.split(' '))
path = np.loadtxt('path_anca.txt',dtype=float)
idx = np.round(np.linspace(0, len(path) - 1, 30)).astype(int)
path_test = path[idx][:]
# path_test = path[:5]

# time = np.arange(len(path_test))

# l_s = []
# l_d = []
# for i in range(len(path_test)):
#   l_s.append(f_legibility(goal,goals,path_test[:i]))
#   l_d.append(f_legibility(goal_decoy,goals,path_test[:i]))

# ave_l = np.mean(l_s)
# print(ave_l)
print(get_path_length(path_test)[1])
# # print(l_s)
# plt.plot(l_s)

# print(len(l_s))
# plt.plot(path_test[:,0], path_test[:,1])
# plt.xlim(0,70)
# plt.ylim(0,70)
# plt.plot(time,l_s,color='r',label="Real")
# plt.plot(time,l_d,color='b',label="Fake")


# print(time)

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
plt.show()