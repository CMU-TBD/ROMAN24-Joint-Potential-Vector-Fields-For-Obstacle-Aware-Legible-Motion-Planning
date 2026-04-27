# import numpy as np
# from scipy.integrate import quad
# from scipy.interpolate import CubicSpline
# from scipy.misc import derivative

# # Define the cost function based on the provided formula
# def cost_function(xi_prime):
#     return (1/2) * np.trapz(xi_prime**2, dx=1)

# # Define the probability function for G given xi
# def probability_G_given_xi(cost_s_to_q, cost_s_to_g, P_G):
#     numerator = np.exp(-cost_s_to_q - cost_s_to_g)
#     denominator = np.exp(-cost_s_to_g)
#     return numerator / denominator * P_G

# # Define the legibility function based on the provided formula
# def legibility(xi, T, P_G_star):
#     # First we create a function that represents the derivative of xi(t)
#     # Assuming xi is a function that returns a numpy array of positions at times t
#     def xi_prime(t):
#         # For a real use-case, xi(t) should be interpolated between waypoints
#         # Here we'll just use a dummy example where xi is the identity function
#         # This should be replaced with the actual xi'(t) computation
#         return derivative(xi, t, dx=1e-6)

#     # Compute the cost function for the path
#     cost_s_to_g = cost_function(xi_prime(np.linspace(0, T, num=100)))

#     # Compute the integral for the numerator of legibility
#     numerator_integral, _ = quad(lambda t: probability_G_given_xi(0, cost_s_to_g, P_G_star) * (T - t), 0, T)
    
#     # Compute the integral for the denominator of legibility
#     denominator_integral, _ = quad(lambda t: T - t, 0, T)
    
#     # Return the legibility score
#     return numerator_integral / denominator_integral

# # Define the waypoints and times for the path ξ(t)
# waypoints = [0, 1, 2, 3, 4, 5] # Example waypoints
# times = np.linspace(0, 10, num=len(waypoints)) # Example times

# # Interpolate a spline through the waypoints to estimate ξ(t)
# xi_spline = CubicSpline(times, waypoints)

# # Define the xi function that evaluates the spline at time t
# def xi(t):
#     return xi_spline(t)

# # Define a constant T and P(G)
# T = 10  # The end time of the path
# P_G = 1  # Assuming a uniform distribution for P(G)

# # Calculate the legibility score
# legibility_score = legibility(xi, T, P_G)
# print(legibility_score)

import numpy as np
from scipy.integrate import quad
from scipy.interpolate import interp1d

# Define the waypoints and times for the path ξ(t)
# Example waypoints in 2D space: (x, y)
waypoints = np.array([
    [0, 0],  # Starting position (x0, y0)
    [1, 2],  # Waypoint 1
    [2, 3],  # Waypoint 2
    [3, 3],
    [4, 3],
    [4, 4],
    [4, 5],
    # ... more waypoints ...
    [5, 5]   # Goal position (x_g, y_g)
])


times = np.linspace(0, 1, num=len(waypoints))  # Normalized times from 0 to 1

# Interpolate the path to create a continuous function of time for x and y
xi_x = interp1d(times, waypoints[:, 0], kind='cubic')
xi_y = interp1d(times, waypoints[:, 1], kind='cubic')

# Define the cost function for a given path xi(t) in 2D
def C(xi_x_prime, xi_y_prime, T):
    # Numerically integrate the square of the derivatives of xi_x and xi_y
    integrand_x = lambda t: xi_x_prime(t)**2
    integrand_y = lambda t: xi_y_prime(t)**2
    integral_x, _ = quad(integrand_x, 0, T)
    integral_y, _ = quad(integrand_y, 0, T)
    return 0.5 * (integral_x + integral_y)

# Define the derivative functions of xi_x(t) and xi_y(t) using finite differences
def derivative(f, t, dt=1e-6):
    return (f(t + dt) - f(t - dt)) / (2 * dt)

# Define the probability function for G given xi
def probability_G_given_xi(C_s_to_q, C_q_to_g, P_G):
    # Assuming a uniform prior for P(G), it cancels out in the ratio
    return np.exp(-C_s_to_q - C_q_to_g) / np.exp(-C_q_to_g)

# Define the legibility function for the path ξ(t)
def legibility(T, P_G_star):
    # Compute the cost from the start to the query point q and from q to the goal G
    C_s_to_q = C(lambda t: derivative(xi_x, t), lambda t: derivative(xi_y, t), T)
    C_q_to_g = C_s_to_q  # This is a simplification for illustration purposes

    # Compute the numerator integral for legibility
    numerator_integral, _ = quad(
        lambda t: probability_G_given_xi(C_s_to_q, C_q_to_g, P_G_star) * (T - t), 0, T
    )
    
    # Compute the denominator integral for legibility
    denominator_integral, _ = quad(lambda t: T - t, 0, T)
    
    # Return the legibility score
    return numerator_integral / denominator_integral

# Assuming a uniform distribution for P(G) and T is the duration of the path
P_G = 1  # Uniform probability of goal G
T = 1    # Assuming the path duration is normalized to 1 for simplicity

# Calculate the legibility score
legibility_score = legibility(T, P_G)
print("Legibility score:", legibility_score)
