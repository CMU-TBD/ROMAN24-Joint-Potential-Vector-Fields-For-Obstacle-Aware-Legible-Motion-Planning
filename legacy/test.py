import math

def calculate_slope_and_intercept(p1, p2):
    """Calculate the slope and y-intercept of the line passing through p1 and p2."""
    x1, y1 = p1
    x2, y2 = p2
    if x2 - x1 == 0:
        return None  # Line is vertical
    m = (y2 - y1) / (x2 - x1)
    b = y1 - m * x1
    return m, b

def generate_equidistant_points(p1, p2, num_points, distance):
    """Generates equidistant points along the line defined by p1 and p2."""
    # Calculate slope and y-intercept
    slope_intercept = calculate_slope_and_intercept(p1, p2)
    if slope_intercept is None:
        print("Cannot generate points on a vertical line using this method.")
        return []
    
    m, b = slope_intercept
    theta = math.atan(m)
    
    # Choose the first point as the starting point
    start_x, start_y = p1
    points = [(start_x, start_y)]
    
    for i in range(1, num_points):
        x = start_x + i * distance * math.cos(theta)
        y = start_y + i * distance * math.sin(theta)
        points.append((x, y))
    
    return points

# Example usage
p1 = (3, 0)
p2 = (4, 6)
num_points = 62  # Number of points to generate
distance = 0.1  # Distance between consecutive points

points = generate_equidistant_points(p1, p2, num_points, distance)
for point in points:
    print(point)

import numpy
numpy.savetxt('points.txt',points)