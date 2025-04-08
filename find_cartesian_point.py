import numpy as np
from scipy.optimize import least_squares

# Define reference points
ref1 = np.array([2, 0, 0])   # bottom-left
ref2 = np.array([0, 2, 0])   # top-left
ref3 = np.array([2, 2, 0])   # top-right
ref4 = np.array([0, 0, 0])   # bottom-right

# Distance variables (measured distances to each known point)
d1 = 1.2  # Distance to ref1
d2 = 1.1  # Distance to ref2
d3 = 1.0  # Distance to ref3
d4 = 1.3  # Distance to ref4

# Define the system of residuals
def my_system(vars):
    x, y, z = vars
    F = np.zeros(4)
    F[0] = np.sqrt((x - ref1[0])**2 + (y - ref1[1])**2 + (z - ref1[2])**2) - d1
    F[1] = np.sqrt((x - ref2[0])**2 + (y - ref2[1])**2 + (z - ref2[2])**2) - d2
    F[2] = np.sqrt((x - ref3[0])**2 + (y - ref3[1])**2 + (z - ref3[2])**2) - d3
    F[3] = np.sqrt((x - ref4[0])**2 + (y - ref4[1])**2 + (z - ref4[2])**2) - d4
    return F

# Solve for the unknown point
initial_guess = [0.5, 0.5, 0.5]  # Starting guess for (x, y, z)
result = least_squares(my_system, initial_guess)

unknown_point = np.round(result.x, decimals=4)

print("Unknown point:", unknown_point)
