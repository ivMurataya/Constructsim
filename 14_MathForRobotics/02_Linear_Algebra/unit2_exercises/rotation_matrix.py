import numpy as np 
import matplotlib.pyplot as plt 
from utilities import plot_vectors

# Vector v
v = np.array (((1,0)))

# Rotation mtrix R
# Angle in degrees and radians
theta_deg = 66
theta_rad = np.radians(theta_deg)

# Define the rotation matrix
R = np.array([
    [np.cos(theta_rad), -np.sin(theta_rad)],
    [np.sin(theta_rad),  np.cos(theta_rad)]
])


# Product of matrices with numpy

w = np.dot(R,v)
print('Produt of matrices w = R*v' ,w)
plot_vectors((v,w), 'rotation')