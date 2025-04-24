import numpy as np
from utilities import plot_vectors_3D

# Vector v
vector = np.array(((0,0,5)))

theta_deg = 90
theta_rad = np.radians(theta_deg)
# Rotation matrix R
# Define the rotation matrix along Z axis
X = np.array([
    [1 , 0 , 0 ],
    [0, np.cos(theta_rad), -np.sin(theta_rad) ],
    [0, np.sin(theta_rad),  np.cos(theta_rad) ]
    
])

Y = np.array([
    [np.cos(theta_rad), 0, -np.sin(theta_rad) ],
                   [0 , 1 , 0 ],
    [np.sin(theta_rad), 0,  np.cos(theta_rad) ]
    
])

Z = np.array([
    [np.cos(theta_rad), -np.sin(theta_rad), 0 ],
    [np.sin(theta_rad),  np.cos(theta_rad), 0 ],
    [0 , 0 ,1]
])



# Product of matrices with numpy
wX = np.dot(X,vector)
print('Product of matrices w = R * v =',wX)


wY = np.dot(Y,wX)
print('Product of matrices w = R * v =',wY)

wZ = np.dot(Z,wY)
print('Product of matrices w = R * v =',wZ)


plot_vectors_3D(
    [vector, wX, wY, wZ]
)