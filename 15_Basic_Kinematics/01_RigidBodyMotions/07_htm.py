import math, rospy
from utilities import set_model_state, get_model_state
from geometry_msgs.msg import Pose, Point, Quaternion
import numpy as np
from tf.transformations import quaternion_matrix

# Retrieve the current state of the 'coke_can' model from Gazebo
model_state = get_model_state('coke_can_0')
pose = model_state.pose  # Extract the pose (position and orientation)

# Convert the orientation (quaternion) to a NumPy array in the order [x, y, z, w]
q = np.array([pose.orientation.x, pose.orientation.y, 
              pose.orientation.z, pose.orientation.w])

# Convert the quaternion into a 4x4 homogeneous rotation matrix
# This includes only rotation initially, with no translation
H = quaternion_matrix(q)

# Add the translation (position) components to the last column of the matrix
H[0][3] = pose.position.x  # Set X position
H[1][3] = pose.position.y  # Set Y position
H[2][3] = pose.position.z  # Set Z position

# Print the original pose object (for debugging/inspection)
print(pose)

# Print the full 4x4 transformation matrix (rotation + translation)
print(H)

"""
Explanation:
- Build a NumPy array representing the quaternion [x, y, z, w].
- Convert this quaternion into a 4x4 homogeneous rotation matrix using tf.transformations.quaternion_matrix.
- The output matrix represents only rotation by default:
    [[ R00 R01 R02 0 ]
     [ R10 R11 R12 0 ]
     [ R20 R21 R22 0 ]
     [  0   0   0  1 ]]
- We then insert translation values into the last column, creating a full transformation matrix.
- If the pose has no rotation (identity quaternion) and is positioned at (0.5, 0, 0.5),
  the result will be:
    [[1. 0. 0. 0.5]
     [0. 1. 0. 0.0]
     [0. 0. 1. 0.5]
     [0. 0. 0. 1.0]]
"""
