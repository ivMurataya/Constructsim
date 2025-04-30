import math, rospy
from utilities import set_model_state, get_model_state, \
                      pause_physics, unpause_physics
from geometry_msgs.msg import Pose, Point, Quaternion
from tf.transformations import quaternion_about_axis
from tf.transformations import quaternion_multiply
from tf.transformations import quaternion_from_euler
from tf.transformations import euler_from_quaternion

# Set the initial position of the object
position = Point(x=0.5, y=0, z=0.5)

# Define desired orientation in Euler angles (degrees)
roll, pitch, yaw = 90, 45, 90

# Convert Euler angles to a quaternion (intrinsic XYZ order)
q_rpy = quaternion_from_euler(math.radians(roll), math.radians(pitch), math.radians(yaw))
orientation = Quaternion(*q_rpy)

# Set the model to the pose with this orientation
set_model_state('coke_can_0', Pose(position, orientation))
print(orientation)  # Print the quaternion representing the combined rotation
rospy.sleep(1.0)

# --- Animate rotation about X axis ---

# Start with a neutral quaternion (no rotation around X)
q_x = quaternion_about_axis(0, (1,0,0))

# Determine range and direction based on positive or negative angle
if roll >= 0:
    roll_range = range(0, roll, 1)
    offset = 1
else:
    roll_range = range(0, roll, -1)
    offset = -1

# Step through rotation angles and apply each as a quaternion
for roll_angle in roll_range:
    q_x = quaternion_about_axis(math.radians(roll_angle + offset), (1,0,0))
    orientation = Quaternion(*q_x)
    set_model_state('coke_can_0', Pose(position, orientation))
    rospy.sleep(0.01)

# --- Animate rotation about Y axis, after X ---

# Initialize Y rotation quaternion (no rotation)
q_y = quaternion_about_axis(0, (0,1,0))

# Combine current X and Y quaternions (initially both 0)
q_xy = quaternion_multiply(q_y, q_x)

# Same logic as before for pitch range
if pitch >= 0:
    pitch_range = range(0, pitch, 1)
    offset = 1
else:
    pitch_range = range(0, pitch, -1)
    offset = -1

# Step through pitch angles, combine with X rotation
for pitch_angle in pitch_range:
    q_y = quaternion_about_axis(math.radians(pitch_angle + offset), (0,1,0))
    q_xy = quaternion_multiply(q_y, q_x)  # Combine Y after X
    orientation = Quaternion(*q_xy)
    set_model_state('coke_can_0', Pose(position, orientation))
    rospy.sleep(0.01)

# --- Animate rotation about Z axis, after Y and X ---

# Initialize Z rotation quaternion (no rotation)
q_z = quaternion_about_axis(0, (0,0,1))

# Combine with existing XY rotation
q_xyz = quaternion_multiply(q_z, q_xy)

# Same logic as before for yaw range
if yaw >= 0:
    yaw_range = range(0, yaw, 1)
    offset = 1
else:
    yaw_range = range(0, yaw, -1)
    offset = -1

# Step through yaw angles, apply on top of XY rotation
for yaw_angle in yaw_range:
    q_z = quaternion_about_axis(math.radians(yaw_angle + offset), (0,0,1))
    q_xyz = quaternion_multiply(q_z, q_xy)  # Combine Z after Y after X
    orientation = Quaternion(*q_xyz)
    set_model_state('coke_can_0', Pose(position, orientation))
    rospy.sleep(0.01)

# Print final orientation after combined roll, pitch, yaw animation
print(orientation)
