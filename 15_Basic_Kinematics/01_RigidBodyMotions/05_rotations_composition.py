import math, rospy
from utilities import set_model_state, get_model_state, \
                      pause_physics, unpause_physics
from geometry_msgs.msg import Pose, Point, Quaternion
from tf.transformations import quaternion_about_axis
from tf.transformations import quaternion_multiply

# Set a fixed position for the model in the simulation
position = Point(x=0.5, y=0, z=0.5)

print("Rotation on X")
# Rotate around the X-axis from 0 to 87 degrees in 3-degree increments
for angle in range(0,90,3):
    # Create a quaternion for rotation around X-axis
    q_x = quaternion_about_axis(math.radians(angle), (1,0,0))
    # Convert quaternion array into a ROS Quaternion message
    orientation = Quaternion(*q_x)
    # Apply the new orientation to the model at the given position
    set_model_state('coke_can_0', Pose(position, orientation))
    # Wait briefly to visualize each step
    rospy.sleep(0.1)
    
print("Rotation on Y")
# Rotate around the Y-axis from 0 to 87 degrees in 3-degree increments
for angle in range(0,90,3):
    # Create a quaternion for rotation around Y-axis
    q_y = quaternion_about_axis(math.radians(angle), (0,1,0))
    # Combine Y rotation with the last computed X rotation
    q_xy = quaternion_multiply(q_y, q_x)  # Apply X first, then Y
    orientation = Quaternion(*q_xy)
    # Update the model's orientation
    set_model_state('coke_can_0', Pose(position, orientation))
    rospy.sleep(0.1)

print("Rotation on Z")
# Rotate around the Z-axis from 0 to 87 degrees in 3-degree increments
for angle in range(0,90,3):
    # Create a quaternion for rotation around Z-axis
    q_z = quaternion_about_axis(math.radians(angle), (0,0,1))
    # Combine Z rotation with the previous XY rotation
    q_xyz = quaternion_multiply(q_z, q_xy)  # Apply XY first, then Z
    orientation = Quaternion(*q_xyz)
    # Set the model's final pose with cumulative rotations
    set_model_state('coke_can_0', Pose(position, orientation))
    rospy.sleep(0.1)
