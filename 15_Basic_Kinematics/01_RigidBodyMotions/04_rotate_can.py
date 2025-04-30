import math, rospy
from utilities import set_model_state, get_model_state, \
                      pause_physics, unpause_physics
from geometry_msgs.msg import Pose, Point, Quaternion
from tf.transformations import quaternion_about_axis

# Pause physics simulation to ensure stable setup before movements
pause_physics()

# Define a fixed position for the model
position = Point(x=0.5, y=0, z=0.5)

# Loop through angles from 0  to 356 in 4 increments
for angle in range(0, 360, 4):
    theta = math.radians(angle)  # Convert degrees to radians

    # Create a quaternion representing rotation around the Z-axis
    q = quaternion_about_axis(theta, (0, 0, 1))  # Axis = Z

    # Convert quaternion array to a ROS Quaternion message
    orientation = Quaternion(*q)

    # Update the pose of the model 'coke_can_0' with the new orientation
    set_model_state('coke_can_0', Pose(position, orientation))

    # Pause briefly to allow time for each update to be visible
    rospy.sleep(0.1)
