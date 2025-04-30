import math, rospy
from utilities import set_model_state, get_model_state
from geometry_msgs.msg import Pose, Point, Quaternion

# Initial position of the model in the XY plane
x, y, z = 1, 0, 0.22  # z = height above ground

# Loop through angles from 0 to 359 (1 step)
for angle in range(0, 360, 10):
    theta = math.radians(angle)  # Convert angle to radians for trigonometric functions

    # Apply 2D rotation around the Z-axis (XY-plane circular motion)
    xp = x * math.cos(theta) - y * math.sin(theta)  # X coordinate after rotation
    yp = x * math.sin(theta) + y * math.cos(theta)  # Y coordinate after rotation

    # Update the model's position while keeping Z constant
    set_model_state('coke_can_0', Pose(position=Point(xp, yp, z)))

    # Pause for a short time to allow visible movement
    rospy.sleep(0.1)
