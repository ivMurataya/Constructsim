import math, rospy
from utilities import set_model_state, get_model_state
from geometry_msgs.msg import Pose, Point, Quaternion

# Fixed position of the model (origin)
position = Point(x=0, y=0, z=0)

# Loop through yaw angles from 0 to 350 in 10 steps
for angle in range(0, 360, 10):
    theta = math.radians(angle)  # Convert degrees to radians

    # Create a quaternion representing a rotation around Z axis (yaw)
    # For pure Z-axis rotation, only z and w are non-zero in quaternion
    orientation = Quaternion(
        x=0, 
        y=0, 
        z=math.sin(theta / 2), 
        w=math.cos(theta / 2)
    )

    # Update the model's orientation at the same fixed position
    set_model_state('mobile_base', Pose(position, orientation))

    # Sleep briefly to allow time for visualization or simulation update
    rospy.sleep(0.1)
