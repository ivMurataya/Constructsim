import math, rospy, time
from utilities import *
from geometry_msgs.msg import Pose, Point, Quaternion

# Optionally delete the model first (commented out here)
# delete_model("coke_can")

# Check if 'coke_can' exists in the simulation
if get_model_state('coke_can').success == False:
    # If not found, spawn a new 'coke_can' at position (0, 1, 1.0)
    spawn_coke_can('coke_can', Pose(position=Point(0,1,1.0)))

# Get the current state of the 'coke_can' model
model_state = get_model_state('coke_can')

# Print the position of the model to the console
print(model_state.pose.position)

# The following code is commented out:
# It would move the can to a new position and then print that new position

# time.sleep(1)  # Wait for 1 second before moving the model

# Move the model to a new position (1, 0, 0.22)
# new_pose = Pose(position=Point(1,0,0.22))
# set_model_state('coke_can', new_pose)

# Get the updated state of the model
# model_state = get_model_state('coke_can')

# Print the new position
# print(model_state.pose.position)
