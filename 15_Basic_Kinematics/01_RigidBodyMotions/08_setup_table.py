import math, rospy
from utilities import spawn_coke_can, spawn_table, \
                        pause_physics, unpause_physics
from geometry_msgs.msg import Pose, Point, Quaternion

# Resume the simulation physics (in case it was paused)
unpause_physics()

# Spawn a table model at position (1, 0, 0) with default orientation
spawn_table('table', Pose(position=Point(1, 0, 0)))

# Loop to spawn 12 coke cans
for i in range(12):
    # Spawn a coke can with a unique name at position (0, 0, 1)
    # This places it in mid-air above the ground or table
    spawn_coke_can('coke_can_' + str(i), Pose(position=Point(0, 0, 1)))

    # Pause briefly to allow the object to be inserted and physics to update
    rospy.sleep(1.0)
