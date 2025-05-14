#!/usr/bin/env python

from gazebo_utils.spawner import Spawner
from geometry_msgs.msg import Pose, Point

spawner = Spawner()

spawner.spawn('table', 'table', 
              Pose(position=Point(0,0,0)))

spawner.spawn('cardboard_box', 'cardboard_box', 
              Pose(position=Point(0,0,1.164)))
