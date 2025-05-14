#!/usr/bin/env python

from gazebo_utils.spawner import Spawner
from geometry_msgs.msg import Pose, Point

spawner = Spawner()

spawner.spawn('table', 'table', 
              Pose(position=Point(0,0,0)))

spawner.spawn('cone', 'cone', 
              Pose(position=Point(0,0,1.064)))

spawner.spawn('coke_can', 'coke_can', 
              Pose(position=Point(0.2,0,1.0128)))

spawner.spawn('tennis_ball', 'tennis_ball', 
              Pose(position=Point(-0.2,0,1.05096)))
