#!/usr/bin/env python

import rospy, sys

# height = int(sys.argv[1])
# width = int(sys.argv[2])
# depth = int(sys.argv[3])
# mass = int(sys.argv[4])

height = 0.3
width = 0.4 
depth = 0.5
mass = 2.0

factor = 1.0 / 12.0
Ih = factor * mass * (width**2 + depth**2)
Iw = factor * mass * (depth**2 + height**2)
Id = factor * mass * (width**2 + height**2)
print(Ih,Iw,Id)