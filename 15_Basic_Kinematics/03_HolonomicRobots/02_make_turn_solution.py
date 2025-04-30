#! /usr/bin/env python

def twist2wheels(wz, vx, vy):
    # half of the wheel base distance (length of robot along x-axis)
    l = 0.500/2
    # the radius of the wheels (from 254 mm diameter)
    r = 0.254/2
    # half of the track width (width of robot along y-axis)
    w = 0.548/2
    
    # Kinematic transformation matrix (4x3) for holonomic drive
    # Each row corresponds to one wheel's contribution from wz, vx, and vy
    H = np.array([[-l-w, 1, -1],
                  [ l+w, 1,  1],
                  [ l+w, 1, -1],
                  [-l-w, 1,  1]]) / r

    # Planar twist vector: [angular z, linear x, linear y]
    twist = np.array([wz, vx, vy])
    twist.shape = (3,1)  # Ensure it's a column vector

    # Compute wheel speeds: u = (1/r) * H * twist
    u = np.dot(H, twist)

    # Print the resulting wheel speeds (4x1 vector)
    print(u)

    # Return as a flat list [u1, u2, u3, u4]
    return u.flatten().tolist()

import rospy, numpy as np
from std_msgs.msg import Float32MultiArray

# Initialize the ROS node
rospy.init_node('make_turn', anonymous=True)

# Publisher to the 'wheel_speed' topic
pub = rospy.Publisher('wheel_speed', Float32MultiArray, queue_size=10)

# Wait 1 second for connections to establish
rospy.sleep(1)

# Generate wheel speeds from twist input: wz=1.5 rad/s, vx=1 m/s, vy=0 m/s
u = twist2wheels(wz=-1.5, vx=2, vy=3)

# Create and publish the message with wheel speeds
msg = Float32MultiArray(data=u)
pub.publish(msg)

# Wait 1 second while the robot moves
rospy.sleep(1)

# Publish a stop command to halt the robot
stop = [0, 0, 0, 0]
msg = Float32MultiArray(data=stop)
pub.publish(msg)
