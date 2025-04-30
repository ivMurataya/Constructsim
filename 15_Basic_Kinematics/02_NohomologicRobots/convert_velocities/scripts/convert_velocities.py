#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from std_msgs.msg import Float64
L = 0.3
R = 0.1

leftW = rospy.Publisher('/left_wheel_controller/command', Float64, queue_size=1)
rightW = rospy.Publisher('/right_wheel_controller/command', Float64, queue_size=1)
# Callback function for receiving messages

def callback(data):
    rospy.loginfo("Linear: %f ,  Angular %f", data.linear.x , data.angular.z)
    v = data.linear.x
    w = data.angular.z
    Vr = ((2*v) + (w*L))/(2*R)
    Vl = (2*v - w*L)/(2*R)
    rospy.loginfo("VR: %f , VL %f", Vr , Vl)
    leftW.publish(Vl)
    rightW.publish(Vr)


def listener():
    # Initialize the node
    rospy.init_node('listener', anonymous=True)

    # Subscribe to the /chatter topic
    rospy.Subscriber("/twist_vels", Twist, callback)
 

    # Keep the node running until it's shut down
    rospy.spin()

if __name__ == '__main__':
    listener()
