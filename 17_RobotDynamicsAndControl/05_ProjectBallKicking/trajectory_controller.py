#!/usr/bin/env python

from __future__ import print_function
import math, rospy, numpy as np

from std_msgs.msg import Float64

def main():
    torque = np.load('/home/user/catkin_ws/src/final_ex/scripts/torque.npy')
    n = 0
    rospy.init_node('linear_controller', anonymous=True)
    j1_effort = rospy.Publisher('/rrbot/joint1_effort_controller/command', 
                                Float64, queue_size=10)
    j2_effort = rospy.Publisher('/rrbot/joint2_effort_controller/command', 
                                Float64, queue_size=10)
    rospy.sleep(1.0)
    r = rospy.Rate(50)
    n = 0
    while n < len(torque) and not rospy.is_shutdown():
        tau_1 = torque[n][0]
        tau_2 = torque[n][1]
        n += 1
        j1_effort.publish(Float64(tau_1))
        j2_effort.publish(Float64(tau_2))
        r.sleep()
    j1_effort.publish(Float64(0.0))
    j2_effort.publish(Float64(0.0))    

if __name__ == '__main__':
    main()
