#!/usr/bin/env python

from __future__ import print_function
import math, rospy

from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from gazebo_msgs.srv import SetModelConfiguration

def callback(msg):
    global j1_effort
    global j2_effort
    tau_1 = 0
    tau_2 = 0
    j1_effort.publish(Float64(tau_1))
    j2_effort.publish(Float64(tau_2))

if __name__ == '__main__':
    rospy.init_node('linear_controller', anonymous=True)
    j1_effort = rospy.Publisher('/rrbot/joint1_effort_controller/command', 
                                Float64, queue_size=10)
    j2_effort = rospy.Publisher('/rrbot/joint2_effort_controller/command', 
                                Float64, queue_size=10)
    rospy.wait_for_service('/gazebo/set_model_configuration')
    rospy.sleep(1.0)
    j1_effort.publish(Float64(0.0))
    j2_effort.publish(Float64(0.0))
    set_joints = rospy.ServiceProxy('/gazebo/set_model_configuration', 
                                    SetModelConfiguration)
    set_joints('rrbot', 'robot_description', 
               ['joint1', 'joint2'], 
               [math.radians(0), math.radians(0)])
    rospy.Subscriber('/rrbot/joint_states', JointState, callback)
    rospy.spin()
