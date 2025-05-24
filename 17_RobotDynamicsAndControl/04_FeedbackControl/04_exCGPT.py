#!/usr/bin/env python

from __future__ import print_function
import math, rospy

from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from gazebo_msgs.srv import SetModelConfiguration

# Desired joint positions (in radians)
theta1_des = math.radians(0)
theta2_des = math.radians(0)

# PD gains
Kp = [25.0, 25.0]   # proportional gains for joint1 and joint2
Kd = [5.0, 5.0]     # derivative gains for joint1 and joint2

def callback(msg):
    global j1_effort
    global j2_effort

    # Get current joint positions and velocities
    theta1 = msg.position[0]
    theta2 = msg.position[1]
    dtheta1 = msg.velocity[0]
    dtheta2 = msg.velocity[1]

    # Compute errors
    e1 = theta1_des - theta1
    e2 = theta2_des - theta2
    de1 = -dtheta1
    de2 = -dtheta2

    # PD control
    tau_1 = Kp[0] * e1 + Kd[0] * de1
    tau_2 = Kp[1] * e2 + Kd[1] * de2

    # Publish torques
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
               [1.5, 0])  # start at 0 radians
    rospy.Subscriber('/rrbot/joint_states', JointState, callback)
    rospy.spin()
