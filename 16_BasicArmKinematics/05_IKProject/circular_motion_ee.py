"""
That will be in charge of generating the elliptical 
positions and publish them in the topic /ee_pose_commands of message type planar_3dof_control.EndEffector
"""
#!/usr/bin/env python3

import rospy
from planar_3dof_control.msg import EndEffector
from rviz_marker import MarkerBasics
from math import sin, cos, pi



if __name__ == '__main__':
    rospy.init_node('marker_basic_node', anonymous=True)
    positions = rospy.Publisher('/ee_pose_commands', EndEffector, queue_size=1)
    markerbasics_object = MarkerBasics()
    rate = rospy.Rate(3)
    theta = 0.0
    index = 0
    message = EndEffector()
    message.ee_xy_theta.z = 0.0
    message.elbow_policy.data = "down"
    markerbasics_object.publish_point(0, 0, roll=theta, index=index)
    rate.sleep()


    while not rospy.is_shutdown():
        markerbasics_object.publish_point(2*cos(theta), sin(theta), roll=theta, index=index)

        message.ee_xy_theta.x = 2*cos(theta)
        message.ee_xy_theta.y = sin(theta)
        message.ee_xy_theta.z = theta
        # message.elbow_policy.data = "down"

        theta += 0.1
        if theta < pi / 2:
            message.elbow_policy.data = "down"
            rospy.loginfo("0 to pi/2  UP")
        elif theta < pi:
            message.elbow_policy.data = "down"
            rospy.loginfo("pi/2 to pi DOWN ")
        elif theta < 3 * pi / 2:
            message.elbow_policy.data = "down"
            rospy.loginfo("pi to 3pi/2 DOWN ")
        elif theta < 2 * pi:
            message.elbow_policy.data = "down"
            rospy.loginfo("3pi/2 to 2pi UP")

        

        if theta >= 2*pi:
            theta = 0

        index += 1
        positions.publish(message)
        rate.sleep()



"""
rosmsg show planar_3dof_control/EndEffector
geometry_msgs/Vector3 ee_xy_theta
  float64 x
  float64 y
  float64 z
std_msgs/String elbow_policy
  string data


python circulas_motion_ee.py
rostopic echo /ee_pose_commands
  
"""