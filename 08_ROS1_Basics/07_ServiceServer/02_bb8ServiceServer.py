#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty, EmptyResponse

def myCallback(request):
    print("Moving Bb8 in Circle")
    # Create a publisher for the /cmd_vel topic
    cmd_vel_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    # Create a Twist message to publish velocity commands
    cmd_vel_msg = Twist()
    # Set the linear and angular velocities
    cmd_vel_msg.linear.x = 0.7  # 0.5 m/s linear velocity
    cmd_vel_msg.angular.z = 0.5  # 0.2 rad/s angular velocity
    rate = rospy.Rate(10)  # 10 Hz
    # Set the duration to run the publisher (10 seconds)
    duration = rospy.Duration(10.0)
    start_time = rospy.Time.now()
    while (rospy.Time.now() - start_time) < duration and not rospy.is_shutdown():
        # Publish the Twist message
        cmd_vel_publisher.publish(cmd_vel_msg)
        rate.sleep()

    cmd_vel_msg.linear.x = 0.0  # 0.0 m/s linear velocity
    cmd_vel_msg.angular.z = 0.0  # 0.0 rad/s angular velocity
    cmd_vel_publisher.publish(cmd_vel_msg)
    print("BB8 Stop")
    return EmptyResponse()


print("Ready for a service call to move bb8")
rospy.init_node('iv_bb8_service')
my_service = rospy.Service('/move_bb8_in_circle',Empty, myCallback)
rospy.spin()
