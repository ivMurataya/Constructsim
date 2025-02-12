#! /usr/bin/env python

import rospy
from std_srvs.srv import Empty, EmptyRequest


# Initialise a ROS node with the name service_client
rospy.init_node('iv_service_client_bb8')
# Wait for the service client /execute_trajectory to be running
rospy.wait_for_service('/move_bb8_in_circle')
# Create the connection to the service
name_service = rospy.ServiceProxy('/move_bb8_in_circle', Empty)
# Create an object of type TrajByNameRequest
name_object = EmptyRequest()
# Send through the connection the name of the trajectory to be executed by the robot
result = name_service(name_object)
# Print the result given by the service called
print("Node move bb8 service called")
print(result)

