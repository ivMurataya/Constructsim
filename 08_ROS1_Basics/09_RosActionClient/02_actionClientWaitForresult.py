#! /usr/bin/env python

import rospy
import time
import actionlib
from ardrone_as.msg import ArdroneAction, ArdroneGoal, ArdroneResult, ArdroneFeedback

nImage = 1

# definition of the feedback callback. This will be called when feedback
# is received from the action server
# it just prints a message indicating a new message has been received
def feedback_callback(feedback):
    global nImage
    print('[Feedback] image n.%d received'%nImage)
    nImage += 1
    

# initializes the action client node
rospy.init_node('example_with_waitforresult_action_client_node')


action_server_name = '/ardrone_action_server'
client = actionlib.SimpleActionClient(action_server_name, ArdroneAction)

# waits until the action server is up and running
rospy.loginfo('Waiting for action Server '+action_server_name)
client.wait_for_server()
rospy.loginfo('Action Server Found...'+action_server_name)


# creates a goal to send to the action server
goal = ArdroneGoal()
goal.nseconds = 10 # indicates, take pictures along 10 seconds

client.send_goal(goal, feedback_cb=feedback_callback)
rate = rospy.Rate(1)

rospy.loginfo("Lets Start The Wait for the Action To finish Loop...")
while not client.wait_for_result():
    rospy.loginfo("Doing Stuff while waiting for the Server to give a result....")
    rate.sleep()

rospy.loginfo("Example with WaitForResult Finished.")

"""
rosrun my_action_client_example_pkg wait_for_result_test.py
[INFO] [1739386578.478770, 0.000000]: Waiting for action Server /ardrone_action_server
[INFO] [1739386578.576576, 1178.918000]: Action Server Found.../ardrone_action_server
[INFO] [1739386578.580877, 1178.923000]: Lets Start The Wait for the Action To finish Loop...
[Feedback] image n.1 received
[Feedback] image n.2 received
[Feedback] image n.3 received
[Feedback] image n.4 received
[Feedback] image n.5 received
[Feedback] image n.6 received
[Feedback] image n.7 received
[Feedback] image n.8 received
[Feedback] image n.9 received
[INFO] [1739386589.421206, 1187.927000]: Example with WaitForResult Finished.
"""
 
