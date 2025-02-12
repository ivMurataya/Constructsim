#! /usr/bin/env python

import rospy
import time
import actionlib
from ardrone_as.msg import ArdroneAction, ArdroneGoal, ArdroneResult, ArdroneFeedback

# We create some constants with the corresponing vaules from the SimpleGoalState class
PENDING = 0
ACTIVE = 1
DONE = 2
WARN = 3
ERROR = 4

nImage = 1

# definition of the feedback callback. This will be called when feedback
# is received from the action server
# it just prints a message indicating a new message has been received
def feedback_callback(feedback):
    """
    Error that might jump
    
    self._feedback.lastImage = 
AttributeError: 'ArdroneAS' obj
    
    """
    global nImage
    print('[Feedback] image n.%d received'%nImage)
    nImage += 1

# initializes the action client node
rospy.init_node('example_no_waitforresult_action_client_node')

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


# You can access the SimpleAction Variable "simple_state", that will be 1 if active, and 2 when finished.
#Its a variable, better use a function like get_state.
#state = client.simple_state
# state_result will give the FINAL STATE. Will be 1 when Active, and 2 if NO ERROR, 3 If Any Warning, and 3 if ERROR
state_result = client.get_state()

rate = rospy.Rate(1)

rospy.loginfo("state_result: "+str(state_result))
counter = 0
while state_result < DONE:
    rospy.loginfo("Doing Stuff while waiting for the Server to give a result....")
    counter += 1
    rate.sleep()
    state_result = client.get_state()
    rospy.loginfo("state_result: "+str(state_result)+", counter ="+str(counter))
    if counter == 2:
        rospy.logwarn("Canceling Goal...")
        client.cancel_goal()
        rospy.logwarn("Goal Canceled")
        state_result = client.get_state()
        rospy.loginfo("Update state_result after Cancel : "+str(state_result)+", counter ="+str(counter))


"""
rosrun my_action_client_example_pkg cancel_goal_test.py
[INFO] [1739387465.057269, 1911.081000]: Waiting for action Server /ardrone_action_server
[INFO] [1739387465.094589, 1911.108000]: Action Server Found.../ardrone_action_server
[INFO] [1739387465.106803, 1911.111000]: state_result: 0
[INFO] [1739387465.108862, 1911.112000]: Doing Stuff while waiting for the Server to give a result....
[Feedback] image n.1 received
[INFO] [1739387466.366846, 1912.111000]: state_result: 1, counter =1
[Feedback] image n.2 received
[INFO] [1739387466.370833, 1912.114000]: Doing Stuff while waiting for the Server to give a result....
[INFO] [1739387467.732164, 1913.111000]: state_result: 1, counter =2
[Feedback] image n.3 received
[WARN] [1739387467.735698, 1913.113000]: Canceling Goal...
[WARN] [1739387467.737536, 1913.114000]: Goal Canceled
[INFO] [1739387467.740565, 1913.114000]: Update state_result after Cancel : 1, counter =2
[INFO] [1739387467.741867, 1913.116000]: Doing Stuff while waiting for the Server to give a result....
[INFO] [1739387469.026016, 1914.111000]: state_result: 1, counter =3
[INFO] [1739387469.028363, 1914.111000]: Doing Stuff while waiting for the Server to give a result....
[INFO] [1739387470.308804, 1915.111000]: state_result: 2, counter =4
"""
