#! /usr/bin/env python

import rospy
from moveit_python import MoveGroupInterface
from moveit_msgs.msg import MoveItErrorCodes

rospy.init_node('moveit_python_tutorial', anonymous=True)

#First of all, we create an instance of the MoveGroupInterface interface.
move_group = MoveGroupInterface("arm", "base_link")

# Next, we set the name of the joints and their goal position values.
joints = ["shoulder_pan_joint", "shoulder_lift_joint", "upperarm_roll_joint",
          "elbow_flex_joint", "forearm_roll_joint", "wrist_flex_joint", "wrist_roll_joint"]

pose = [0.0, 0.0, 0.0, -1.57, 0.0, 1.57, 0.0]
pose1 = [-0.742669, -0.08320, 2.261977, -0.7403532, -1.06993, -0.715316, -0.977430]
pose2 = [0.14980549, 0.450292, 0.4589491, -0.049886, 0.726429, 0.749970, 0.55580]
pose3 = [1.32,1.4, -0.2, 1.7, 0.0, 1.57, 0.0]


while not rospy.is_shutdown():
    #we call the moveToJointPosition function, passing the names of the joints and their position values, as well as a tolerance of 0.02.
    result = move_group.moveToJointPosition(joints, pose3, 0.02)
    
    if result:

    #As you can see, we are saving the output of the function to the varialbe result. 
    # This is because next, as you can see, we evaluate this result to check if the trajectory has been completed successfully or not.
        if result.error_code.val == MoveItErrorCodes.SUCCESS:
            rospy.loginfo("Trajectory successfully executed!")
            break
        else:
            rospy.logerr("Arm goal in state: %s",
                         move_group.get_move_action().get_state())
    else:
        rospy.logerr("MoveIt failure! No result returned.")
#Finally, when we terminate the program, we cancel all the goals so the arm stops.
move_group.get_move_action().cancel_all_goals()
