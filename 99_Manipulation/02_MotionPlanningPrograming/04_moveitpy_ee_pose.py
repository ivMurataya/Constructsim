#! /usr/bin/env python

import rospy
from moveit_python import MoveGroupInterface
from moveit_msgs.msg import MoveItErrorCodes
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion

rospy.init_node('moveit_python_tutorial', anonymous=True)

move_group = MoveGroupInterface("arm", "base_link")

gripper_frame = 'gripper_link'
# As you can see, the code is very similar as the one you used in the previous Exercise. 
#The only difference is that now we are setting a PoseStamped goal for the End Effector.  

gripper_poses = [Pose(Point(0.809, -0.392, 1.358), Quaternion(0.01, 0.0, 0.0, 1.0)),
                 Pose(Point(0.840, 0.532, 1.036), Quaternion(0.01, 0.0, 0.0, 1.0))]

gripper_pose_stamped = PoseStamped()
gripper_pose_stamped.header.frame_id = 'base_link'

while not rospy.is_shutdown():
#In fact, we are setting 2 different goals. This will make our program constanly move from one goal to another, 
# until the program is terminated, because of the following for loop we placed.
    for pose in gripper_poses:
        gripper_pose_stamped.header.stamp = rospy.Time.now()

        gripper_pose_stamped.pose = pose

        result = move_group.moveToPose(gripper_pose_stamped, gripper_frame)

        if result:

            if result.error_code.val == MoveItErrorCodes.SUCCESS:
                rospy.loginfo("Trajectory successfully executed!")
            else:
                rospy.logerr("Arm goal in state: %s",
                             move_group.get_move_action().get_state())
        else:
            rospy.logerr("MoveIt failure! No result returned.")

move_group.get_move_action().cancel_all_goals()
