#!/usr/bin/env python

import copy
import actionlib
import rospy

from math import sin, cos
from moveit_python import (MoveGroupInterface,
                           PlanningSceneInterface,
                           PickPlaceInterface)
from moveit_python.geometry import rotate_pose_msg_by_euler_angles

from control_msgs.msg import FollowJointTrajectoryAction, FollowJointTrajectoryGoal
from control_msgs.msg import PointHeadAction, PointHeadGoal
from grasping_msgs.msg import FindGraspableObjectsAction, FindGraspableObjectsGoal
from geometry_msgs.msg import PoseStamped
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from moveit_msgs.msg import PlaceLocation, MoveItErrorCodes
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


# Point the head using controller

class PointHeadClient(object):

    def __init__(self):
        self.client = actionlib.SimpleActionClient(
            "head_controller/point_head", PointHeadAction)
        rospy.loginfo("Waiting for head_controller...")
        self.client.wait_for_server()

    def look_at(self, x, y, z, frame, duration=1.0):
        goal = PointHeadGoal()
        goal.target.header.stamp = rospy.Time.now()
        goal.target.header.frame_id = frame
        goal.target.point.x = x
        goal.target.point.y = y
        goal.target.point.z = z
        goal.min_duration = rospy.Duration(duration)
        self.client.send_goal(goal)
        self.client.wait_for_result()

# Tools for grasping


class GraspingClient(object):

    def __init__(self):
        self.scene = PlanningSceneInterface("base_link")
        self.pickplace = PickPlaceInterface("arm", "gripper", verbose=True)
        self.move_group = MoveGroupInterface("arm", "base_link")

        find_topic = "basic_grasping_perception/find_objects"
        rospy.loginfo("Waiting for %s..." % find_topic)
        self.find_client = actionlib.SimpleActionClient(
            find_topic, FindGraspableObjectsAction)
        self.find_client.wait_for_server()

    def updateScene(self):
        # find objects
        goal = FindGraspableObjectsGoal()
        goal.plan_grasps = True
        self.find_client.send_goal(goal)
        self.find_client.wait_for_result(rospy.Duration(5.0))
        find_result = self.find_client.get_result()

        # remove previous objects
        for name in self.scene.getKnownCollisionObjects():
            self.scene.removeCollisionObject(name, False)
        for name in self.scene.getKnownAttachedObjects():
            self.scene.removeAttachedObject(name, False)
        self.scene.waitForSync()

        # insert objects to scene
        objects = list()
        idx = -1
        for obj in find_result.objects:
            idx += 1
            obj.object.name = "object%d" % idx
            self.scene.addSolidPrimitive(obj.object.name,
                                         obj.object.primitives[0],
                                         obj.object.primitive_poses[0],
                                         use_service=False)
            if obj.object.primitive_poses[0].position.x < 0.85:
                objects.append([obj, obj.object.primitive_poses[0].position.z])

        for obj in find_result.support_surfaces:
            # extend surface to floor, and make wider since we have narrow field of view
            height = obj.primitive_poses[0].position.z
            obj.primitives[0].dimensions = [obj.primitives[0].dimensions[0],
                                            1.5,  # wider
                                            obj.primitives[0].dimensions[2] + height]
            obj.primitive_poses[0].position.z += -height/2.0

            # add to scene
            self.scene.addSolidPrimitive(obj.name,
                                         obj.primitives[0],
                                         obj.primitive_poses[0],
                                         use_service=False)

        self.scene.waitForSync()

        # store for grasping
        #self.objects = find_result.objects
        self.surfaces = find_result.support_surfaces

        # store graspable objects by Z
        objects.sort(key=lambda object: object[1])
        objects.reverse()
        self.objects = [object[0] for object in objects]
        # for object in objects:
        #    print(object[0].object.name, object[1])
        # exit(-1)

    def getGraspableObject(self):
        graspable = None
        for obj in self.objects:
            # need grasps
            if len(obj.grasps) < 1:
                continue
            # check size
            if obj.object.primitives[0].dimensions[0] < 0.03 or \
               obj.object.primitives[0].dimensions[0] > 0.25 or \
               obj.object.primitives[0].dimensions[0] < 0.03 or \
               obj.object.primitives[0].dimensions[0] > 0.25 or \
               obj.object.primitives[0].dimensions[0] < 0.03 or \
               obj.object.primitives[0].dimensions[0] > 0.25:
                continue
            # has to be on table
            if obj.object.primitive_poses[0].position.z < 0.5:
                continue
            print(obj.object.primitive_poses[0], obj.object.primitives[0])
            return obj.object, obj.grasps
        # nothing detected
        return None, None

    def getSupportSurface(self, name):
        for surface in self.support_surfaces:
            if surface.name == name:
                return surface
        return None

    def getPlaceLocation(self):
        pass

    def pick(self, block, grasps):
        success, pick_result = self.pickplace.pick_with_retry(block.name,
                                                              grasps,
                                                              support_name=block.support_surface,
                                                              scene=self.scene)
        self.pick_result = pick_result
        return success

    def place(self, block, pose_stamped):
        places = list()
        l = PlaceLocation()
        l.place_pose.pose = pose_stamped.pose
        l.place_pose.header.frame_id = pose_stamped.header.frame_id

        # copy the posture, approach and retreat from the grasp used
        l.post_place_posture = self.pick_result.grasp.pre_grasp_posture
        l.pre_place_approach = self.pick_result.grasp.pre_grasp_approach
        l.post_place_retreat = self.pick_result.grasp.post_grasp_retreat
        places.append(copy.deepcopy(l))
        # create another several places, rotate each by 360/m degrees in yaw direction
        m = 16  # number of possible place poses
        pi = 3.141592653589
        for i in range(0, m-1):
            l.place_pose.pose = rotate_pose_msg_by_euler_angles(
                l.place_pose.pose, 0, 0, 2 * pi / m)
            places.append(copy.deepcopy(l))

        success, place_result = self.pickplace.place_with_retry(block.name,
                                                                places,
                                                                scene=self.scene)
        return success

    def tuck(self):
        joints = ["shoulder_pan_joint", "shoulder_lift_joint", "upperarm_roll_joint",
                  "elbow_flex_joint", "forearm_roll_joint", "wrist_flex_joint", "wrist_roll_joint"]
        pose = [1.32, 1.40, -0.2, 1.72, 0.0, 1.66, 0.0]
        while not rospy.is_shutdown():
            result = self.move_group.moveToJointPosition(joints, pose, 0.02)
            if result.error_code.val == MoveItErrorCodes.SUCCESS:
                return

    def stow(self):
        joints = ["shoulder_pan_joint", "shoulder_lift_joint", "upperarm_roll_joint",
                  "elbow_flex_joint", "forearm_roll_joint", "wrist_flex_joint", "wrist_roll_joint"]
        pose = [1.32, 0.7, 0.0, -2.0, 0.0, -0.57, 0.0]
        while not rospy.is_shutdown():
            result = self.move_group.moveToJointPosition(joints, pose, 0.02)
            if result.error_code.val == MoveItErrorCodes.SUCCESS:
                return

    def intermediate_stow(self):
        joints = ["shoulder_pan_joint", "shoulder_lift_joint", "upperarm_roll_joint",
                  "elbow_flex_joint", "forearm_roll_joint", "wrist_flex_joint", "wrist_roll_joint"]
        pose = [0.7, -0.3, 0.0, -0.3, 0.0, -0.57, 0.0]
        while not rospy.is_shutdown():
            result = self.move_group.moveToJointPosition(joints, pose, 0.02)
            if result.error_code.val == MoveItErrorCodes.SUCCESS:
                return


if __name__ == "__main__":
    # Create a node
    rospy.init_node("demo")

    # Make sure sim time is working
    while not rospy.Time.now():
        pass

    # Setup clients
    head_action = PointHeadClient()
    grasping_client = GraspingClient()

    # Point the head at the cube we want to pick
    # head_action.look_at(3.7, 3.18, 0.0, "map")
    cube_in_grapper = False
    grasping_client.stow()
    pick_place_done = False

    while not rospy.is_shutdown() and not pick_place_done:
        head_action.look_at(1.2, 0.0, 0.0, "base_link")

        # Get block to pick
        fail_ct = 0
        while not rospy.is_shutdown() and not cube_in_grapper:
            rospy.loginfo("Picking object...")
            grasping_client.updateScene()
            cube, grasps = grasping_client.getGraspableObject()
            if cube == None:
                rospy.logwarn("Perception failed.")
                # grasping_client.intermediate_stow()
                grasping_client.stow()
                head_action.look_at(1.2, 0.0, 0.0, "base_link")
                continue

            # Pick the block
            if grasping_client.pick(cube, grasps):
                cube_in_grapper = True
                break
            rospy.logwarn("Grasping failed.")
            grasping_client.stow()
            if fail_ct > 15:
                fail_ct = 0
                break
            fail_ct += 1

        # Tuck the arm
        # grasping_client.tuck()

        # Place the block
        while not rospy.is_shutdown() and cube_in_grapper:
            rospy.loginfo("Placing object...")
            pose = PoseStamped()
            pose.pose = cube.primitive_poses[0]
            pose.pose.position.y *= -1.0
            pose.pose.position.z += 0.02
            pose.header.frame_id = cube.header.frame_id
            if grasping_client.place(cube, pose):
                cube_in_grapper = False
                break
            rospy.logwarn("Placing failed.")
            grasping_client.intermediate_stow()
            grasping_client.stow()
            if fail_ct > 15:
                fail_ct = 0
                break
            fail_ct += 1
        # Tuck the arm, lower the torso
        grasping_client.intermediate_stow()
        grasping_client.stow()
        pick_place_done = True
        rospy.loginfo("Finished")
