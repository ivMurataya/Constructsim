#! /usr/bin/env python3

import sys
import rclpy
from rclpy.duration import Duration
from rclpy.action import ActionClient
from rclpy.node import Node
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
# ros2 action list -t
# ros2 action info /joint_trajectory_controller/follow_joint_trajectory -t
# ros2 interface show control_msgs/action/FollowJointTrajectory

class SteeringActionClient(Node):

    def __init__(self):
        super().__init__('move_laser_actionclient')
        self._action_client = ActionClient(self, FollowJointTrajectory, '/joint_trajectory_controller/follow_joint_trajectory')

    def send_goal(self, position_value):
        goal_msg = FollowJointTrajectory.Goal()

        # Fill in data for trajectory
        """
        Here set the following:
        Joint names. In this case, you only have one joint controlled by this controller (laser_scan_link_joint).
        You can set many different points. In this case, you only send one point to be achieved by the joint, but you could send an array of them to each joint.
        """
        joint_names = ["laser_scan_link_joint"]

        points = []

        point_to_move_to = JointTrajectoryPoint()
        point_to_move_to.time_from_start = Duration(seconds=1, nanoseconds=0).to_msg()
        point_to_move_to.positions = [position_value]

        points.append(point_to_move_to)

        goal_msg.goal_time_tolerance = Duration(seconds=1, nanoseconds=0).to_msg()
        goal_msg.trajectory.joint_names = joint_names
        goal_msg.trajectory.points = points

        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)

        self._send_goal_future.add_done_callback(self.goal_response_callback)
    
    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)
    
    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Result: '+str(result))
        rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback

    

def main(args=None):
    
    rclpy.init()

    action_client = SteeringActionClient()


    """
    The joint_trajectory_controller you loaded gives you an action named /joint_trajectory_controller/follow_joint_trajectory.
    You will use this action to send commands to the prismatic joints
    Using this, you will get the argument passed through the command line to send the joint to the position given within 
    the limits described in the URDF.
    """
    position_value = float(sys.argv[1])
    future = action_client.send_goal(position_value)

    rclpy.spin(action_client)


if __name__ == '__main__':
    main()



