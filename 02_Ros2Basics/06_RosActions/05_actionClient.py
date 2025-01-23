#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav_msgs.msg import Odometry
import math

#from custom_interfaces.action import GoToPose
from actions_quiz_msg.action import Distance

class CustomActionServer(Node):

    def __init__(self):
        super().__init__('custom_action_client')
        self._action_client = ActionClient(self, Distance, 'distance_as')

        # Initialize positions
        self.robot_x = None
        self.robot_y = None

        self.goal_x = None
        self.goal_y = None
    
        # Store goal handle to keep track of active goal
        self.goal_handle = None


    def send_goal(self, x, y, yaw):
        self.goal_x = x
        self.goal_y = y
        goal_msg = Distance.Goal()
        goal_msg.x = x
        goal_msg.y = y
        goal_msg.yaw = yaw

        self.get_logger().info(f'Sending goal: x={x}, y={y}, yaw={yaw}')

        # Wait until the action server is available
        self._action_client.wait_for_server()

        # Send the goal asynchronously
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback)

        # Attach callback for when the goal response is received
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        # Handle the goal response
        self.goal_handle = future.result()

        if not self.goal_handle.accepted:
            self.get_logger().info('Goal was rejected by the action server.')
            return

        self.get_logger().info('Goal accepted by the action server.')

        # Attach callback for when the result is received
        self._get_result_future = self.goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        # Handle the result
        result = future.result().result
        self.get_logger().info(f'Action completed with success: {result.success}, total distance Traveled {result.distance_traveled}')
        # Optionally, shutdown the node or send another goal

    def feedback_callback(self, feedback_msg):
        # Handle feedback (if any)
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Received feedback from action server. {feedback}')



def main(args=None):
    rclpy.init(args=args)
    action_client = CustomActionServer()

    # Send the initial goal
    initial_x = 8.3
    initial_y = -2.2
    initial_yaw = -0.2

    action_client.send_goal(initial_x, initial_y, initial_yaw)
    # Keep the node alive to receive callbacks
    rclpy.spin(action_client)
    action_client.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


"""
cd ~/ros2_ws
rm -rf build install log
source /home/simulations/ros2_sims_ws/install/setup.bash
colcon build
source ~/ros2_ws/install/setup.bash
source /home/simulations/ros2_sims_ws/install/setup.bash
ros2 launch leo_description loc_nav.launch.py


"""
