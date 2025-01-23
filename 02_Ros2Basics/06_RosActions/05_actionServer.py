#!/usr/bin/env python3 

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped, Point
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32
from rclpy.action import ActionClient, ActionServer
from tf_transformations import quaternion_from_euler
from action_msgs.msg import GoalStatus
import time
#from leo_description.action import GoToPose
from actions_quiz_msg.action import Distance
import math
import asyncio



class MyActionServer(Node):
    def __init__(self):
        super().__init__('my_action_server')

        # Action server to accept goals
        self.action_server = ActionServer(
            self, Distance, 'distance_as', self.execute_callback)

        # Publisher to set initial pose
        self.initial_pose_publisher = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        
        # Publisher for distance left
        self.distance_left_publisher = self.create_publisher(Float32, '/distance_left', 10)
        
        # Publisher for distance left
        self.distance_traveled_publisher = self.create_publisher(Float32, '/distance_traveled', 10)
        
        # Action client for navigation to pose
        self.nav_to_pose_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')

        # Subscriber for odometry to track current position
        self.odom_subscriber = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)

        # Wait for the localization node to be ready
        self.wait_for_localization()

        # Set the initial pose to (0, 0, 0)
        self.set_initial_pose(0.0, 0.0, 0.0)

        self.firstOdom = None
        self.isFirt = False
        self.finalDistance = 0.0

        # Robot's current position
        self.current_position = Point()

    def odom_callback(self, msg):
        """Update the robot's current position from odometry."""
        self.current_position = msg.pose.pose.position
        if not self.isFirt:
            self.firstOdom = msg.pose.pose.position
            self.isFirt = True

    def wait_for_localization(self):
        self.get_logger().info("Waiting for localization to be active...")
        amcl_pose_topic = '/amcl_pose'
        while not self.count_subscribers('/initialpose') > 0:
            self.get_logger().info("Waiting for subscribers to /initialpose...")
            time.sleep(1.0)
        while not self.topic_exists(amcl_pose_topic):
            self.get_logger().info(f"Waiting for {amcl_pose_topic} topic...")
            time.sleep(1.0)
        self.get_logger().info("Localization is active.")

    def topic_exists(self, topic_name):
        topics = self.get_topic_names_and_types()
        return any(topic[0] == topic_name for topic in topics)

    def set_initial_pose(self, x, y, yaw):
        """Publish the initial pose to the /initialpose topic multiple times."""
        initial_pose = PoseWithCovarianceStamped()
        initial_pose.header.frame_id = 'map'
        initial_pose.header.stamp = self.get_clock().now().to_msg()

        initial_pose.pose.pose.position.x = x
        initial_pose.pose.pose.position.y = y

        # Set the orientation as quaternion (for yaw rotation)
        q = quaternion_from_euler(0, 0, yaw)
        initial_pose.pose.pose.orientation.x = q[0]
        initial_pose.pose.pose.orientation.y = q[1]
        initial_pose.pose.pose.orientation.z = q[2]
        initial_pose.pose.pose.orientation.w = q[3]

        # Publish the initial pose multiple times
        for i in range(10):
            self.initial_pose_publisher.publish(initial_pose)
            self.get_logger().info(f"Publishing initial pose ({i+1}/10)")
            time.sleep(0.1)

        self.get_logger().info(f"Initial pose set to x: {x}, y: {y}, yaw: {yaw}")

    async def execute_callback(self, goal_handle):
        """Handle the incoming goal request."""
        self.get_logger().info('Received goal request')

        x = goal_handle.request.x
        y = goal_handle.request.y
        yaw = goal_handle.request.yaw

        self.get_logger().info(f"Navigating to: x={x}, y={y}, yaw={yaw}")

        feedback_msg = Distance.Feedback()

        # Send navigation goal
        result = await self.send_navigation_goal(x, y, yaw, goal_handle, feedback_msg, self.firstOdom)

        #After attempting to navigate, it checks if the robot successfully reached the goal:
        if result:
            self.get_logger().info('Goal reached successfully!')
            goal_handle.succeed()
            return Distance.Result(success=True, distance_traveled = self.finalDistance)
        else:
            self.get_logger().info('Failed to reach goal :(')
            goal_handle.abort()
            return Distance.Result(success=False, distance_traveled = self.finalDistance)



    # is responsible for sending a goal to another action server—in this case, the NavigateToPose action server. 
    # Declares an asynchronous function to handle sending the navigation goal without blocking the main thread.
    async def send_navigation_goal(self, x, y, yaw, goal_handle, feedback_msg, firstOdom):
        """Send a navigation goal using the NavigateToPose action."""
        goal_msg = NavigateToPose.Goal()

        # Set goal position
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y

        # Set goal orientation (yaw as quaternion)
        q = quaternion_from_euler(0, 0, yaw)
        goal_msg.pose.pose.orientation.x = q[0]
        goal_msg.pose.pose.orientation.y = q[1]
        goal_msg.pose.pose.orientation.z = q[2]
        goal_msg.pose.pose.orientation.w = q[3]

        # Set the frame and timestamp
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()

        # Wait for the NavigateToPose action server
        self.get_logger().info("Waiting for the NavigateToPose action server...")
        self.nav_to_pose_client.wait_for_server()

        # Send the goal
        self.get_logger().info(f"Sending navigation goal to: x={x}, y={y}, yaw={yaw}")
        send_goal_future = self.nav_to_pose_client.send_goal_async(goal_msg)
        nav_goal_handle = await send_goal_future

        if not nav_goal_handle.accepted:
            self.get_logger().info('Navigation goal rejected')
            return False

        self.get_logger().info('Navigation goal accepted')


        # Monitor the navigation and send feedback
        def feedback_callback():
            # Calculate distance left
            distance_msg = Float32()
            distance_traveled_msg = Float32()
            distance_left = math.sqrt(
                (x - self.current_position.x) ** 2 +
                (y - self.current_position.y) ** 2
            )
            distance_msg.data = distance_left

            self.distance_traveled = math.sqrt(
                (self.current_position.x - firstOdom.x ) ** 2 +
                ( self.current_position.y - firstOdom.y ) ** 2
            )
            # Publish distance left
            distance_traveled_msg.data = self.distance_traveled
            self.distance_left_publisher.publish(distance_msg)
            self.distance_traveled_publisher.publish(distance_traveled_msg)

            # Send feedback to the action client
            feedback_msg.distance_left = distance_left
            goal_handle.publish_feedback(feedback_msg)

            #self.get_logger().info(f"Distance left: {distance_left:.2f} m")

            # Create a timer to publish feedback every 0.1 seconds
        timer = self.create_timer(0.1, feedback_callback)

        # Wait for result
        get_result_future = nav_goal_handle.get_result_async()
        nav_result = await get_result_future

            # Cancel the timer once the goal is done
        

        if nav_result.status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Navigation succeeded')
            timer.cancel()
            self.finalDistance = self.distance_traveled
            return True
        else:
            self.get_logger().info(f'Navigation failed with status: {nav_result.status}')
            return False

def main(args=None):
    rclpy.init(args=args)
    node = MyActionServer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
