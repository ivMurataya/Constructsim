#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient, ActionServer
from tf_transformations import quaternion_from_euler
from action_msgs.msg import GoalStatus
import time
from leo_description.action import GoToPose

"""


In the constructor of the class, initialize a ROS2 node named my_action_server. Also, and very important, create an ActionServer object to which you pass four arguments:

    The ROS2 node that contains the Action Client: in this case, self.
    The type of Action: GoToPose.
    The Action name: go_to_pose
    A callback method to be executed when the Action receives a goal: self.execute_callback.

    Then we create a publisher to the /initialpose topic, which is crucial for accurate navigation.

    We set up an ActionClient for the NavigateToPose action provided by the Nav2 package.

    Before sending any navigation goals, we ensure that the localization system is active by calling: self.wait_for_localization()

    Finally, we set the robot's starting position and orientation by calling: self.set_initial_pose(0.0, 0.0, 0.0)


"""

class MyActionServer(Node):
    def __init__(self):
        super().__init__('my_action_server')

        # Action server to accept goals
        self.action_server = ActionServer(
            self, GoToPose, 'go_to_pose', self.execute_callback)

        # Publisher to set initial pose
        self.initial_pose_publisher = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', 10)

        # Action client for navigation to pose
        self.nav_to_pose_client = ActionClient(
            self, NavigateToPose, '/navigate_to_pose')

        # Wait for the localization node to be ready
        self.wait_for_localization()

        # Set the initial pose to (0, 0, 0)
        self.set_initial_pose(0.0, 0.0, 0.0)




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


    """
    This method will be called when the Action Server receives a goal, 
    and it contains the main functionality of the Action (what the Action will do).
    The method logs that it has received a goal request and extracts the 
    target position and orientation from the goal:

    """
    async def execute_callback(self, goal_handle):
        """Handle the incoming goal request."""
        self.get_logger().info('Received goal request')

        x = goal_handle.request.x
        y = goal_handle.request.y
        yaw = goal_handle.request.yaw

        """ It logs the coordinates and orientation then method calls self.send_navigation_goal(x, y, yaw) to send the goal to the navigation system:"""

        self.get_logger().info(f"Navigating to: x={x}, y={y}, yaw={yaw}")

        # Send navigation goal
        result = await self.send_navigation_goal(x, y, yaw)

        #After attempting to navigate, it checks if the robot successfully reached the goal:
        if result:
            self.get_logger().info('Goal reached successfully!')
            goal_handle.succeed()
            return GoToPose.Result(success=True)
        else:
            self.get_logger().info('Failed to reach goal :(')
            goal_handle.abort()
            return GoToPose.Result(success=False)



    # is responsible for sending a goal to another action server—in this case, the NavigateToPose action server. 
    # Declares an asynchronous function to handle sending the navigation goal without blocking the main thread.
    async def send_navigation_goal(self, x, y, yaw):
        """Send a navigation goal using the NavigateToPose action."""
        goal_msg = NavigateToPose.Goal()

        """
        Sets the coordinate frame and timestamps the goal, which are important for the 
        action server to process the goal correctly. 
        Then logs a message and waits until the NavigateToPose action server is 
        ready to accept goals. This is crucial to ensure that the goal is not sent 
        before the server is available.
        """
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
        """
        Logs the action of sending the goal and sends it asynchronously using 
        send_goal_async. It then awaits the future to obtain a goal handle, 
        allowing the client to track the goal's status without blocking execution.
        """
        self.get_logger().info(f"Sending navigation goal to: x={x}, y={y}, yaw={yaw}")
        send_goal_future = self.nav_to_pose_client.send_goal_async(goal_msg)
        nav_goal_handle = await send_goal_future

        if not nav_goal_handle.accepted:
            self.get_logger().info('Navigation goal rejected')
            return False

        self.get_logger().info('Navigation goal accepted')

        # Wait for result
        """
        Finally, the last part waits for the robot to complete moving to the specified 
        goal and then checks if it was successful, logging the outcome and returning 
        True or False accordingly."""
        get_result_future = nav_goal_handle.get_result_async()
        nav_result = await get_result_future

        if nav_result.status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Navigation succeeded')
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


"""
This action server enables a robot to navigate to a specified position and 
orientation by accepting goals from clients and using the nav2 package for 
autonomous navigation. It initializes the robot's starting pose, waits for 
localization to be active, and then sends navigation goals to the nav2 stack 
to move the robot to the desired location. 


colcon build --packages-select my_action_server
ros2 launch leo_description loc_nav.launch.py
ros2 run my_action_server action_server.py
ros2 action send_goal /go_to_pose leo_description/action/GoToPose "{x: -8.0, y: 6.0, yaw: 1.57}"



"""
