#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.qos import ReliabilityPolicy, QoSProfile
from custom_interfaces.srv import FindWall
from custom_interfaces.action import OdomRecord  
from rclpy.action import ActionClient

 


class WallFollowingNode(Node):
    def __init__(self):
        super().__init__('wall_following_node')

        #Create Action Client
        self._action_client = ActionClient(self, OdomRecord, 'record_odom')
        
    
        # Service client for /find_wall
        self.client = self.create_client(FindWall, '/find_wall')

        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /find_wall service...')
        self.get_logger().info('/find_wall service is ready.')

        #

        # Subscriber to LaserScan
        self.subscriber_laser = self.create_subscription(
            LaserScan,
            '/scan',
            self.laserscan_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        )

        # Publisher for movement commands
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)

        # Robot control parameters
        self.wall_follow_tolerance = 0.5  # Tolerance for wall following (m)
        self.min_wall_distance = 0.17
        self.max_wall_distance = 0.22

        self.ServiceDone = False

        self.get_logger().info("wall_following_node Ready...")

    def call_find_wall_service(self):
        # Create a request object
        request = FindWall.Request()

        # Call the service and wait for the response
        self.get_logger().info('Calling /find_wall service...')
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            response = future.result()
            self.get_logger().info(f'/find_wall response: success={response.wallfound}')
            if response.wallfound:
                self.ServiceDone = response.wallfound
                self.send_goal()
                self.get_logger().info('Wall found successfully. Starting wall-following behavior...')
                return True
            else:
                self.get_logger().error('Failed to find the wall. Exiting...')
                return False
        else:
            self.get_logger().error('Service call to /find_wall failed.')
            return False

    def laserscan_callback(self, msg):
        if self.ServiceDone == True:
            front_range = msg.ranges[0]
            right_range = min(msg.ranges[535:545])
            left_range = msg.ranges[180]
            
            # Initialize Twist message
            cmd = Twist()

            # Logic for wall following
            if front_range < self.wall_follow_tolerance:  # Wall in front
                cmd.linear.x = 0.025  # Move forward slowly
                cmd.angular.z = 0.5 # Turn sharply to the left
                print(f"Front: {front_range}, -----  Right: {right_range}  Wall in front")
            else:
                if right_range < self.min_wall_distance:  # Muy cerca de la pared
                    cmd.linear.x = 0.05  # Move forward
                    cmd.angular.z = 0.1  # Turn Izq
                    print(f"Front: {front_range}, -----  Right: {right_range} Alejandose de la pared")
                elif right_range > self.max_wall_distance :  # Lejos de la pared
                    cmd.linear.x = 0.05  # Move forward
                    cmd.angular.z = -0.1  # Turn derecha
                    print(f"Front: {front_range}, -----  Right: {right_range} Acercandoce a la pared")
                else:  # Maintain distance
                    cmd.linear.x = 0.075  # Move forward 
                    print(f"Front: {front_range}, -----  Right: {right_range} Move forward ")


            # Publish velocity command
            self.publisher_.publish(cmd)
    

#-------------------------------------------
    def send_goal(self):
        goal_msg = OdomRecord.Goal()

        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback)

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
        self.get_logger().info(f"Result  {result.list_of_odoms}")
        self.ServiceDone = False
        cmd = Twist()
        self.publisher_.publish(cmd)
        rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f"{feedback}")



#--------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    node = WallFollowingNode()

    # Call the /find_wall service before starting the control loop
    if node.call_find_wall_service():
        rclpy.spin(node)  # Only start wall-following behavior if the service succeeds
    else:
        node.get_logger().error('Could not start wall-following behavior.')

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
