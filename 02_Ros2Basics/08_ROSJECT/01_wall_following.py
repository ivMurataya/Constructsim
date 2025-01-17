#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.qos import ReliabilityPolicy, QoSProfile




class WallFollowingNode(Node):
    def __init__(self):
        super().__init__('wall_following_node')

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
        self.min_wall_distance = 0.2
        self.max_wall_distance = 0.3

        self.get_logger().info("wall_following_node Ready...")

    def laserscan_callback(self, msg):
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
    
def main(args=None):
    rclpy.init(args=args)
    node = WallFollowingNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()


"""
To achieve this behavior in the robot, you need to do:

    Create a ROS2 package named wall_follower
        This package is the one that will contain the full project
        Inside the package include a ROS Python file named wall_following.py

    Subscribe to the laser topic of the robot
        Inside the wall_following.py, you need to subscribe to the laser topic and capture the rays

        In the callback of the subscriber, select the ray on the right (the one that makes a 90º angle to the right with the front of the robot) and use it to know the robot's distance to the wall

        NOTE: The topics for different simulations most likely will not be named the same. So make sure that you are using the correct name. For example, you used /kobuki/laser/scan during the course. The laser topic in this simulation is /scan

    Publish to the velocity topic of the robot
        Also inside the wall_following.py, create a publisher to the /cmd_vel topic that controls the wheels

        At every step of the control loop, you need to publish the proper velocity command on that topic, based on the value of the distances detected by the laser:
            If the distance to the wall is bigger than 0.3m, you need to make the robot approach the wall a little, by adding some rotational speed to the robot
            If the distance to the wall is smaller than 0.2m, you need to move the robot away from the wall, by adding rotational speed in the opposite direction
            If the distance to the wall is between 0.2m and 0.3m, just keep the robot moving forward

        IMPORTANT

        When the robot is moving along a wall, it can reach the next wall just infront of it. At that point in time, you should take into account how to progressively transition the robot from following the current wall to the next one.

        To detect the wall in the front, we recommend that you use the laser ray just in the front of the robot. If the distance measured by that ray is shorter than 0.5m, then make the robot turn fast to the left (moving forward at the same time).

        """
