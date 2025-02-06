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
        self.min_wall_distance = 0.17
        self.max_wall_distance = 0.22

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
