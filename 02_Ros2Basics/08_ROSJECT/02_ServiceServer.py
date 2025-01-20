#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
import sys
from sensor_msgs.msg import LaserScan
from custom_interfaces.srv import FindWall
from rclpy.qos import ReliabilityPolicy, QoSProfile
import math

from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

class RotateRobot(Node):
    def __init__(self):

        super().__init__('rotate_robot')
        self.reentrant_group_1 = ReentrantCallbackGroup()

        #Variables used for Odom Data
        self.current_angle = None
        self.target_angle = 0.0
        self.moveDeg = 0.0
        self.odom_received = False

        #Variables used for laser data
        self.minRangeList = None
        self.sensorData = None  
        self.angle_min = None
        self.angle_increment = None

        self.odom_subscriber = self.create_subscription(
            Odometry,
            '/odom',  # Odometry data topic
            self.odom_callback,
            10,
            callback_group=self.reentrant_group_1
        )

        self.subscriber_laser = self.create_subscription(
            LaserScan,
            '/scan',
            self.laserscan_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        )

        self.cmd_vel_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        self.srv = self.create_service(FindWall, 'find_wall', self.empty_callback,callback_group=self.reentrant_group_1)

        self.get_logger().info("Service node is ready")


    def laserscan_callback(self, msg):
        self.sensorData = msg
        self.angle_min = msg.angle_min
        self.angle_increment = msg.angle_increment

    def odom_callback(self, msg):
        # Extract the current angle from the odometry data (quaternion to Euler conversion)
        orientation_q = msg.pose.pose.orientation
        yaw = self.quaternion_to_euler(orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w)
        self.current_angle = yaw  # Yaw angle (z-axis)
        self.odom_received = True

    def quaternion_to_euler(self, qx, qy, qz, qw):
        """
        Converts quaternion to Euler angles (roll, pitch, yaw)
        """
         # Yaw (rotation around Z-axis)
        siny_cosp = 2.0 * (qw * qz + qx * qy)
        cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return yaw


    def wait_for_odom(self):
        """
        Waits until odometry data is received.
        """
        self.get_logger().info('Waiting for /odom data...')
        while not self.odom_received:
            rclpy.spin_once(self)
        self.get_logger().info('Odometry data received!')

    def empty_callback(self, request, response):
        self.get_logger().info("Service Called")
        if not self.sensorData and not self.odom_received:
            response.wallfound = False
            self.get_logger().info("No laser or Odom data received yet.")
            return response
        min_distance = self.getLaserData(self.sensorData)
        if min_distance is not None:
            self.moveDeg = self.getAngle(min_distance)
            self.rotate_relative(self.moveDeg)
            response.wallfound = True
        else:
            response.wallfound = False
        return response

    def getLaserData(self, data):
        try:
            self.minRangeList = data.ranges.index(min(data.ranges))
            self.minValueLaser = data.ranges[self.minRangeList]
            self.get_logger().info(f"The min value is {self.minValueLaser} located on {self.minRangeList}")
            return self.minValueLaser
        except ValueError:
            self.get_logger().info("Error Procesing laser scan data.")
            return None


    def rotate_relative(self, relative_angle_degrees):
        # Wait for odometry before starting the rotation
        self.wait_for_odom()
        # Calculate the target angle relative to the current angle
        relative_angle_radians = math.radians(relative_angle_degrees)
        target_angle = self.current_angle + relative_angle_radians
        
        # Normalize the target angle to be within -pi to pi
        target_angle = (target_angle + math.pi) % (2 * math.pi) - math.pi
        self.target_angle = target_angle

        # Start rotating
        twist_msg = Twist()
        while abs(self.target_angle - self.current_angle) > 0.02:
            # Determine the error in angle
            angle_diff = self.target_angle - self.current_angle
            # Normalize the angle difference to be within -pi to pi
            angle_diff = (angle_diff + math.pi) % (2 * math.pi) - math.pi
            #self.get_logger().info(f"Angle diff {angle_diff}")

            # Set angular velocity (simple proportional control)
            twist_msg.angular.z = 0.5 * angle_diff  # Proportional gain of 0.5
            self.cmd_vel_publisher.publish(twist_msg)
            #self.get_logger().info(f'Rotating: Current angle = {self.current_angle}, Target angle = {self.target_angle}')
            rclpy.spin_once(self)  # Keep the program responsive

        # Stop once the desired angle is reached
        twist_msg.angular.z = 0.0
        self.cmd_vel_publisher.publish(twist_msg) 
        self.get_logger().info(f'Rotation completed. Reached target angle {self.target_angle}')


    def getAngle(self, min_distance):
        # Calculate the angle to the minimum range
        angle_to_min = self.angle_min + self.minRangeList * self.angle_increment
        angle_to_mind = math.degrees(angle_to_min)
        self.get_logger().info(f"Rotating to angle: {math.degrees(angle_to_min):.2f} degrees")
        return angle_to_mind


def main(args=None):
    rclpy.init(args=args)
    moving_service = RotateRobot()

    # Use MultiThreadedExecutor
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(moving_service)
    try:
        executor.spin()
    finally:
        moving_service.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
"""
colcon build --packages-select wall_follower
source install/setup.bash

ros2 run wall_follower test_node -180
ros2 run wall_follower 2test_node
"""


