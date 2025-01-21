#!/usr/bin/env python3
from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from custom_interfaces.srv import FindWall
from rclpy.qos import ReliabilityPolicy, QoSProfile
import math

from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

class Service(Node):

    def __init__(self):
        super().__init__('wall_finder_node')
        self.reentrant_group_1 = ReentrantCallbackGroup()

        self.subscriber_laser = self.create_subscription(
            LaserScan,
            '/scan',
            self.laserscan_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE),
            callback_group=self.reentrant_group_1
        )

        self.srv = self.create_service(FindWall, 'find_wall', self.empty_callback,callback_group=self.reentrant_group_1)

        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)

        
        self.minRangeList = None
        self.sensorData = None
        self.angle_min = None
        self.angle_increment = None
        self.get_logger().info("Service node is ready")

    def laserscan_callback(self, msg):
        self.sensorData = msg
        self.angle_min = msg.angle_min
        self.angle_increment = msg.angle_increment

    def empty_callback(self, request, response):
        self.get_logger().info("Service Called")
        if not self.sensorData:
            response.wallfound = False
            self.get_logger().info("No laser scan data received yet.")
            return response
        min_distance = self.getLaserData(self.sensorData)
        if min_distance is not None:
            response.wallfound = True
            self.rotate_and_move_to_object(min_distance)
        else:
            response.wallfound = False
        return response

    def getLaserData(self, data):
        try:
            self.minRangeList = data.ranges.index(min(data.ranges))
            self.minValueLaser = data.ranges[self.minRangeList]
            self.get_logger().info(f"the min value is {self.minValueLaser} located on {self.minRangeList}")
            return self.minValueLaser
        except ValueError:
            self.get_logger().info("Error Procesing laser scan data.")
            return None
 

    def rotate_and_move_to_object(self, min_distance):
        # Calculate the angle to the minimum range
        angle_to_min = self.angle_min + (self.minRangeList * self.angle_increment)
        #target_angle = (angle_to_min + math.pi) % (2 * math.pi) - math.pi
        #self.get_logger().info(f"Moving forward to the object at {min_distance} meters")
        self.get_logger().info(f"Rotating to angle: {math.degrees(angle_to_min):.2f} degrees")
        twist = Twist()
        
        """
        # Rotate the robot to face the object
        twist = Twist()
        while abs(self.sensorData.ranges[0]) > (min_distance + 0.0211):  # Rotate until the angle is approximately zero
            twist.angular.z = 0.3 * math.copysign(1, angle_to_min)

            self.publisher_.publish(twist)
            rclpy.spin_once(self, timeout_sec=0.1)
            self.get_logger().info(f"Sensor front:{self.sensorData.ranges[0]}")
            angle_to_min = self.angle_min + self.minRangeList * self.angle_increment
            #self.get_logger().info(f"Angle to min: {str(angle_to_min)}")
        """
        # Stop rotating
        twist.angular.z = 0.0
        self.publisher_.publish(twist)
        self.get_logger().info("Pointing at wall.")




def main(args=None):
    rclpy.init(args=args)
    moving_service = Service()
    
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
