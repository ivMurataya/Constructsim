#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
import time



class RobotStatusService(Node):
    def __init__(self):
        super().__init__('robot_move_service')

        self.mutuallyexclusive_group_1 = MutuallyExclusiveCallbackGroup()
        self.mutuallyexclusive_group_2 = MutuallyExclusiveCallbackGroup()

        name_service = '/start_turn'
        self.srv = self.create_service(SetBool, name_service, self.SetBool_callback,callback_group=self.mutuallyexclusive_group_1)
        self.timer = self.create_timer(0.5, self.timer_callback,callback_group=self.mutuallyexclusive_group_2)
  

        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.cmd = Twist()
        
        self.get_logger().info(name_service+" Service Server Ready...")

    def SetBool_callback(self, request, response):
        if request.data:
            self.get_logger().info('Request is TRUE: Setting to TURN')
            self.cmd.linear.x = 0.0
            self.cmd.angular.z = 0.3  # Rotate at 0.3 rad/s
        else:
            self.get_logger().info('Request is FALSE: Setting to GO FORWARD')
            self.cmd.linear.x = 0.2  # Move forward at 0.2 m/s
            self.cmd.angular.z = 0.0
        
        # Sleep to observe multithreading behavior
        self.publisher_.publish(self.cmd) 
        for i in range(15):
            self.get_logger().info(f"Sleeping... {i+1} seconds")
            time.sleep(1.0)
        
        self.cmd.linear.x = 0.0
        self.cmd.angular.z = 0.0            

        # Set response
        self.publisher_.publish(self.cmd) 
        response.success = True
        response.message = "Command processed"
        return response


    def timer_callback(self):
        self.get_logger().info(f"Publishing cmd: linear.x={self.cmd.linear.x}, angular.z={self.cmd.angular.z}")



class CalculationClass(Node):
    def __init__(self, seconds_sleeping=10):
        super().__init__('calculation_node')
        self.laser_forward = 0.0
        self.timeSeconds = seconds_sleeping
        self.cmd = Twist()

        # Callback Groups
        self.service_group = MutuallyExclusiveCallbackGroup()
        self.subscriber_group = MutuallyExclusiveCallbackGroup()

        # Service Server
        self.srv = self.create_service(
            SetBool,
            '/calculations',
            self.Calculations_callback,
            callback_group=self.service_group
        )

        # Subscribe to LaserScan data
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.laser_callback,
            10,
            callback_group=self.subscriber_group
        )

        # Publisher for cmd_vel
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)

    def laser_callback(self, msg):
        self.laser_forward = msg.ranges[359]

    def Calculations_callback(self, request, response):
        self.get_logger().info('Calculations_callback triggered')
        first_read = self.laser_forward
        self.get_logger().info(f"First laser read: {first_read}")

        # Sleep to wait for 5 seconds
        for i in range(10):
            self.get_logger().info("SLEEPING=="+str(i)+" seconds")
            time.sleep(1)
        

        second_read = self.laser_forward
        self.get_logger().info(f"Second laser read: {second_read}")

        # Calculate delta
        delta = second_read - first_read
        self.get_logger().info(f"Delta: {delta}")

        # Check delta and publish movement
        if delta > 0:
            self.get_logger().info("Delta is positive: Going FORWARD")
            self.cmd.linear.x = 0.2
            self.cmd.angular.z = 0.0
        else:
            self.get_logger().info("Delta is negative or zero: Going BACKWARD")
            self.cmd.linear.x = -0.2
            self.cmd.angular.z = 0.0
        
        self.publisher_.publish(self.cmd)

        # Set response
        response.success = True
        response.message = "Calculations completed"
        return response




def main(args=None):
    rclpy.init(args=args)

    robot_status_service = RobotStatusService()
    robot_calculation = CalculationClass()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(robot_status_service)
    executor.add_node(robot_calculation)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass

    # Destroy the node explicitly
    robot_status_service.destroy_node()
    robot_calculation.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()




