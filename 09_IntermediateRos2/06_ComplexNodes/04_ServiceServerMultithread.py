#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool
from geometry_msgs.msg import Twist
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
import time



class RobotStatusService(Node):
    def __init__(self):
        super().__init__('robot_move_service')

        #For Multithreading
        #self.mutuallyexclusive_group_1 = MutuallyExclusiveCallbackGroup()
        #self.mutuallyexclusive_group_2 = MutuallyExclusiveCallbackGroup()

        name_service = '/start_turn'
        #self.srv = self.create_service(SetBool, name_service, self.SetBool_callback,callback_group=self.mutuallyexclusive_group_1)
        #self.timer = self.create_timer(0.5, self.timer_callback,callback_group=self.mutuallyexclusive_group_2)
        self.srv = self.create_service(SetBool, name_service, self.SetBool_callback)      
        self.timer = self.create_timer(0.5, self.timer_callback)

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
    

def main(args=None):
    rclpy.init(args=args)

    robot_status_service = RobotStatusService()
    #Multithreading
    #executor = MultiThreadedExecutor(num_threads=2)
    #executor.add_node(robot_status_service)

    try:
        #Multithreading
        #executor.spin()

        #Single Thread
        rclpy.spin(robot_status_service)
    except KeyboardInterrupt:
        pass

    # Destroy the node explicitly
    robot_status_service.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()



"""

    A service server called /start_turn, that uses message type std_srvs/SetBool. Its Callback has to be named SetBool_callback. Depending on the value of the service request, if TRUE, it has to set an internal class variable self.cmd of type geometry_msgs/Twist to make the robot TURN at 0.3 radiant per second. If False, set the self.cmd to make the robot GO FORWARD.
    Inside the SetBool_callback you need to set a time.sleep(15.0) (15.0 seconds wait). The reason for setting this is to see the multithreading behavior better.

    Also, print the time it is sleeping so you can see when you are executing this Callback, especially in the waiting phase.

    A rostopic publisher published in the /cmd_vel topic, named publisher_ .
    A timer that loops every 1.0 seconds, and publishes to the publisher_ the self.cmd values set in the SetBool_callback. In the init method, initialize the self.cmd variable to self.cmd = Twist(). Please log the self.cmd sent in each loop to see better the loop circle.

"""
"""
Request is TRUE: Setting to TURN
[INFO] [1739826469.508627807] [robot_move_service]: Sleeping... 1 seconds
[INFO] [1739826469.969731066] [robot_move_service]: Publishing cmd: linear.x=0.0, angular.z=0.3
[INFO] [1739826470.465058875] [robot_move_service]: Publishing cmd: linear.x=0.0, angular.z=0.3
[INFO] [1739826470.510143993] [robot_move_service]: Sleeping... 2 seconds
[INFO] [1739826470.970538489] [robot_move_service]: Publishing cmd: linear.x=0.0, angular.z=0.3
[INFO] [1739826471.464843013] [robot_move_service]: Publishing cmd: linear.x=0.0, angular.z=0.3
[INFO] [1739826471.511711074] [robot_move_service]: Sleeping... 3 seconds
[INFO] [1739826471.965598861] [robot_move_service]: Publishing cmd: linear.x=0.0, angular.z=0.3
[INFO] [1739826472.465496654] [robot_move_service]: Publishing cmd: linear.x=0.0, angular.z=0.3
[INFO] [1739826472.513870087] [robot_move_service]: Sleeping... 4 seconds
[INFO] [1739826472.965654588] [robot_move_service]: Publishing cmd: linear.x=0.0, angular.z=0.3
[INFO] [1739826473.465157674] [robot_move_service]: Publishing cmd: linear.x=0.0, angular.z=0.3
[INFO] [1739826473.516146741] [robot_move_service]: Sleeping... 5 seconds
[INFO] [1739826473.965074297] [robot_move_service]: Publishing cmd: linear.x=0.0, angular.z=0.3
[INFO] [1739826474.464871605] [robot_move_service]: Publishing cmd: linear.x=0.0, angular.z=0.3
[INFO] [1739826474.518287132] [robot_move_service]: Sleeping... 6 seconds
"""
