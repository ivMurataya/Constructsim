#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
from robot_control.srv import RotateRobot, RotateRobotResponse
import time
from sensor_msgs.msg import LaserScan
import math


class RobotControl():

    def __init__(self, start_service=True): 
        rospy.init_node('robot_control_node', anonymous=True)
        self.vel_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        if start_service:
            self.service_server = rospy.Service('rotate_robot', RotateRobot, self.srv_callback)
        rospy.loginfo("Class instanciate")
        self.cmd = Twist()
        self.ctrl_c = False
        self.rate = rospy.Rate(10)
        rospy.on_shutdown(self.shutdownhook)

    def srv_callback(self, request):
        # Initilize velocities
        self.cmd.linear.x = 0
        self.cmd.linear.y = 0
        self.cmd.linear.z = 0
        self.cmd.angular.x = 0
        self.cmd.angular.y = 0
        response = RotateRobotResponse()

        self.convert_degree_to_rad(request.speed_d, request.angle_d)

        if request.clockwise_yn == "y":
            self.clockwise = True
        elif request.clockwise_yn == "n":
            self.clockwise = False
        else:
            response.rotation_successfull = False
            return response
                



        if self.clockwise:
            print ("Clockwise")
            self.cmd.angular.z = -abs(self.angular_speed_r)
        else:
            print ("Not clockwise")
            self.cmd.angular.z = abs(self.angular_speed_r)

        # t0 is the current time
        t0 = rospy.Time.now().secs

        current_angle = 0

        # loop to publish the velocity estimate, current_distance = velocity * (t1 - t0)
        while (current_angle < self.angle_r):

            # Publish the velocity
            self.vel_publisher.publish(self.cmd)
            # t1 is the current time
            t1 = rospy.Time.now().secs
            # Calculate current_distance
            current_angle = self.angular_speed_r * (t1 - t0)
            # ros::spinOnce();
            self.rate.sleep()

        # set velocity to zero to stop the robot
        self.stop_robot()
        
        response.rotation_successfull = True
        return response

    def publish_once_in_cmd_vel(self):
        """
        This is because publishing in topics sometimes fails the first time you publish.
        In continuos publishing systems there is no big deal but in systems that publish only
        once it IS very important.
        """
        while not self.ctrl_c:
            connections = self.vel_publisher.get_num_connections()
            if connections > 0:
                self.vel_publisher.publish(self.cmd)
                #rospy.loginfo("Cmd Published")
                break
            else:
                self.rate.sleep()

    def shutdownhook(self):
        # works better than the rospy.is_shutdown()
        self.stop_robot()
        self.ctrl_c = True

    def stop_robot(self):
        #rospy.loginfo("shutdown time! Stop the robot")
        self.cmd.linear.x = 0.0
        self.cmd.angular.z = 0.0
        self.publish_once_in_cmd_vel()

    def convert_degree_to_rad(self, speed_deg, angle_deg):

        self.angular_speed_r = speed_deg * 3.14 / 180
        self.angle_r = angle_deg * 3.14 / 180
        return [self.angular_speed_r, self.angle_r]

    def rotate(self):

        # Initilize velocities
        self.cmd.linear.x = 0
        self.cmd.linear.y = 0
        self.cmd.linear.z = 0
        self.cmd.angular.x = 0
        self.cmd.angular.y = 0

        speed_d, angle_d = self.get_inputs_rotate()
        self.convert_degree_to_rad(speed_d, angle_d)

        print (self.clockwise)

        if self.clockwise:
            self.cmd.angular.z = -abs(self.angular_speed_r)
        else:
            self.cmd.angular.z = abs(self.angular_speed_r)

        # t0 is the current time
        t0 = rospy.Time.now().secs

        current_angle = 0

        # loop to publish the velocity estimate, current_distance = velocity * (t1 - t0)
        while (current_angle < self.angle_r):

            # Publish the velocity
            self.vel_publisher.publish(self.cmd)
            # t1 is the current time
            t1 = rospy.Time.now().secs
            # Calculate current_distance
            current_angle = self.angular_speed_r * (t1 - t0)
            # ros::spinOnce();
            self.rate.sleep()

        # set velocity to zero to stop the robot
        self.stop_robot()

    def get_inputs_rotate(self):
        self.angular_speed_d = int( raw_input('Enter desired angular speed (degrees): '))
        self.angle_d = int(raw_input('Enter desired angle (degrees): '))
        clockwise_yn = raw_input('Do you want to rotate clockwise? (y/n): ')
        if clockwise_yn == "y":
            self.clockwise = True
        if clockwise_yn == "n":
            self.clockwise = False

        return [self.angular_speed_d, self.angle_d]

if __name__ == '__main__':
    #rospy.init_node('robot_control_node', anonymous=True)
    robotcontrol_object = RobotControl()
    rospy.spin()


'''
## Description

This ROS node implements the RobotControl class, which allows a mobile robot to rotate by publishing angular velocity commands. 
It also provides a service called /rotate_robot that enables the robot to rotate by a specified angle, speed, and direction (clockwise or counterclockwise).

## Features

- Publishes velocity commands to the /cmd_vel topic using Twist messages.
- Provides a service /rotate_robot (type RotateRobot.srv) that:
    - Takes input parameters: angular speed (degrees/sec), angle (degrees), and direction (y or n for clockwise).
    - Converts degrees to radians and sends angular commands until the target rotation is reached.
- Includes helper methods for:
    - Publishing once to /cmd_vel safely.
    - Stopping the robot on shutdown.
    - Manual command-line rotation input.

## Usage

Run the node with:

bash
rosrun robot_control rotate_robot_complete.py
'''