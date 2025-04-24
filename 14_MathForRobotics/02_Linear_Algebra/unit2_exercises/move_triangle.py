#!/usr/bin/env python

import rospy
from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
import numpy as np
import time, math

##################################################  HELPER FUNCTIONS  ############################################

def sum_of_vectors(v1, v2):
    return v1 + v2

def angle_between_vectors(v1, v2):
    v_prod = np.dot(v1, v2)
    v1_norm = np.linalg.norm(v1)
    v2_norm = np.linalg.norm(v2)
    angle = np.arccos(v_prod / (v1_norm * v2_norm))
    angle = angle * 180 / np.pi
    return angle

def calc_length(vec):
    return np.linalg.norm(vec)

def calc_hypotenusa(len_1, len_2):
    return np.sqrt(len_1**2 + len_2**2)

def calculate_triangle_parameters(leg_1,leg_2):
    
    # DO: Call the sum of vectors to sum leg_1 and leg_2
    hypotenusa = sum_of_vectors(leg_1,leg_2)
    
    # DO: Call the angle between vectors twice, to calculate angle between leg_1 and leg_2, and angle between leg_2 and vector sum
    angle_12 = angle_between_vectors(leg_1,leg_2)
    angle_2sum = angle_between_vectors(leg_2,hypotenusa)
    
    # DO: Call the length of a vector twice, for leg_1 and leg_2
    leg_1_len = calc_length(leg_1)
    leg_2_len = calc_length(leg_2)
    
    # DO: Call the calculate hypotenusa, for the lengths of leg_1 and leg_2
    hypotenusa_len = calc_hypotenusa(leg_1_len,leg_2_len)
    
    print('Orthogonal vectors leg_1 and leg_2: ',leg_1,leg_2)
    print('Vector hypotenusa of the triangle: ',hypotenusa)
    print('Angles between legs: ',angle_12, angle_2sum)
    print('Length of legs and hypotenusa: ',leg_1_len,leg_2_len,hypotenusa_len)
    
    #return results
    return leg_1_len,leg_2_len,hypotenusa_len,angle_12,angle_2sum

##################################################################################################################

# ROS Node Class
class TriangleMovement:
    def __init__(self):
        rospy.init_node('triangle_movement')
        self.pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)
        
        self.current_pose = Pose()
        self.position_ini = Pose()
        self.rate = rospy.Rate(10)

    # Odometry Callback
    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose

    # Move robot straight a certain distance
    def move_straight(self, distance, speed=0.2):
        time.sleep(1)
        vel_msg = Twist()
        vel_msg.linear.x = speed
        current_distance = 0
        self.reset_position_ini()
        while current_distance < distance:
            current_distance = self.calc_distance(self.current_pose.position, self.position_ini.position)
            self.pub.publish(vel_msg)
            self.rate.sleep()

        vel_msg.linear.x = 0
        self.pub.publish(vel_msg)

    # Rotate robot a certain angle
    def rotate(self, angle, speed=0.1):
        vel_msg = Twist()
        vel_msg.angular.z = speed
        start_time = rospy.Time.now().to_sec()
        current_angle = 0

        while current_angle < angle:
            self.pub.publish(vel_msg)
            self.rate.sleep()
            current_time = rospy.Time.now().to_sec()
            current_angle = speed * (current_time - start_time)
        
        vel_msg.angular.z = 0
        self.pub.publish(vel_msg)

    # Reset robot's initial position
    def reset_position_ini(self):
        self.position_ini = Pose()
        self.position_ini.position = self.current_pose.position
        self.position_ini.orientation = self.current_pose.orientation

    # Calculate distance moved by robot
    def calc_distance(self,vec_1,vec_2):
        return math.sqrt((vec_1.x-vec_2.x)**2 + (vec_1.y-vec_2.y)**2)

    # Triangle movement with Pythagorean Theorem
    def run(self):
        leg_1 = np.array([4.0, 0.0])
        leg_2 = np.array([0.0, 3.0])
        leg_1_len, leg_2_len, hypotenusa_len, angle_12, angle_2sum = calculate_triangle_parameters(leg_1, leg_2)
        
        self.move_straight(leg_1_len)
        self.rotate(np.radians(angle_12))
        self.move_straight(leg_2_len)
        self.rotate(np.radians(90 + angle_2sum))  # Adjust based on actual angle required
        self.move_straight(hypotenusa_len)

if __name__ == '__main__':
    try:
        triangle_movement = TriangleMovement()
        triangle_movement.run()
    except rospy.ROSInterruptException:
        pass
