#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
from tf.transformations import euler_from_quaternion, quaternion_from_euler

import math, time
import numpy as np

from move_triangle import calculate_triangle_parameters

# VELOCITY PARAMETERS
VX = 0.5
WZ = 0.3

class TurtlebotMoves():
    def __init__(self,leg_1,leg_2):
        # Positions
        self.odom_topic = '/odom'
        self.odom_sub = rospy.Subscriber(self.odom_topic, Odometry,self.odomCallback)
        self.current_position = Pose()
        self.position_ini = Pose()
        self.yaw = 0.0
        self.prev_yaw = 0.0

        # Velocities
        self.vel_topic = '/cmd_vel'
        self.vel_pub = rospy.Publisher(self.vel_topic, Twist, queue_size=10)
        self.vel_msg = Twist()
        self.vx,self.wz = 0,0

        # Rate of publishing
        self.rate = rospy.Rate(10)

        # triangle movement with Pythagorean Theorem
        rospy.loginfo('Calculating triangle parameters ..............')
        self.leg_1_len,self.leg_2_len,self.hypotenusa_len,self.angle_12,self.angle_2sum = calculate_triangle_parameters(leg_1,leg_2)
        self.leg_1_done, self.leg_2_done, self.hypotenusa_done = False,False,False
        rospy.loginfo('Triange parameters calculated.')
    
    def odomCallback(self,odom_msg):
        self.current_position = odom_msg.pose.pose
        yaw = get_angle_from_pose(self.current_position)
        if( yaw < 0 ): yaw += np.pi
        if abs(yaw-self.prev_yaw) > 3.0: yaw +=np.pi
        self.yaw = yaw
        self.prev_yaw = self.yaw
        #rospy.loginfo('Current position: '+str(self.yaw))

    def move(self):
        rospy.loginfo('Waiting for the first position lecture..........')
        time.sleep(2)
        self.reset_position_ini()
        rospy.loginfo('Initiating movement ....................')
        while not rospy.is_shutdown():
            self.vx,self.wz = self.decide()
            self.publish_vel(self.vx,self.wz)
            self.rate.sleep()
        else: 
            self.stop_robot()

    def decide(self):
        dist = calc_distance(self.current_position.position,self.position_ini.position)
        if dist<self.leg_1_len and not self.leg_1_done:
            self.vx,self.wz = VX,0.0
        elif dist>self.leg_1_len and not self.leg_1_done:
            print('LEG 1 DONE ------------------------------------------------')
            self.turn(self.angle_12)
            self.leg_1_done = True
            dist = 0.0
            self.reset_position_ini()
        elif self.leg_1_done:
            if dist<self.leg_2_len and not self.leg_2_done:
                self.vx,self.wz = VX,0.0
            elif dist>self.leg_2_len and not self.leg_2_done:
                print('LEG 2 DONE ---------------------------------------------')
                self.turn(self.angle_2sum+90.0)
                self.leg_2_done = True
                dist = 0.0
                self.reset_position_ini()
            elif self.leg_2_done:
                if dist<self.hypotenusa_len and not self.hypotenusa_done:
                    self.vx,self.wz = VX,0.0
                elif dist>self.hypotenusa_len and not self.hypotenusa_done:
                    self.hypotenusa_done = True
                elif self.hypotenusa_done:
                    print('END OF THE MOVEMENT  -------------------------------')
                    self.vx,self.wz = 0.0,0.0
        return self.vx,self.wz

    def turn(self,target_angle):
        target_angle = target_angle*np.pi/180.0 + self.yaw
        error_angle = target_angle - self.yaw
        while abs(error_angle) > 0.1 and not rospy.is_shutdown():
            error_angle = target_angle - self.yaw
            command_angle = 0.2 * error_angle
            command_vel = 0.0
            self.publish_vel(command_vel,command_angle)
            self.rate.sleep()
        else:
            print('TURN COMPLETED')

    def reset_position_ini(self):
        self.position_ini = Pose()
        self.position_ini.position = self.current_position.position
        self.position_ini.orientation = self.current_position.orientation

    def stop_robot(self):
        vx = 0.0
        wz = 0.0
        self.publish_vel(vx,wz)
        rospy.signal_shutdown('End of movement')

    def publish_vel(self,vx,wz):
        self.vel_msg.linear.x = vx
        self.vel_msg.angular.z = wz
        # publish velocity message
        #rospy.loginfo('Publishing vx: {} and wz: {}'.format(self.vel_msg.linear.x,self.vel_msg.angular.z))
        self.vel_pub.publish(self.vel_msg)


################### HELPER FUNCTIONS ##############################
def calc_distance(vec_1,vec_2):
    return math.sqrt((vec_1.x-vec_2.x)**2 + (vec_1.y-vec_2.y)**2)

def calc_angle(vec):
    return math.atan2(vec.y,vec.x)

def get_angle_from_pose(pose):
    orient_list = [pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w]
    (roll,pitch,yaw) = euler_from_quaternion(orient_list)
    return yaw
####################################################################
