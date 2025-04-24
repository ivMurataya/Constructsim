#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Twist, Pose
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float64
from tf.transformations import euler_from_quaternion, quaternion_from_euler

import math, time
import numpy as np


##############################   VELOCITY PARAMETERS  ##################################
VX = 0.6
WZ = 0.3
AX = 0.03
########################################################################################

class TurtlebotProject():
    def __init__(self):
        # Positions
        self.odom_topic = '/odom'
        self.odom_sub = rospy.Subscriber(self.odom_topic, Odometry,self.odomCallback)
        self.current_position = Pose()
        self.yaw = 0.0
        self.prev_yaw = 0.0

        # Velocities
        self.vel_topic = '/cmd_vel'
        self.vel_pub = rospy.Publisher(self.vel_topic, Twist, queue_size=10)
        self.vel_msg = Twist()
        self.vx,self.wz = 0,0
        self.v0 = 0.0
        # Acceleration
        self.ax = AX

        # Laser
        self.laser_topic = '/kobuki/laser/scan'
        self.laser_sub = rospy.Subscriber(self.laser_topic,LaserScan, self.laserCallback)
        self.laser_dist = 100000 #laser reading at the front
        self.margin = 0.6 # margin to avoid the wall

        # Rate of publishing
        self.rate = rospy.Rate(10)

        # Time
        self.init_time = rospy.get_time()

    
    def odomCallback(self,odom_msg):
        # position
        self.current_position = odom_msg.pose.pose
        # angle
        yaw = get_angle_from_pose(self.current_position)
        if( yaw < 0 ): yaw += np.pi
        if abs(yaw-self.prev_yaw) > 3.0: yaw +=np.pi
        self.yaw = yaw
        self.prev_yaw = self.yaw

    def laserCallback(self,laser_msg):
        self.laser_dist = laser_msg.ranges[360]

    def move(self):
        time.sleep(2)
        rospy.loginfo('Choosing left or right..................')
        if decide_left_or_right():
            self.traj_right()
        else:
            self.traj_left()

        
    def traj_right(self):
        time.sleep(1)
        rospy.loginfo('Initiating movement to the right ..........')
        self.run_hallway(self.laser_dist-self.margin)
        time.sleep(1)
        self.turn(-90.0)
        time.sleep(1)
        self.run_hallway(self.laser_dist-self.margin)
        time.sleep(1)
        self.turn(-90.0)
        time.sleep(1)
        self.run_hallway(self.laser_dist-self.margin)
        time.sleep(1)
        self.turn(+90.0)
        time.sleep(1)
        self.escape()

    def traj_left(self):
        time.sleep(1)
        rospy.loginfo('Initiating movement to the left ..........')
        self.turn(180.0)
        time.sleep(1)
        self.run_hallway(self.laser_dist-self.margin)
        time.sleep(1)
        self.turn(90.0)
        time.sleep(1)
        self.run_hallway(self.laser_dist-self.margin)
        time.sleep(1)
        self.turn(90.0)
        time.sleep(1)
        self.run_hallway(self.laser_dist-self.margin)
        time.sleep(1)
        self.turn(-90.0)
        time.sleep(1)
        self.escape()

    def run_hallway(self,total_dist):
        rospy.loginfo('Running down the hallway, to distance '+str(total_dist))
        run = True
        dist_ini = self.laser_dist
        self.halfway_done = False
        while run and not rospy.is_shutdown():
            run = self.change_acceleration(total_dist,dist_ini)
            t = rospy.get_time() - self.init_time
            vx = self.v0 + self.ax*t
            self.vx,self.wz = vx, 0.0
            self.publish_vel(self.vx,self.wz)
            self.rate.sleep()
        else:
            rospy.loginfo('HALLWAY COMPLETED')

    def change_acceleration(self,total_dist,dist_ini):
        dist_run = abs(self.laser_dist-dist_ini)
        if dist_run < total_dist/2 and not self.halfway_done:
            # first half of the hallway
            self.ax = +AX
        elif dist_run > total_dist/2 and not self.halfway_done:
            # first half done
            self.v0,self.ax = self.vx,0.0
            self.halfway_done = True
            self.init_time = rospy.get_time()
        elif dist_run < total_dist and self.halfway_done:
            # second half of the hallway
            self.ax = -AX
        elif dist_run > total_dist and self.halfway_done:
            # second half done
            self.v0,self.ax = 0.0,0.0
            self.halfway_done = False
            return False
        return True

    def turn(self,target_angle):
        target_angle = target_angle*np.pi/180.0 + self.yaw
        error_angle = target_angle - self.yaw
        while abs(error_angle) > 0.15 and not rospy.is_shutdown():
            error_angle = target_angle - self.yaw
            command_angle = 0.2 * error_angle
            command_vel = 0.0
            self.publish_vel(command_vel,command_angle)
            self.rate.sleep()
        else:
            rospy.loginfo('TURN COMPLETED')

    def escape(self):
        while not rospy.is_shutdown():
            self.vx,self.wz = VX,0.0
            self.publish_vel(self.vx,self.wz)
            self.rate.sleep()
        else:
            self.stop_robot()

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
def decide_left_or_right(limit=0.8):
    number = np.random.uniform(0,1) # get a random number from a uniform distribution
    print('The random number generated is: ',number)
    
    if number<limit:
        return True
    else:
        return False

def get_angle_from_pose(pose):
    orient_list = [pose.orientation.x,pose.orientation.y,pose.orientation.z,pose.orientation.w]
    (roll,pitch,yaw) = euler_from_quaternion(orient_list)
    return yaw
####################################################################
