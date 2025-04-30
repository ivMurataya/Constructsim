#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Twist, Quaternion, Pose, Point, Vector3
from std_msgs.msg import Float64, UInt32, Float32
from simple_robot_gazebo.msg import encoders
from math import pi, sin, cos
from nav_msgs.msg import Odometry
from tf.transformations import quaternion_from_euler
from tf.broadcaster import TransformBroadcaster

# Constants
N = 360     # Encoder ticks per wheel revolution
R = 0.1     # Wheel radius in meters
L = 0.3     # Distance between wheels (wheelbase)

class createOdometry():
    def __init__(self):
        # Subscribe to encoder data
        self.sub = rospy.Subscriber('/encoders', encoders, self.callback)
        # Publisher for Odometry message
        self.pub = rospy.Publisher('/odom', Odometry , queue_size=1)
        # TF broadcaster for odom -> base transform
        self.odom_broadcaster = TransformBroadcaster()
        # Prepare Odometry message and related variables
        self.odomMsg = Odometry()
        self.twist_vels = Twist()
        self.rate = rospy.Rate(1)

        # Initialize encoder tick values and timestamps
        self.currentLeftTik = 0.1
        self.currentRightTik = 0.1
        self.lastLeftTik = 0.1
        self.lastRightTik = 0.1
        self.currtimeSt = rospy.Time.now()
        self.lastTimeSt = rospy.Time.now()

        # Wheel travel distances and pose estimates
        self.dR = 0.1
        self.dL = 0.1
        self.phi = 0.0      # Robot orientation (yaw)
        self.x = 0.0        # Robot X position
        self.y = 0.0        # Robot Y position

        # Start odometry calculation loop
        self.getDelta()

    def callback(self, msg):
        # Update current encoder tick readings from incoming message
        self.currentLeftTik = msg.encoderTicks[0]
        self.currentRightTik = msg.encoderTicks[1]
        # Uncomment below if using timestamp from message
        # self.currtimeSt = msg.timeStamp

    def getDelta(self):
        # Main odometry computation loop
        while not rospy.is_shutdown():
            rate = rospy.Rate(1)

            # Calculate change in encoder ticks since last cycle
            delta_l = self.currentLeftTik - self.lastLeftTik
            delta_r = self.currentRightTik - self.lastRightTik

            # Convert ticks to linear distances
            self.dL = (2 * pi * R) * (delta_l / N)
            self.dR = (2 * pi * R) * (delta_r / N)

            # Update last tick values for next iteration
            self.lastRightTik = self.currentRightTik
            self.lastLeftTik = self.currentLeftTik

            # Calculate elapsed time
            self.currtimeSt = rospy.Time.now()
            delta_T = float((self.currtimeSt - self.lastTimeSt).to_sec())

            if delta_T == 0.0:
                continue  # Avoid division by zero

            # Compute linear and angular velocities
            DC = (self.dL + self.dR) / 2                 # Average distance
            Vel = DC / delta_T                           # Linear velocity
            angular = ((self.dR - self.dL) / L) / delta_T # Angular velocity

            # Update robot orientation and position
            self.phi += angular
            self.x += Vel * cos(self.phi)
            self.y += Vel * sin(self.phi)

            # Print debug info
            rospy.loginfo("Velocity: %f , Angular: %f , X: %f, Y: %f , P: %f", Vel, angular, self.x, self.y, self.phi)

            # Convert yaw to quaternion for TF and odometry
            odom_quat = quaternion_from_euler(0, 0, self.phi)

            # Broadcast the transform over tf
            self.odom_broadcaster.sendTransform(
                (self.x, self.y, 0.0),     # Translation
                odom_quat,                 # Rotation as quaternion
                self.currtimeSt,           # Time
                "link_chassis",            # Child frame (robot base)
                "odom"                     # Parent frame (odometry frame)
            )

            # Create and publish the Odometry message
            odom = Odometry()
            odom.header.stamp = self.currtimeSt
            odom.header.frame_id = "odom"

            # Set the robot position and orientation
            odom.pose.pose = Pose(Point(self.x, self.y, 0.), Quaternion(*odom_quat))

            # Set the robot linear and angular velocity
            odom.child_frame_id = "link_chassis"
            odom.twist.twist = Twist(Vector3(Vel, 0, 0), Vector3(0, 0, angular))

            # Publish odometry data
            self.pub.publish(odom)

            # Save current time for next loop
            self.lastTimeSt = self.currtimeSt

            rate.sleep()

# Node entry point
if __name__ == '__main__':
    rospy.init_node('convert_vels_node', anonymous=True)
    rospy.loginfo("Convert velocities node initialized.")
    cv = createOdometry()
    rospy.spin()
