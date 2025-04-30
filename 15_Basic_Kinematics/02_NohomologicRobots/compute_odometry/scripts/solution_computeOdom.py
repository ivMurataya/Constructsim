#! /usr/bin/env python

import rospy
from math import pi, sin, cos
from geometry_msgs.msg import Twist, Pose, Point, Quaternion, Vector3
from simple_robot_gazebo.msg import encoders
from nav_msgs.msg import Odometry
from rosgraph_msgs.msg import Clock
import tf
from tf.broadcaster import TransformBroadcaster

class OdometryClass:

    def __init__(self):
        # Subscribe to encoder data
        self.ticks_sub = rospy.Subscriber('/encoders', encoders, self.getTicks)
        # Publisher for odometry data
        self.odom_pub = rospy.Publisher('/odom', Odometry, queue_size=1)
        # Broadcaster for tf (odom -> base_link)
        self.odom_broadcaster = TransformBroadcaster()
        self.odom = Odometry()
        self.rate = rospy.Rate(1)  # 1 Hz update rate

        # Initialize encoder tick values
        self.lastLeftTicks = 0
        self.lastRightTicks = 0
        self.currentLeftTicks = 0
        self.currentRightTicks = 0

        # Timestamps
        self.current_time = rospy.Time.now()
        self.last_time = rospy.Time.now()

        # Robot parameters
        self.L = 0.3    # Wheelbase in meters
        self.R = 0.1    # Wheel radius in meters
        self.N = 360    # Ticks per revolution

        # Robot pose (x, y, theta)
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0

        # Wait for simulated time to start
        rospy.wait_for_message('/clock', Clock)
        # Start the odometry update loop
        self.updatePose()

    def getTicks(self, msg):
        # Get encoder tick values from incoming message
        self.currentLeftTicks = msg.encoderTicks[1]   # Assuming index 1 is left wheel
        self.currentRightTicks = msg.encoderTicks[0]  # Assuming index 0 is right wheel
        
    def updatePose(self):
        while not rospy.is_shutdown():
            # Calculate tick difference since last update
            delta_l = self.currentLeftTicks - self.lastLeftTicks
            delta_r = self.currentRightTicks - self.lastRightTicks

            # Convert tick difference to distance (meters)
            d_l = 2 * pi * self.R * delta_l / self.N
            d_r = 2 * pi * self.R * delta_r / self.N

            # Update last tick values
            self.lastLeftTicks = self.currentLeftTicks
            self.lastRightTicks = self.currentRightTicks

            # Compute time elapsed
            self.current_time = rospy.Time.now()
            dt = (self.current_time - self.last_time).to_sec()

            if dt == 0.0:
                continue  # Avoid division by zero

            # Compute linear and angular velocities
            v = ((d_r + d_l) / 2) / dt        # Linear velocity
            w = ((d_r - d_l) / self.L) / dt   # Angular velocity

            # Estimate change in position and orientation
            delta_x = v * cos(self.th)
            delta_y = v * sin(self.th)
            delta_th = w

            # Update robot's pose
            self.x += delta_x
            self.y += delta_y
            self.th += delta_th

            # Convert yaw (theta) to quaternion
            odom_quat = tf.transformations.quaternion_from_euler(0, 0, self.th)

            # Broadcast transform (odom → link_chassis)
            self.odom_broadcaster.sendTransform(
                (self.x, self.y, 0.),
                odom_quat,
                self.current_time,
                "link_chassis",  # Child frame
                "odom"           # Parent frame
            )

            # Prepare odometry message
            odom = Odometry()
            odom.header.stamp = self.current_time
            odom.header.frame_id = "odom"

            # Set robot pose
            odom.pose.pose = Pose(Point(self.x, self.y, 0.), Quaternion(*odom_quat))

            # Set robot velocity
            odom.child_frame_id = "link_chassis"
            odom.twist.twist = Twist(Vector3(v, 0, 0), Vector3(0, 0, w))

            # Publish odometry message
            self.odom_pub.publish(odom)

            # Update time for next loop
            self.last_time = self.current_time

            # Sleep to maintain loop rate
            self.rate.sleep()

if __name__ == "__main__":
    rospy.init_node('pub_odom')
    rospy.loginfo("Odometry node initialized.")
    oc = OdometryClass()
    rospy.spin()
