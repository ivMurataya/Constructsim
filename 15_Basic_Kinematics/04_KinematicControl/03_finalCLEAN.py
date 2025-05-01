import rospy
import math
import numpy as np
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion

# Initialize the ROS node
rospy.init_node('Kinematic_controller', anonymous=True)

class VelocityController:
    def __init__(self, topic):
        self.cmd_vel = rospy.Publisher(topic, Twist, queue_size=10)
        rospy.sleep(0.1)

    def move(self, linear_velocity=0.0, angular_velocity=0.0):
        msg = Twist()
        msg.linear.x = linear_velocity
        msg.angular.z = angular_velocity
        self.cmd_vel.publish(msg)

class OdometryReader:
    def __init__(self, topic):
        self.odom_pose = {}
        self.trajectory = []
        self.topic = topic
        self.subscribe()
    
    def callback(self, msg):
        self.odom_pose['x'] = msg.pose.pose.position.x
        self.odom_pose['y'] = msg.pose.pose.position.y
        self.trajectory.append((self.odom_pose['x'], self.odom_pose['y']))
        _, _, self.odom_pose['theta'] = euler_from_quaternion([msg.pose.pose.orientation.x, 
                                                              msg.pose.pose.orientation.y, 
                                                              msg.pose.pose.orientation.z, 
                                                              msg.pose.pose.orientation.w])
    
    def subscribe(self):
        self.odom_subscriber = rospy.Subscriber(self.topic, Odometry, self.callback)
        rospy.sleep(0.1)

    def unregister(self):
        np.save('trajectory', self.trajectory)
        self.odom_subscriber.unregister()

# Normalize the angle to be in the range [-pi, pi]
def normalize(angle):
    return (angle + np.pi) % (2 * np.pi) - np.pi

# Function to move to the goal position
def go_to(xg, yg, thetag_degrees):
    rho = float("inf")
    thetag = math.radians(thetag_degrees)
    while rho > 0.01:
        dx = xg - odometry.odom_pose['x']
        dy = yg - odometry.odom_pose['y']
        rho = np.sqrt(dx**2 + dy**2)  # Distance to goal

        theta = odometry.odom_pose['theta']
        # Compute alpha (angle to goal relative to robot's heading)
        alpha = normalize(-theta + np.arctan2(dy, dx))
        # Compute beta (angle to correct the robot's heading)
        beta = normalize(thetag - np.arctan2(dy, dx))

        # Compute linear velocity (rho scaled by k_rho)
        v = k_rho * rho
        # Compute angular velocity (weighted sum of alpha and beta)
        w = k_alpha * alpha + k_beta * beta

        # Scale velocities to maintain constant linear velocity
        s = 0.3 / abs(v) if v != 0 else 1  # Avoid division by zero
        v_scaled = s * v
        w_scaled = s * w

        velocity.move(v_scaled, w_scaled)
        rospy.sleep(0.01)

# Controller parameters
k_rho = 0.3
k_alpha = 0.8
k_beta = -0.15

# Initialize the velocity controller and odometry reader
velocity = VelocityController('/cmd_vel')
odometry = OdometryReader('/odom')

# Define waypoints (xg, yg, theta)
waypoints = [
    (1, -1, -90), (2, -2, 0), (3, -2, 0), (4, -1, 90), 
    (3.5, -0.5, 180), (3, 0, 90), (3, 1, 90), (2, 1, -90), 
    (1, 0, 180), (0, 0, 180)
]

# List to store the robot's trajectory
ref = [(odometry.odom_pose['x'], odometry.odom_pose['y'])]

# Navigate through each waypoint
for xg, yg, thetag in waypoints:
    go_to(xg, yg, thetag)
    ref.append((odometry.odom_pose['x'], odometry.odom_pose['y']))

# Stop the robot and save the trajectory
velocity.move(0, 0)
odometry.unregister()

# Calculate final positioning error
error = math.hypot(odometry.odom_pose['x'], odometry.odom_pose['y'])

# Save the waypoints and print the final error
np.save('waypoints.npy', ref)
print('Final positioning error is %.2fm' % error)
