import rospy, math, numpy as np 
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import random

rospy.init_node('Kinematic_controller', anonymous=True)

class VelocityController():
    def __init__(self, topic):
        self.cmd_vel = rospy.Publisher(topic, Twist, queue_size=10)
        rospy.sleep(0.1)

    def move(self, linear_velocity = 0.0, angular_velocity = 0.0):
        msg = Twist()
        msg.linear.x = linear_velocity
        msg.angular.z = angular_velocity
        self.cmd_vel.publish(msg)

class OdometryReader():
    def __init__(self,topic):
        self.odom_pose = {}
        self.trajectory = []
        self.wapo = []
        self.topic = topic
        self.subscribe()
    
    def callback(self, msg):
        self.odom_pose['x'] = msg.pose.pose.position.x
        self.odom_pose['y'] = msg.pose.pose.position.y
        self.trajectory.append((self.odom_pose['x'], self.odom_pose['y']))
        (_, _, self.odom_pose['theta']) = euler_from_quaternion([msg.pose.pose.orientation.x, 
                                                            msg.pose.pose.orientation.y, 
                                                            msg.pose.pose.orientation.z, 
                                                            msg.pose.pose.orientation.w])
    def subscribe(self):
        self.odom_subscriber = rospy.Subscriber(self.topic, Odometry, self.callback)
        rospy.sleep(0.1)

    def unregister(self):
        np.save('trajectory',self.trajectory)
        self.odom_subscriber.unregister()


def normalize(angle):
    return (angle + np.pi) % (2 * np.pi) - np.pi

#Function to test normalize 
# You should test the previous function with some angles (first and third columns should be equal):
"""
import random
for _ in range(10):
    angle = (random.random()-0.5)*2*math.pi
    new_angle = angle + (random.randint(0,10)-5) * 2*math.pi
    norm_angle = normalize(new_angle)
    print('%9.4f %9.4f %9.4f' %(angle, new_angle, norm_angle))
"""


def go_to(xg, yg, thetag_degrees,constant_vel = None):
    rho = float("inf")
    thetag = math.radians(thetag_degrees)
    while rho>0.01:
        dx = xg - odometry.odom_pose['x']
        dy = yg - odometry.odom_pose['y']
        rho = np.sqrt((dx)**2 + (dy)**2)
        theta = odometry.odom_pose['theta']
        # Compute alpha (angle to goal relative to robot's heading)
        alpha = normalize( -theta + np.arctan2(dy, dx))
        #beta = normalize(180)

        #alpha = normalize(np.arctan2(dy, dx) - theta)
        beta = normalize(thetag - np.arctan2(dy, dx))
        v = k_rho * rho
        # Compute angular velocity w (weighted sum of alpha and beta)
        w = k_alpha * alpha + k_beta * beta
        if constant_vel:
            s = constant_vel / abs(v) if v != 0 else 1  # Avoid division by zero
            
            # Apply the scaling factor to both linear and angular velocities
            v = s * v
            w = s * w

        velocity.move(v, w)
        rospy.sleep(0.01)
        
k_rho = 0.3
k_alpha = 1.5
k_beta = -0.15


velocity = VelocityController('/cmd_vel')
odometry = OdometryReader('/odom')

#waypoints = [(1,-1,-90),(2,-2,0),(3,-2,0),(4,-1,90),(3.5,-0.5,180),
 #            (3,0,90),(3,1,90),(2,1,-90),(1,0,180),(0,0,180)]


waypoints = [
    (0.0, 0.0, 0.0),    # Start (top middle)
    (1.1666, 1.0088, 0.0),
    (1.4155, 1.5688, 0.0),
    (1.62, 2.182, 0.0),
    (1.4777, 2.831, 0.0),
    (1.0, 3.0, 0.0),     # Bottom tip
    (0.4288, 2.81333, 0.0),
    (0, 2.5, 0.0),
    (-0.5311, 2.8755, 0.0),
    (-1, 3, 0.0),
    (-1.455, 2.7422, 0.0),
    (-1.74, 2.173, 0.0) ,  
    (-1.58, 1.5866, 0.0),
    (-1.242, 0.9733, 0.0),
    (-0.726, 0.528, 0.0),
    (0.677, 0.484, 0.0)
]

ref = []
ref.append((odometry.odom_pose['x'], odometry.odom_pose['y']))
for xg, yg, thetag in waypoints:
    #go_to(xg, yg, thetag)
    go_to(xg/3, yg/3, thetag, 0.2) #For constant Velocity
    ref.append((odometry.odom_pose['x'], odometry.odom_pose['y']))

velocity.move(0,0)
odometry.unregister()
error = math.hypot(odometry.odom_pose['x'], odometry.odom_pose['y'])
np.save('waypoints.npy', ref)
print('Final positioning error is %.2fm' % error)

