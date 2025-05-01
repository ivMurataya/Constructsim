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


def go_to(xg, yg, thetag_degrees, constant_vel = None):
    rho = float("inf")
    thetag = math.radians(thetag_degrees)
    while rho>0.01:
        dx = xg - odometry.odom_pose['x']
        dy = yg - odometry.odom_pose['y']
        rho = np.sqrt(dx**2 + dy**2)
        theta = odometry.odom_pose['theta']
        alpha = normalize(np.arctan2(dy, dx) - theta)
        beta = normalize(thetag - np.arctan2(dy, dx))
        v = k_rho * rho
        w = k_alpha * alpha + k_beta * beta
        if constant_vel:
            abs_v = abs(v)
            v = v / abs_v * constant_vel
            w = w / abs_v * constant_vel
        velocity.move(v, w)
        rospy.sleep(0.01)
        
k_rho = 0.3
k_alpha = 0.8
k_beta = -0.15


velocity = VelocityController('/cmd_vel')
odometry = OdometryReader('/odom')

go_to(3, 1, 0)

velocity.move(0,0)
odometry.unregister()
error = math.hypot(odometry.odom_pose['x'], odometry.odom_pose['y'])
print('Final positioning error is %.2fm' % error)

