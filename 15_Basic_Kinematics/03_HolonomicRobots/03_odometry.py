#! /usr/bin/env python

from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion
import rospy, numpy as np
from std_msgs.msg import Float32MultiArray

def odom_callback(msg):
    global phi   
    position = msg.pose.pose.position
    (_, _, phi) = euler_from_quaternion([msg.pose.pose.orientation.x, 
                                         msg.pose.pose.orientation.y, 
                                         msg.pose.pose.orientation.z, 
                                         msg.pose.pose.orientation.w])

# Initialize ROS node
rospy.init_node("holonomic_controller", anonymous=True)
pub = rospy.Publisher('wheel_speed', Float32MultiArray, queue_size=10)
position_sub = rospy.Subscriber("/odom", Odometry, odom_callback)
rospy.sleep(1)
    
def velocity2twist(dphi, dx, dy):
    A = np.array([
        [ 1, 0, 0],  # Wheel 1
        [ 0, np.cos(phi),  np.sin(phi)],  # Wheel 2
        [ 0, -np.sin(phi), np.cos(phi)],  # Wheel 3
    ])
    b = np.array([
        [dphi],  # angular velocity around z
        [dx],  # linear velocity in x
        [dy]   # linear velocity in y
    ])
    wz, vx, vy =  np.dot(A, b).flatten()
    print( wz,vx,vy)
    return wz, vx, vy

def twist2wheels(wz, vx, vy):
    R = 0.254 / 2 
    L = 0.500 / 2 
    W = 0.548 / 2 
    A = np.array([
        [ -L - W, 1, -1],  # Wheel 1
        [  L + W, 1,  1],  # Wheel 2
        [  L + W, 1, -1],  # Wheel 3
        [ -L - W, 1,  1]   # Wheel 4
    ])
    b = np.array([
        [wz],  # angular velocity around z
        [vx],  # linear velocity in x
        [vy]   # linear velocity in y
    ])
    result = (1/R) * np.dot(A, b)
    print(result)
    return result




for _ in range(100):
    wz, vx, vy = velocity2twist(dphi=1.57, dx=3, dy=2)
    u = twist2wheels(wz, vx, vy)
    msg = Float32MultiArray(data=u)
    pub.publish(msg)
    rospy.sleep(0.01)
    

stop = [0,0,0,0]
msg = Float32MultiArray(data=stop)
pub.publish(msg)

