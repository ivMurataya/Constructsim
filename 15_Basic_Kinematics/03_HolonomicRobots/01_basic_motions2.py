import rospy, math, numpy as np 
from std_msgs.msg import Float32MultiArray
from utilities import reset_world


rospy.init_node("olonomic_controller", anonymous=True)
pub = rospy.Publisher('wheel_speed', Float32MultiArray, queue_size=10)
rospy.sleep(1.0)

forward = [1, 1, 1, 1]
msg = Float32MultiArray(data=forward)
pub.publish(msg)
rospy.sleep(3.0)

back = [-1, -1, -1, -1]
msg = Float32MultiArray(data=back)
pub.publish(msg)
rospy.sleep(3.0)

left = [-1, 1, -1, 1]
msg = Float32MultiArray(data=left)
pub.publish(msg)
rospy.sleep(3.0)

right = [1, -1, 1, -1]
msg = Float32MultiArray(data=right)
pub.publish(msg)
rospy.sleep(3.0)

counter = [-1, 1, 1, -1]
msg = Float32MultiArray(data=counter)
pub.publish(msg)
rospy.sleep(3.0)


clock = [1, -1, -1, 1]
msg = Float32MultiArray(data=clock)
pub.publish(msg)
rospy.sleep(3.0)

stop = [0, 0, 0, 0]
msg = Float32MultiArray(data=stop)
pub.publish(msg)
