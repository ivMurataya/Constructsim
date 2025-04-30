import rospy
from std_msgs.msg import Float32MultiArray

rospy.init_node("olonomic_controller", anonymous=True)
pub = rospy.Publisher('wheel_speed', Float32MultiArray, queue_size=10)
rospy.sleep(1.0)

# Define movement commands with names (for clarity)
movements = [
    [1, 1, 1, 1],     # forward
    [-1, -1, -1, -1], # back
    [-1, 1, -1, 1],   # left
    [1, -1, 1, -1],   # right
    [-1, 1, 1, -1],   # counter-clockwise rotation
    [1, -1, -1, 1]    # clockwise rotation
]

# Publish each movement with 3 seconds delay
for move in movements:
    pub.publish(Float32MultiArray(data=move))
    rospy.sleep(3.0)

# Stop
pub.publish(Float32MultiArray(data=[0, 0, 0, 0]))
