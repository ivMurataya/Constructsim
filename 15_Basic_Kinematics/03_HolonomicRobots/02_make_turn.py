import rospy, math, numpy as np 
from std_msgs.msg import Float32MultiArray
from utilities import reset_world

# Initialize ROS node
rospy.init_node("olonomic_controller", anonymous=True)

# Create a publisher for the 'wheel_speed' topic
pub = rospy.Publisher('wheel_speed', Float32MultiArray, queue_size=10)

# Wait a moment to ensure publisher is ready
rospy.sleep(1.0)

"""
# (Optional test code - commented out)
# This would manually send wheel speeds and move the robot forward
forward = [10, 1, 1, 10]
msg = Float32MultiArray(data=forward)
pub.publish(msg)
rospy.sleep(3.0)
"""

# Define robot physical constants
R = 0.254 / 2   # Wheel radius (meters), based on 254mm diameter
L = 0.500 / 2   # Half of robot length (wheelbase)
W = 0.548 / 2   # Half of robot width (track width)

# Function to convert planar twist (vx, vy, wz) into individual wheel speeds
def twist2wheels(wz, vx, vy):
    # Kinematic transformation matrix (4x3)
    A = np.array([
        [ -L - W, 1, -1],  # Wheel 1
        [  L + W, 1,  1],  # Wheel 2
        [  L + W, 1, -1],  # Wheel 3
        [ -L - W, 1,  1]   # Wheel 4
    ])

    # Twist vector: [angular z, linear x, linear y]
    b = np.array([
        [wz],  # angular velocity around z
        [vx],  # linear velocity in x
        [vy]   # linear velocity in y
    ])

    # Compute wheel speeds: u = (1/R) * A * b
    result = (1/R) * np.dot(A, b)

    # Print wheel speeds to console
    print(result)

    return result

# Compute wheel speeds from desired twist (wz=1.5 rad/s, vx=1 m/s, vy=0)
u = twist2wheels(wz=-1.5, vx=2, vy=3)

# Publish computed wheel speeds to the topic
msg = Float32MultiArray(data=u)
pub.publish(msg)

# Let the robot run for a moment
rospy.sleep(1)

# Send stop command to halt the robot
stop = [0, 0, 0, 0]
msg = Float32MultiArray(data=stop)
pub.publish(msg)
