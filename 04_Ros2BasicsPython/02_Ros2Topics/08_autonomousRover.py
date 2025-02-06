
"""
    Based on what we developed previously with the subscriber_obstacle_detector.py and the publish_mars_rover_move.py, we now need to create a script named autonomous_exploration.py that allows our Mars rover to explore its surroundings avoiding obstacles.
    Apply what we have seen on the Subscriber and Publisher combination to move around randomly, and when the laser detects an obstacle, move accordingly to avoid it.
    There are many strategies to do this, but the one we recommend is:
        When you detect something in front, turn until it is no longer detected in front.
        If detected on the sides, turn accordingly.
        If no detection or detection on the rear sections of the laser, move forward.

    For that, we need the Mars rover to detect when it's too far away from the center of its exploration zone.
    Normally, we define a circle of a certain radius ( 5.0 meters ), where we are going to look for life and take samples.
    The Mars rover should detect the obstacles but also detect when it's exiting the 5-meter radius exploration circle and turn around.
"""

#!/usr/bin/env python

import rclpy
from rclpy.node import Node
# import the LaserScan module from sensor_msgs interface
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
# import Quality of Service library, to set the correct profile and reliability to read sensor data.
from rclpy.qos import ReliabilityPolicy, QoSProfile
import signal
import math
import tf_transformations



class ObstacleDetectorNode(Node):
    def __init__(self, node_name="obstacle_detector_node"):
        self._node_name = node_name
        self.distance_from_origin = 0
        self.current_position =  {'x': 0.0, 'y': 0.0}

        super().__init__(self._node_name)

        self.subscriber = self.create_subscription(
            LaserScan,
            '/laser_scan',
            self.laserscan_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))  
        
        self.subscriberOdom = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))  

        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)

        self.get_logger().info(self._node_name +" Ready...")

    def laserscan_callback(self, msg):
        self.Lranges_dict = {
            "Right_Rear": (min(msg.ranges[0:33]),"Action Suggested = Go Forwards", 0.5, 0.0),
            "Right": (min(msg.ranges[34:66]),"Action Suggested = Go Forwards turning slightly left", 0.5, 1.0),
            "Front_Right": (min(msg.ranges[67:100]),"Action Suggested = Turn Left", 0.0, 1.0),
            "Front_Left": (min(msg.ranges[101:133]),"Action Suggested = Turn Right", 0.0, -1.0),
            "Left": (min(msg.ranges[134:166]),"Action Suggested = Go Forwards turning slightly Right", 0.5, -1.0),
            "Left_Rear": (min(msg.ranges[167:199]),"Action Suggested = Go Forwards", 0.5, 0.0 )}

        # Log the minimum distance value from all values
        for variable, value in self.Lranges_dict.items(): 
            self.get_logger().info(f'{variable}: {value[0]:.2f}')

        # Find the variable (key) with the minimum value
        min_variable = min(self.Lranges_dict, key=lambda x: self.Lranges_dict[x][0])
        min_value = self.Lranges_dict[min_variable][0]
        action_suggested = self.Lranges_dict[min_variable][1]

        self.movement = Twist()
        if self.distance_from_origin < 6.0:
            if min_value > 0.8 or min_value == "inf":
                self.movement.linear.x = 0.5
                self.movement.angular.z = 0.0
            elif min_value < 0.8:
                self.movement.linear.x = self.Lranges_dict[str(min_variable)][2]
                self.movement.angular.z = self.Lranges_dict[str(min_variable)][3]
                self.get_logger().info(f'Minimum Variable: {min_variable}, Value: {min_value:.2f}')
                self.get_logger().warn(f'Action Suggested: {action_suggested}') 
            self.publisher_.publish(self.movement)
        else:
            #self.stop_rover()
            self.return_to_origin()

        

    def stop_rover(self):
        stopMsg = Twist()
        self.publisher_.publish(stopMsg)
        self.get_logger().info('Stoping the Robot')

    def odom_callback(self,msg):
        self.current_position['x'] = msg.pose.pose.position.x
        self.current_position['y'] = msg.pose.pose.position.y

        # Calculate the distance from the origin (0,0)
        self.distance_from_origin = math.sqrt(self.current_position['x']**2 + self.current_position['y']**2)
        self.get_logger().info(f'Distance from origin: {self.distance_from_origin:.2f} meters')

        # Calculate the yaw (orientation around the z-axis)
        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, self.yaw) = tf_transformations.euler_from_quaternion(orientation_list)


    #The return_to_origin is the method that you have to use to trigger the action of returning to the center until the distance to the center is lower than the radius of the exp`loration area.
    def return_to_origin(self):
        action = Twist()

        # Calculate the desired angle to the origin
        desired_yaw = math.atan2(-self.current_position['y'], -self.current_position['x'])

        # Calculate the difference between current yaw and desired yaw
        yaw_error = desired_yaw - self.yaw

        # Normalize the yaw error to the range [-pi, pi]
        yaw_error = (yaw_error + math.pi) % (2 * math.pi) - math.pi

        # If the yaw error is significant, rotate towards the origin
        if abs(yaw_error) > 0.1:  # 0.1 radians threshold for orientation
            action.angular.z = 0.5 if yaw_error > 0 else -0.5
            self.get_logger().info(f'Turning towards origin. Yaw error: {yaw_error:.2f}')
        else:
            # If oriented towards the origin, move forward
            action.linear.x = 0.5
            self.get_logger().info('Heading towards origin.')

        self.publisher_.publish(action)
        
def main(args=None):
    rclpy.init(args=args)
    nodew = ObstacleDetectorNode()

   # pause the program execution, waits for a request to kill the node (ctrl+c)
    def signal_handler(sig, frame):
        nodew.stop_rover()
        nodew.destroy_node()
        rclpy.shutdown()

    signal.signal(signal.SIGINT, signal_handler)
    

    rclpy.spin(nodew)
    

if __name__ == '__main__':
    main()
