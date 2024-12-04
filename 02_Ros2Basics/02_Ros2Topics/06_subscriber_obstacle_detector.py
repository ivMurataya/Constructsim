"""
Need to provide suggested movements based on the detection.

"""

#!/usr/bin/env python

import rclpy
from rclpy.node import Node
# import the LaserScan module from sensor_msgs interface
from sensor_msgs.msg import LaserScan
# import Quality of Service library, to set the correct profile and reliability to read sensor data.
from rclpy.qos import ReliabilityPolicy, QoSProfile

class ObstacleDetectorNode(Node):
    def __init__(self, node_name="obstacle_detector_node"):
        self._node_name = node_name
        super().__init__(self._node_name)

        # create the subscriber object
        # in this case, the subscriptor will be subscribed on /scan topic with a queue size of 10 messages.
        # use the LaserScan module for /scan topic
        # send the received info to the laserscan_callback method.
        self.subscriber = self.create_subscription(
            LaserScan,
            '/laser_scan',
            self.laserscan_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))  # is the most used to read LaserScan data

        self.get_logger().info(self._node_name +" Ready...")

    def laserscan_callback(self, msg):
        # Find the minimum distance in the ranges array
        
        Lranges_dict = {
            "Right_Rear": (min(msg.ranges[0:33]),"Action Suggested = Go Forwards"),
            "Right": (min(msg.ranges[34:66]),"Action Suggested = Go Forwards turning slightly left"),
            "Front_Right": (min(msg.ranges[67:100]),"Action Suggested = Turn Left"),
            "Front_Left": (min(msg.ranges[101:133]),"Action Suggested = Turn Right"),
            "Left": (min(msg.ranges[134:166]),"Action Suggested = Go Forwards turning slightly Right"),
            "Left_Rear": (min(msg.ranges[167:199]),"Action Suggested = Go Forwards")
}

        # Log the minimum distance value from all values
        for variable, value in Lranges_dict.items(): 
            self.get_logger().info(f'{variable}: {value[0]:.2f}')

        # Find the variable (key) with the minimum value
        min_variable = min(Lranges_dict, key=lambda x: Lranges_dict[x][0])
        min_value = Lranges_dict[min_variable][0]
        action_suggested = Lranges_dict[min_variable][1]
        if min_value < 0.8:
            self.get_logger().info(f'Minimum Variable: {min_variable}, Value: {min_value:.2f}')
            self.get_logger().info(f'Action Suggested: {action_suggested}') 

        
def main(args=None):
    rclpy.init(args=args)
    node = ObstacleDetectorNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
