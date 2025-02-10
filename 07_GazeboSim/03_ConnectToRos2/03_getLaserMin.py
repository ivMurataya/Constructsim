import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

class DisplayOdometry(Node):

    def __init__(self):
        # initialize node
        super().__init__("display_odom_node")

        self.cmd_vel_pub = self.create_publisher(msg_type=Twist, topic="/cmd_vel",qos_profile=1)        
        self.twist_msg = Twist()

        # initialize odom subscriber
        self.odom_sub_qos = QoSProfile(depth=10,
                                       reliability=ReliabilityPolicy.RELIABLE,
                                       durability=DurabilityPolicy.VOLATILE)

        self.laser_sub = self.create_subscription(msg_type=LaserScan,
                                                 topic="/laser/scan",
                                                 callback=self.laser_callback,
                                                 qos_profile=self.odom_sub_qos)
        
        self.twist_msg.linear.x = 0.50
        self.twist_msg.angular.z = 0.75
        self.cmd_vel_pub.publish(self.twist_msg)

        return None

    def laser_callback(self, laser_msg):
        min_range = min(laser_msg.ranges)  # Get the minimum range
        self.get_logger().info(f'Minimum Range: {min_range:.3f} meters')
        return None

def main(args=None):
    # initialize ROS2 communication
    rclpy.init(args=args)
    # initialize node
    display_odom_node = DisplayOdometry()
    # spin the node
    rclpy.spin(display_odom_node)
    # destroy the node
    display_odom_node.destroy_node()
    # shutdown ROS2 communication
    rclpy.shutdown()

    return None

if __name__ == "__main__":
    main()

# End of Code
