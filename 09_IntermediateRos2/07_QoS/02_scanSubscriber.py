import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from rclpy.qos import (
    QoSProfile,
    QoSHistoryPolicy,
    QoSDurabilityPolicy,
    QoSLivelinessPolicy,
    QoSReliabilityPolicy
    
)

from rclpy.duration import Duration


class ScanSubscriber(Node):
    def __init__(self):
        super().__init__('subscriber_scan_qos')
        self.get_logger().info("Initializing subscriber_scan_qos...")

        # Custom QoS Configuration for CycloneDDS
        qos_profile = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
            lifespan=Duration(seconds = 5),
            deadline=Duration(),
            liveliness=QoSLivelinessPolicy.AUTOMATIC,
            liveliness_lease_duration=Duration()
        )

        # Subscriber with custom QoS profile
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            qos_profile
        )

    def scan_callback(self, msg):
        """
        Callback function for /scan topic
        """
        front_distance = msg.ranges[359]  # Front laser value
        self.get_logger().info(f"Front Distance: {front_distance:.2f} meters")


def main(args=None):
    rclpy.init(args=args)
    node = ScanSubscriber()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
