import argparse
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from rclpy.qos_event import PublisherEventCallbacks
from rclpy.qos import QoSProfile
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSReliabilityPolicy

from rclpy.duration import Duration


class PublisherQoS(Node):

    def __init__(self, qos_profile, node_name="publisher_qos_obj"):

        super().__init__(node_name)

        rclpy.logging.set_logger_level(
            node_name, rclpy.logging.LoggingSeverity.INFO)

        event_callbacks = PublisherEventCallbacks(
            incompatible_qos=self.incompatible_qos_clb)

        self.publisher_ = self.create_publisher(msg_type=String,
                                                topic='/qos_test',
                                                qos_profile=qos_profile,
                                                event_callbacks=event_callbacks)

    def incompatible_qos_clb(self, event):
        """
        This is the Callback that will be executed when the event of **Incompatible QoS** is
        triggered.
        """
        self.get_logger().error("PUBLISHER::: INCOMPATIBLE QoS Triggered!")
        self.get_logger().error(str(event.last_policy_kind))
        self.get_logger().error("############################")

    def publish_one_message(self):
        # Here you have the Callback method
        msg = String()
        test_time = self.get_clock().now()
        self.current_time_s, self.current_time_ns = test_time.seconds_nanoseconds()
        time_str = str(self.current_time_s)+","+str(self.current_time_ns)
        msg.data = time_str
        self.publisher_.publish(msg)
        self.get_logger().info('Published: "%s"' % msg)


def get_parser():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        '-lifespan',
        type=float,
        help='Select Policy for lifespan, use ros2 run qos_tests_pkg name_of_exe -lifespan 3.2')
    return parser


def main(args=None):

    parser = get_parser()
    parsed_args = parser.parse_args()

    # Depth of History is Compulsory
    qos_profile_publisher = QoSProfile(depth=1)

    lifespan_seconds = float(parsed_args.lifespan)
    lifespan = Duration(seconds=lifespan_seconds)
    print("lifespan=="+str(lifespan))
    qos_profile_publisher.lifespan = lifespan

    # Reliability set to reliable
    qos_profile_publisher.reliability = QoSReliabilityPolicy.RELIABLE
    # Durability Set to Transient Local
    qos_profile_publisher.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL

    rclpy.init(args=args)
    pub_qos_obj = PublisherQoS(qos_profile_publisher)
    pub_qos_obj.publish_one_message()
    rclpy.spin(pub_qos_obj)
    pub_qos_obj.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
"""
#Publish the message with a lifespan of 30 seconds
ros2 run qos_tests_pkg publisher_lifespan_exe -lifespan 30.0
lifespan==30000000000 nanoseconds
[INFO] [1739898480.492928784] [publisher_qos_obj]: Published: "std_msgs.msg.String(data='1739898480,472083251')"

#Start the subscriber with lifespan of 5 seconds at sec 6, the message will not be valid
ros2 run qos_tests_pkg subscriber_lifespan_exe -lifespan 5.0
lifespan==5000000000 nanoseconds

#Start subscriber at sec 7, the message will be valid
ros2 run qos_tests_pkg subscriber_lifespan_exe -lifespan 10.0
lifespan==10000000000 nanoseconds
[INFO] [1739898490.094275693] [subscriber_qos_obj]: Data Received =1739898480,472083251
[INFO] [1739898490.095006542] [subscriber_qos_obj]: SPLIT =['1739898480', '472083251']
[INFO] [1739898490.095647006] [subscriber_qos_obj]: seconds =1739898480.0, nseconds = 472083251.0
[INFO] [1739898490.096257049] [subscriber_qos_obj]: total_seconds =1739898480.4720833
[INFO] [1739898490.096918116] [subscriber_qos_obj]: total_current_time =1739898490.096315
[INFO] [1739898490.097547359] [subscriber_qos_obj]: Message Age =9.624231576919556
"""
