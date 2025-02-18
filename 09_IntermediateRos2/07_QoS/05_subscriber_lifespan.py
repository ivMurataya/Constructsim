import argparse
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from rclpy.qos_event import SubscriptionEventCallbacks
from rclpy.qos import QoSProfile
from rclpy.qos import QoSDurabilityPolicy
from rclpy.qos import QoSReliabilityPolicy

from rclpy.duration import Duration


class SubscriberQoS(Node):

    def __init__(self, qos_profile, node_name="subscriber_qos_obj"):

        super().__init__(node_name)

        rclpy.logging.set_logger_level(
            node_name,
            rclpy.logging.LoggingSeverity.INFO)

        event_callbacks = SubscriptionEventCallbacks(
            incompatible_qos=self.incompatible_qos_clb)

        # create the subscriber object
        self.subscriber = self.create_subscription(
            msg_type=String,
            topic='/qos_test',
            callback=self.listener_callback,
            qos_profile=qos_profile,
            event_callbacks=event_callbacks)

    def listener_callback(self, msg):
        """
        Parse it and calculate the time passed
        1644948035,948696546
        """
        raw_data = msg.data
        self.get_logger().info("Data Received ="+str(raw_data))
        split_data = raw_data.split(",")
        self.get_logger().info("SPLIT ="+str(split_data))

        seconds = float(split_data[0])
        nseconds = float(split_data[1])

        self.get_logger().info("seconds ="+str(seconds)+", nseconds = "+str(nseconds))

        total_seconds = seconds + nseconds * (10 ** -9)

        self.get_logger().info("total_seconds ="+str(total_seconds))

        # Get time now
        test_time = self.get_clock().now()
        current_time_s, current_time_ns = test_time.seconds_nanoseconds()

        total_current_time = float(current_time_s) + \
            (float(current_time_ns) * (10 ** -9))

        self.get_logger().info("total_current_time ="+str(total_current_time))

        delta = total_current_time - total_seconds

        self.get_logger().info("Message Age ="+str(delta))

    def incompatible_qos_clb(self, event):
        """
        This is the Callback that will be executed when the event of **Incompatible QoS** is
        triggered.
        """
        self.get_logger().error("SUBSCRIBER::: INCOMPATIBLE QoS Triggered!")
        self.get_logger().error(str(event.last_policy_kind))
        self.get_logger().error("############################")


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
    qos_profile_subscriber = QoSProfile(depth=1)

    lifespan_seconds = float(parsed_args.lifespan)
    lifespan = Duration(seconds=lifespan_seconds)
    print("lifespan=="+str(lifespan))
    qos_profile_subscriber.lifespan = lifespan

    # Reliability set to reliable
    qos_profile_subscriber.reliability = QoSReliabilityPolicy.RELIABLE
    # Durability Set to Transient Local
    qos_profile_subscriber.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL

    rclpy.init(args=args)
    sub_qos_obj = SubscriberQoS(qos_profile_subscriber)
    rclpy.spin(sub_qos_obj)
    sub_qos_obj.destroy_node()
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
