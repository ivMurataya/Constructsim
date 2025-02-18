import argparse
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from rclpy.qos_event import PublisherEventCallbacks
from rclpy.qos import QoSProfile
from rclpy.qos import QoSDurabilityPolicy

import time
from rclpy.duration import Duration


class PublisherQoS(Node):

    def __init__(self, qos_profile, node_name="publisher_qos_obj"):

        super().__init__(node_name)

        rclpy.logging.set_logger_level(
            node_name, rclpy.logging.LoggingSeverity.INFO)

        event_callbacks = PublisherEventCallbacks(
            incompatible_qos=self.incompatible_qos_clb,
            deadline=self.deadline_qos_clb)

        self.publisher_ = self.create_publisher(msg_type=String,
                                                topic='/qos_test',
                                                qos_profile=qos_profile,
                                                event_callbacks=event_callbacks)

        # Create a timer
        self.timer_period = 1.0
        self.swap_state_time = 5.0
        self.time_pause = 2.0
        self.counter = 0

        self.create_timer(self.timer_period, self.timer_callback)

    def incompatible_qos_clb(self, event):
        """
        This is the Callback that will be executed when the event of **Incompatible QoS** is
        triggered.
        """
        self.get_logger().error("PUBLISHER::: INCOMPATIBLE QoS Triggered!")
        self.get_logger().error(str(event.last_policy_kind))
        self.get_logger().error("############################")

    def deadline_qos_clb(self, event):
        """
        Triggered when the deadline is achieved
        """
        self.get_logger().error("PUBLISHER:::  Deadline Triggered!")

    def publish_one_message(self):
        # Here you have the callback method
        msg = String()
        test_time = self.get_clock().now()
        self.current_time_s, self.current_time_ns = test_time.seconds_nanoseconds()
        time_str = str(self.current_time_s)+","+str(self.current_time_ns)
        msg.data = time_str
        self.publisher_.publish(msg)
        self.get_logger().info('Published: "%s"' % msg)

    def timer_callback(self):
        # print a ROS2 log on the terminal with a great message!

        if self.counter > int(self.swap_state_time / self.timer_period):
            delta = 0.1
            range_steps = int(self.time_pause / delta)
            for i in range(range_steps):
                time.sleep(delta)
                self.get_logger().info("Paused ="+str(i*delta)+"/"+str(self.time_pause))
            self.counter = 0
        else:
            self.publish_one_message()
            self.counter += 1
            self.get_logger().info("Counter ="+str(self.counter))


def get_parser():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        '-deadline',
        type=float,
        help='Select Policy for deadline in seconds, use ros2 run qos_tests_pkg name_of_exe -deadline 1.350')
    return parser


def main(args=None):

    parser = get_parser()
    parsed_args = parser.parse_args()

    # Depth of History is Compulsory
    qos_profile_publisher = QoSProfile(depth=1)

    deadline_seconds = float(parsed_args.deadline)
    deadline = Duration(seconds=deadline_seconds)
    print("deadline=="+str(deadline))
    qos_profile_publisher.deadline = deadline

    rclpy.init(args=args)
    pub_qos_obj = PublisherQoS(qos_profile_publisher)
    pub_qos_obj.publish_one_message()
    rclpy.spin(pub_qos_obj)
    pub_qos_obj.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
# ros2 run qos_tests_pkg publisher_deadline_exe -deadline 1.1
"""
[INFO] [1739893722.209079737] [publisher_qos_obj]: Published: "std_msgs.msg.String(data='1739893722,208353537')"
[INFO] [1739893722.209557093] [publisher_qos_obj]: Counter =6
[INFO] [1739893723.309450574] [publisher_qos_obj]: Paused =0.0/2.0
[INFO] [1739893723.410443497] [publisher_qos_obj]: Paused =0.1/2.0
[INFO] [1739893723.511437610] [publisher_qos_obj]: Paused =0.2/2.0
[INFO] [1739893723.612141120] [publisher_qos_obj]: Paused =0.30000000000000004/2.0
[INFO] [1739893723.713171052] [publisher_qos_obj]: Paused =0.4/2.0
[INFO] [1739893723.814140313] [publisher_qos_obj]: Paused =0.5/2.0
[INFO] [1739893723.915276928] [publisher_qos_obj]: Paused =0.6000000000000001/2.0
[INFO] [1739893724.015950250] [publisher_qos_obj]: Paused =0.7000000000000001/2.0
[INFO] [1739893724.119139315] [publisher_qos_obj]: Paused =0.8/2.0
[INFO] [1739893724.219951684] [publisher_qos_obj]: Paused =0.9/2.0
[INFO] [1739893724.320899970] [publisher_qos_obj]: Paused =1.0/2.0
[INFO] [1739893724.422542506] [publisher_qos_obj]: Paused =1.1/2.0
[INFO] [1739893724.523574600] [publisher_qos_obj]: Paused =1.2000000000000002/2.0
[INFO] [1739893724.625618731] [publisher_qos_obj]: Paused =1.3/2.0
[INFO] [1739893724.726297908] [publisher_qos_obj]: Paused =1.4000000000000001/2.0
[INFO] [1739893724.827897825] [publisher_qos_obj]: Paused =1.5/2.0
[INFO] [1739893724.928647932] [publisher_qos_obj]: Paused =1.6/2.0
[INFO] [1739893725.030216944] [publisher_qos_obj]: Paused =1.7000000000000002/2.0
[INFO] [1739893725.131311789] [publisher_qos_obj]: Paused =1.8/2.0
[INFO] [1739893725.232565851] [publisher_qos_obj]: Paused =1.9000000000000001/2.0
[ERROR] [1739893725.233322030] [publisher_qos_obj]: PUBLISHER:::  Deadline Triggered!
[INFO] [1739893725.233959840] [publisher_qos_obj]: Published: "std_msgs.msg.String(data='1739893725,233471486')"
[INFO] [1739893725.234578717] [publisher_qos_obj]: Counter =1
"""
