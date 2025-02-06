


#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from custom_interfaces.msg  import Age
from datetime import datetime

class CreationDatePublisher(Node):

    def __init__(self):
        super().__init__('creation_date_publisher')

        # Create a publisher on the 'creation_date' topic
        self.publisher_ = self.create_publisher(Age, 'creation_date', 10)

        # Get the creation date of the script
        self.creation_time = datetime.now()
        self.timer_period = 0.5
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

    def timer_callback(self):
        # Initialize the message
        msg = Age()
        msg.year = self.creation_time.year
        msg.month = self.creation_time.month
        msg.day = self.creation_time.day

        # Publish the message
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: {msg.year}-{msg.month:02}-{msg.day:02}')

def main(args=None):
    rclpy.init(args=args)

    creation_date_publisher = CreationDatePublisher()

    # Spin the node to keep the program alive
    rclpy.spin(creation_date_publisher)

    # Shutdown ROS 2
    creation_date_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()




# Create Pkg with custom_interfaces depencencies
# ros2 pkg create --build-type ament_python example36_pkg --dependencies rclpy std_msgs geometry_msgs custom_interfaces

