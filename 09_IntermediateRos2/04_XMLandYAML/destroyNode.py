import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        self.timer = self.create_timer(0.5, self.timer_callback)
        self.get_logger().info('Node has been started.')

    def timer_callback(self):
        msg = String()
        msg.data = 'Hello, ROS 2!'
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)

    def __del__(self):
        self.get_logger().info('Node is being destroyed.')
        # Perform cleanup here
        self.destroy_timer(self.timer)
        self.destroy_publisher(self.publisher_)
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = MinimalPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        node.get_logger().info('Shutting down node...')
        node.destroy_node()

if __name__ == '__main__':
    main()
