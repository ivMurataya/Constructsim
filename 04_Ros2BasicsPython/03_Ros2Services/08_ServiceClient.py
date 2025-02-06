
from services_quiz_srv.srv import Turn
import rclpy
from rclpy.node import Node
import sys


class ClientAsync(Node):

    def __init__(self):
        super().__init__('service_client')
        self.client = self.create_client(Turn, 'turn')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = Turn.Request()
        

    def send_request(self):
        self.req.direction = "right"
        self.req.angular_velocity = 0.2
        self.req.time = 10
        self.future = self.client.call_async(self.req)


def main(args=None):
    # initialize the ROS communication
    rclpy.init(args=args)
    # declare the node constructor
    client = ClientAsync()
    # run the send_request() method
    client.send_request()

    while rclpy.ok():
        rclpy.spin_once(client)
        if client.future.done():
            try:
                response = client.future.result()
            except Exception as e:
                client.get_logger().info(
                    'Service call failed %r' % (e,))
            else:
                client.get_logger().info(
                    'Response state %r' % (response.success))
            break

    client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
