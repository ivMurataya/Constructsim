
# ros2 pkg create --build-type ament_python movement_pkg --dependencies rclpy custom_interfaces std_msgs geometry_msgs sensor_msgs
# ros2 run movement_pkg movement_client "Turn Left"

from custom_interfaces.srv import MyCustomServiceMessage
import rclpy
from rclpy.node import Node
import sys


class ClientAsync(Node):

    def __init__(self):
        super().__init__('service_client')
        self.client = self.create_client(MyCustomServiceMessage, 'movement')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        #self.req = Empty.Request()
        self.req = MyCustomServiceMessage.Request()
        

    def send_request(self):
        self.req.move = sys.argv[1]
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
