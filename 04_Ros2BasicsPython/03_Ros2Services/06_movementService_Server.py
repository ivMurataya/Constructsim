# ros2 pkg create --build-type ament_python movement_pkg --dependencies rclpy custom_interfaces std_msgs geometry_msgs sensor_msgs

from custom_interfaces.srv import MyCustomServiceMessage
from geometry_msgs.msg import Twist
# import the ROS2 Python client libraries
import rclpy
from rclpy.node import Node


class Service(Node):

    def __init__(self):
        super().__init__('movement_server')
        self.srv = self.create_service(MyCustomServiceMessage, 'movement', self.funcion_callback)
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        

    def funcion_callback(self, request, response):

        msg = Twist()
        if request.move == "Left":
            msg.linear.x = 0.3
            msg.angular.z = 0.5
            response.success = True
            
        elif request.move == "Right":
            msg.linear.x = 0.3
            msg.angular.z = -0.5
            response.success = True
            
        elif request.move == "Stop":
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            response.success = True
        else:
            response.success = False
            
        # Publish the message to the Topic
        self.publisher_.publish(msg)
        # print a pretty message
        self.get_logger().info('Action Done')
        
        
        # return the response parameter
        return response


def main(args=None):
    # initialize the ROS communication
    rclpy.init(args=args)
    # declare the node constructor
    moving_service = Service()
    # pause the program execution, waits for a request to kill the node (ctrl+c)
    rclpy.spin(moving_service)
    # shutdown the ROS communication
    rclpy.shutdown()


if __name__ == '__main__':
    main()


