# import the Empty module from std_servs Service interface
from std_srvs.srv import Empty
from std_srvs.srv import SetBool
# import the Twist module from geometry_msgs messages interface
from geometry_msgs.msg import Twist
# import the ROS2 Python client libraries
import rclpy
from rclpy.node import Node


class Service(Node):

    def __init__(self):
        # Here you have the class constructor

        # call the class constructor to initialize the node as service_moving
        super().__init__('service_move')
        # create the Service Server object
        # defines the type, name, and callback function
        self.srv = self.create_service(SetBool, 'starstop', self.funcion_callback)
        # create the Publisher object
        # in this case, the Publisher will publish on /cmd_vel topic with a queue size of 10 messages.
        # use the Twist module
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        

    def funcion_callback(self, request, response):
        # The callback function receives the self-class parameter, 
        # received along with two parameters called request and response
        # - receive the data by request
        # - return a result as a response
        

        # create a Twist message
        msg = Twist()
        if request.data == True:
            # define the linear x-axis velocity of /cmd_vel topic parameter to 0.3
            msg.linear.x = 0.3
            # define the angular z-axis velocity of /cmd_vel topic parameter to 0.3
            msg.angular.z = 0.3
            response.success = True
            response.message = "Robot Rotating"
        if request.data == False:
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            response.success = True
            response.message = "Robot Stoped"
        # Publish the message to the Topic
        self.publisher_.publish(msg)
        # print a pretty message
        self.get_logger().info('RUN ROBOT RUN!')
        
        
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


"""


Based on the previous examples, make a Service that executes the following instructions:

    Create a new package and call it exercise42_pkg with rclpy, std_msgs, sensor_msgs, geometry_msgs, and std_srvs as dependencies.

    Use the previous Service Server codes to create a new Service Server that makes the robot turn right (as in the example of the /moving Service, but the opposite direction). But this time will be a little bit more difficult. You will use the Service SetBool, part of the std_srv package. It will work like this:
        When the request is true, the robot turns right.
        When the input is false, the robot stops!

    Create the launch file to start your node.
    Test your Service by calling it using the ros2 service call command.


"""
