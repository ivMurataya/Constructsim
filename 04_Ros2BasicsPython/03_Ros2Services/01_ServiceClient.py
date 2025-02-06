# import the empty module from std_servs Service interface
from std_srvs.srv import Empty
# import the ROS2 Python client libraries
import rclpy
from rclpy.node import Node


class ClientAsync(Node):

    def __init__(self):
        # Here you have the class constructor

        # call the class constructor to initialize the node as service_client
        super().__init__('service_client')
        # create the Service Client object
        # defines the name and type of the Service Server you will work with.
        """This is the line where you create the Client, that uses the Empty Service type and connects to a Service named /moving. """
        self.client = self.create_client(Empty, 'moving')
        # checks once per second if a Service matching the type and name of the Client is available.
        """This while loop is used to ensure that the Service Server (in this case, /moving) is up and running"""
        while not self.client.wait_for_service(timeout_sec=1.0):
            # if it is not available, a message is displayed
            self.get_logger().info('service not available, waiting again...')
        
        # create an Empty request
        self.req = Empty.Request()
        

    def send_request(self):
        
        # send the request
        """Send an asynchronous request to the Service Server using the call_async() method. 
        Then, store the response from the Server in the variable self.future. 
        After making the request, the Server will immediately return future, 
        which indicates whether the call and response are finished (but it does not 
        contain the value of the response itself)"""
        self.future = self.client.call_async(self.req)


def main(args=None):
    # initialize the ROS communication
    rclpy.init(args=args)
    # declare the node constructor
    client = ClientAsync()
    # run the send_request() method
    client.send_request()

    while rclpy.ok():
        # pause the program execution, waits for a request to kill the node (ctrl+c)

        """This will spin the client node one time. It is similar to rclpy.spin (that you already saw ), but instead of spinning the node indefinitely, it will only spin it once."""
        rclpy.spin_once(client)
        #you keep checking (while the program is running, rclpy.ok()) if the Service response is finished:
        if client.future.done():
            try:
                # checks the future for a response from the Service
                # while the system is running. 
                # If the Service has sent a response, the result will be written
                # to a log message.
                # If it is finished, you then get the value of the Server response
                response = client.future.result()
            except Exception as e:
                # Display the message on the console
                client.get_logger().info(
                    'Service call failed %r' % (e,))
            else:
                # Display the message on the console
                # While you wait for the Service response to finish, you are printing messages to the node's log:
                client.get_logger().info(
                    'the robot is moving' ) 
            break

    client.destroy_node()
    # shutdown the ROS communication
    rclpy.shutdown()


if __name__ == '__main__':
    main()
