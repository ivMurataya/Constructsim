# Import the Trigger module from the std_srvs package, which defines a simple service interface
from std_srvs.srv import Trigger
# Import the ROS 2 Python client libraries
import rclpy
from rclpy.node import Node


# Define the RobotStatusClient class, which inherits from rclpy's Node base class
class RecognitionClient(Node):

    def __init__(self):
        # Initialize the node with the name 'robot_status_client'
        super().__init__('text_recognition_client_node')
        
        # Define the name and type of the service this client will communicate with
        # The service is named '/get_robot_status' and uses the Trigger service type
        name_service = '/text_recognition_service'
        self.client = self.create_client(Trigger, name_service)
        
        # Wait for the service server to be available
        # This loop retries every second until the service becomes available
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service '+name_service+' not available, waiting again...')

        # Create an empty Trigger request object
        # Since Trigger has no input fields, this is straightforward
        self.req = Trigger.Request()

    def send_request(self):
        # Send the Trigger request to the service server asynchronously
        # call_async() returns a Future object that will hold the result of the service call
        # The Future is a placeholder for the eventual response or exception from the service call.
        self.future = self.client.call_async(self.req)


# The main function to set up and run the client node
def main(args=None):
    # Initialize the ROS 2 communication system
    rclpy.init(args=args)
    
    # Create an instance of the RobotStatusClient node
    client = RecognitionClient()
    
    # Send a request to the service server
    client.send_request()

    # Keep checking for the service response
    while rclpy.ok():
        # Spin once to process callbacks and allow the client to check for the service response
        rclpy.spin_once(client)
        
        # Check if the service call is complete using the `done()` method of the Future object
        if client.future.done():
            try:
                # Attempt to retrieve the response from the Future
                # If the service call was successful, this will return the response object
                response = client.future.result()
            except Exception as e:
                # If an exception occurred during the service call, log the error
                # For example, if the service server fails or is unreachable
                client.get_logger().info(f'Service call failed: {e}')
            else:
                # Log the service response
                # The response contains two fields:
                # - `success` (boolean): Whether the service call was successful
                # - `message` (string): Additional information provided by the service
                client.get_logger().info(f'Success: {response.success}')
                client.get_logger().info(f'Status Report: {response.message}')
            break  # Exit the loop once the response is handled

    # Clean up the client node to release resources
    client.destroy_node()
    
    # Shut down the ROS 2 communication system
    rclpy.shutdown()


# Run the main function when the script is executed directly
if __name__ == '__main__':
    main()


"""
Key Methods of Future in ROS 2:
1. done()
    Checks whether the asynchronous service call has completed (i.e., the result is available).
    Returns True if:

        The service call succeeded, or
        The service call failed and an exception was raised.

    Returns False if the service call is still in progress.


2. result()
    Retrieves the result of the asynchronous operation.
    If the service call was successful:

        result() contains the response object from the service, which in this case includes success (bool) and message (string).

    If the service call failed:

        result() raises an exception, which must be handled using try-except.


3. exception()
    Checks if the asynchronous operation raised an exception.
    Returns:

        The exception object if one was raised.
        None if no exception occurred.
"""
