# initialize the ROS communication
rclpy.init(args=args)
# declare the node constructor
client = ClientAsync()
# run the send_request() method
client.send_request()
while rclpy.ok():
        # pause the program execution, waits for a request to kill the node (ctrl+c)
        rclpy.spin_once(client)
        if client.future.done():
            try:
                # checks the future for a response from the Service
                # while the system is running. 
                # If the Service has sent a response, the result will be written
                # to a log message.
                response = client.future.result()
            except Exception as e:
                # Display the message on the console
                client.get_logger().info(
                    'Service call failed %r' % (e,))
            else:
                # Display the message on the console
                client.get_logger().info(
                    'Pretty message' ) 
            break
            
client.destroy_node()
# shutdown the ROS communication
rclpy.shutdown()



"""
"""
# initialize the ROS communication
rclpy.init(args=args)
# declare the node constructor
client = ClientSync()
# start the communication thread
spin_thread = Thread(target=rclpy.spin, args=(client,))
spin_thread.start()
# run the send_request() method
response = client.send_request()
# Display the message on the console
client.get_logger().info('Pretty message') 

minimal_client.destroy_node()
# shutdown the ROS communication
rclpy.shutdown()

