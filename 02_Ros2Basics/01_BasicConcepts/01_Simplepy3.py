#!/usr/bin/env python
# Indicate which interpreter should be used to execute the script. 
import rclpy 
import time 

# We define a method named main, which is standard in Python.
def main (args=None):
    # initialize the ROS communication
    rclpy.init(args=args)
    i = 0 
    max_i = 50
    while i < max_i:
        i += 1 
        time_stamp = time.time()
        # print a message to the terminal
        print(str(i) + ": Mars rover 1 is alive... " + str(time_stamp))
        time.sleep(1)
    #shutdown the ROS comunication
    rclpy.shutdown()

if __name__ == '__main__':
    main() #call the main function


# Publish a message in the console without creating a ROS2 Node
