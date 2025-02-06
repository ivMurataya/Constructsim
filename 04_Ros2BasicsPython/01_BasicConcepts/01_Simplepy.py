import rclpy
# import the Node module from ROS2 Python library
from rclpy.node import Node

def main(args=None):
    # initialize the ROS communication
    rclpy.init(args=args)
    # print a message to the terminal
    print("Moe yo Byakugan! Kore ga watashi no nindō yo ")
    # english translation: "Blaze Away, Byakugan! This is My Ninja Way!"
    # shutdown the ROS communication
    rclpy.shutdown()

if __name__ == '__main__':
    main() #call the main function

# Let’s execute it in the simplest way possible: 
# python3 heartbeat.py 
