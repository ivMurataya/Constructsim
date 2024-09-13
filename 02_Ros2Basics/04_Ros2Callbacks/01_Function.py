#!/usr/bin/env python3
import rclpy
import time

def robot_message(text, robot_name="Robot-1"):
    print(robot_name+": "+text)


def main(args=None):
    rclpy.init(args=args)
    period = 1.0
    robot_message(text="Robot Booting Up...")
    time.sleep(period)
    robot_message(text="Robot Ready...")
    time.sleep(period)
    robot_message(text="Robot ShuttingDown...")
    rclpy.shutdown()

if __name__ == '__main__':
    main()
"""
 

    This was straightforward:
        You have a main function that performs the main task of your script.
        In this case, the main task is to give the robot's internal status messages.
        You have a function named robot_message that prints a message in the console when called.
    This script has only ONE task to perform:
        Giving the robot's internal status messages.


 """
