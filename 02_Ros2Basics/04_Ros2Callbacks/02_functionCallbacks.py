#!/usr/bin/env python3
import rclpy
import time
from rclpy.node import Node

class RobotStatus(Node):
    def __init__(self):
        super().__init__('robot_status')
        self.time_robot_on = 0.0
        self.timer_period = 0.1
        self.sleep_timer_counter = 0.0
        self.robot_status = ["Robot Booting Up...",
                             "Robot Ready...",
                             "Robot ShuttingDown..."]
        self.main_task_period = 1.0
        self.sleep_time_main_task = 0.0
    
        self.create_timer(self.timer_period, self.timer_counter)
        self.create_timer(self.main_task_period, self.main_task)
        
    def robot_message(self, text, robot_name="Robot-1"):
        self.get_logger().info(robot_name+": "+text)

    def timer_counter(self):
        self.time_robot_on += self.timer_period
        self.get_logger().info("Updated Time Robot On="+str(self.time_robot_on))
        time.sleep(self.sleep_timer_counter)
        self.get_logger().info("Updated Time Robot On="+str(self.time_robot_on))

    def main_task(self):

        if len(self.robot_status)>=1:
            status = self.robot_status.pop(0)
            self.robot_message(text=status)
        else:
            status = "ShuttingDown"

        

        if "ShuttingDown" in status:
            self.get_logger().info('Shutting down node...')
            self.destroy_node()
            rclpy.shutdown()
        else:
            self.get_logger().info('Continue....')
            time.sleep(self.sleep_time_main_task)

def main(args=None):
    rclpy.init(args=args)
    robot_status_node = RobotStatus()
    rclpy.spin(robot_status_node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

"""


    Here, you made two functions, main_task and timer_counter, into CALLBACKS.

    These CALLBACKS will be TRIGGERED by two different timers:
        self.timer_period: The timer_counter will be called at that period ( 0.1 seconds )
        self.main_task_period: The main_task will be called at that period ( 1.0 seconds )

    You must differentiate between CALLBACK PERIODS, the self.timer_period and self.main_task_period, the period in which you call the callback functions.

    The FUNCTION PROCESSING TIME is the time each callback takes to execute and finish.

    Each function, timer_counter and main_task, has a sleep inside them simulating time to process that function, giving them different FUNCTION PROCESSING TIMES.

    You add this because normally, functions, especially in robotics perception or similar, can take time to finish.
    For now, set them to be instantaneous:
        self.sleep_timer_counter = 0.0
        self.sleep_time_main_task = 0.0


"""
