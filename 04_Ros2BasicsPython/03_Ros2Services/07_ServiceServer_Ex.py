""" Inerface created 
string direction               # Direction to spin (right or left)
float64 angular_velocity       # Angular Velocity (in rad/s)
int32 time                     # Duration of the spin (in seconds)
---
bool success                   # Did it achieve it?

"""

from services_quiz_srv.srv import Turn
from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node

class Service(Node):

    def __init__(self):
        super().__init__('movement_server')
        self.srv = self.create_service(Turn, 'turn', self.funcion_callback)
        self.publisher_ = self.create_publisher(Twist, 'cmd_vel', 10)
        self.timer = None  # Timer will be created when needed
        self.is_moving = False  # Track whether the robot is currently moving
        self.doneMoving = False

    def funcion_callback(self, request, response):
        # Check if the robot is already moving and prevent another movement
        if self.is_moving:
            self.get_logger().warn('Robot is already moving, ignoring new request.')
            response.success = False
            return response

        # Start moving the robot
        self.is_moving = True  # Indicate the robot is now moving
        msg = Twist()
        msg.angular.z = request.angular_velocity  # Set the angular velocity from request
        duration = request.time  # Duration for how long the robot should move

        # Determine the direction of rotation
        if request.direction == "left":
            msg.angular.z = abs(msg.angular.z)  # Turn left
        elif request.direction == "right":
            msg.angular.z = -abs(msg.angular.z)  # Turn right
        else:
            response.success = False  # Invalid direction
            self.is_moving = False  # Reset the movement flag
            return response

        # Publish the movement command
        self.publisher_.publish(msg)
        self.get_logger().info(f'Moving Robot {request.direction} for {duration} seconds')

        # Create a timer to stop the robot after the specified duration
        if duration > 0:
            self.timer = self.create_timer(duration, self.stop_robot)
        else:
            self.get_logger().warn('Duration must be greater than 0 to move the robot.')
            self.is_moving = False  # Reset the movement flag
            response.success = False
            return response

        response.success = True
        #if self.doneMoving:
        return response

    def stop_robot(self):
        """Function to stop the robot by publishing zero velocities."""
        stop_msg = Twist()
        stop_msg.linear.x = 0.0
        stop_msg.angular.z = 0.0
        self.publisher_.publish(stop_msg)
        self.get_logger().info('Stopped robot after movement')

        # Cancel the timer if it exists
        if self.timer:
            self.timer.cancel()
            self.timer = None  # Set the timer to None to indicate it's no longer in use

        self.is_moving = False  # Reset the movement flag
        self.doneMoving = True

def main(args=None):
    rclpy.init(args=args)

    # Create the node
    moving_service = Service()

    # Keep the node spinning to handle requests and timers
    try:
        rclpy.spin(moving_service)
    except KeyboardInterrupt:
        pass  # Allow graceful shutdown

    rclpy.shutdown()

if __name__ == '__main__':
    main()
