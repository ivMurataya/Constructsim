import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math
import sys

class RotateRobot(Node):
    def __init__(self):
        super().__init__('rotate_robot')
        self.odom_subscriber = self.create_subscription(
            Odometry,
            '/odom',  # Odometry data topic
            self.odom_callback,
            10
        )


        self.cmd_vel_publisher = self.create_publisher(
            Twist,
            '/cmd_vel',  # Velocity command topic
            10
        )

        self.current_angle = None
        self.target_angle = 0.0
        self.odom_received = False
        self.get_logger().info("Odom rotation node initialized. Listening to /odom...")


    def odom_callback(self, msg):
        # Extract the current angle from the odometry data (quaternion to Euler conversion)
        orientation_q = msg.pose.pose.orientation
        yaw = self.quaternion_to_euler(orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w)
        self.current_angle = yaw  # Yaw angle (z-axis)
        self.odom_received = True

    def quaternion_to_euler(self, qx, qy, qz, qw):
        """
        Converts quaternion to Euler angles (roll, pitch, yaw)
        """
         # Yaw (rotation around Z-axis)
        siny_cosp = 2.0 * (qw * qz + qx * qy)
        cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return yaw


    def wait_for_odom(self):
        """
        Waits until odometry data is received.
        """
        self.get_logger().info('Waiting for /odom data...')
        while not self.odom_received:
            rclpy.spin_once(self)
        self.get_logger().info('Odometry data received!')


    def rotate_relative(self, relative_angle_degrees):
        # Wait for odometry before starting the rotation
        self.wait_for_odom()
        # Calculate the target angle relative to the current angle
        relative_angle_radians = math.radians(relative_angle_degrees)
        target_angle = self.current_angle + relative_angle_radians
        
        # Normalize the target angle to be within -pi to pi
        target_angle = (target_angle + math.pi) % (2 * math.pi) - math.pi
        self.target_angle = target_angle

        # Start rotating
        twist_msg = Twist()
        while abs(self.target_angle - self.current_angle) > 0.02:
            # Determine the error in angle
            angle_diff = self.target_angle - self.current_angle
            # Normalize the angle difference to be within -pi to pi
            angle_diff = (angle_diff + math.pi) % (2 * math.pi) - math.pi

            # Set angular velocity (simple proportional control)
            twist_msg.angular.z = 0.5 * angle_diff  # Proportional gain of 0.5
            self.cmd_vel_publisher.publish(twist_msg)
            self.get_logger().info(f'Rotating: Current angle = {self.current_angle}, Target angle = {self.target_angle}')
            rclpy.spin_once(self)  # Keep the program responsive

        # Stop once the desired angle is reached
        twist_msg.angular.z = 0.0
        self.cmd_vel_publisher.publish(twist_msg) 
        self.get_logger().info(f'Rotation completed. Reached target angle {self.target_angle}')


def main(args=None):
    rclpy.init(args=args)

    # Parse the command-line argument for the relative angle
    if len(sys.argv) < 2:
        print("Usage: ros2 run rotate_robot rotate_robot <relative_angle_degrees>")
        return
    
    try:
        relative_angle = float(sys.argv[1])  # Convert the argument to a float
    except ValueError:
        print("Invalid angle. Please provide a numeric value.")
        return

    node = RotateRobot()

    # Rotate by the specified angle relative to the current orientation
    node.rotate_relative(relative_angle)

    rclpy.shutdown()

if __name__ == '__main__':
    main()


"""
colcon build --packages-select wall_follower
source install/setup.bash

ros2 run wall_follower test_node -180
ros2 run wall_follower 2test_node
"""
