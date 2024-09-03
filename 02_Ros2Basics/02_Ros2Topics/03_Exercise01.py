import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from rclpy.qos import ReliabilityPolicy, QoSProfile


class RobotController(Node):

    def __init__(self):
        super().__init__('robot_controller')

        # Publisher for controlling the robot's velocity
        self.velocity_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # Subscriber for laser scan data
        self.laser_subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.laser_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))

        self.laser_subscription  # prevent unused variable warning

        # Timer to regularly check and publish commands
        self.timer = self.create_timer(0.1, self.move_robot)
        self.get_logger().info('Robot Controller node has been started.')

        # Initialize Twist message for velocity commands
        self.cmd = Twist()

        # Parameters for obstacle detection and movement control
        self.distance_front_threshold = 5.0  # meters (start moving straight when an object is detected within 5 meters)
        self.stop_distance_threshold = 0.5   # meters (stop when distance is less than 50 cm)
        self.wall_follow_distance = 0.5      # meters (target distance to maintain from the wall)
        self.min_distance_ahead = float('inf')  # initialize with a large number

    def laser_callback(self, msg):
        # Find the minimum distance in front of the robot (within a certain angle range)
        front_angles = range(-15, 15)  # Check approximately 30 degrees in front
        distances = [msg.ranges[i] for i in front_angles if 0 <= i < len(msg.ranges)]

        # Find the minimum distance within these ranges
        if distances:
            self.min_distance_ahead = min(distances)
        else:
            self.min_distance_ahead = float('inf')

        self.get_logger().info(f'Minimum distance ahead: {self.min_distance_ahead:.2f} meters')

    def move_robot(self):
        # Decision-making based on laser scan data
        if self.min_distance_ahead > self.distance_front_threshold:
            # Turn left until an object is detected within 5 meters
            self.cmd.linear.x = 0.0
            self.cmd.angular.z = 0.5  # Turn left
            self.get_logger().info('Turning left to find an obstacle...')
        elif self.min_distance_ahead > self.wall_follow_distance:
            # Move towards the wall if the distance is greater than 50 cm
            self.cmd.linear.x = 0.1  # Move forward slowly (0.1 m/s)
            self.cmd.angular.z = 0.0  # No turning
            self.get_logger().info('Moving towards the wall...')
        elif self.min_distance_ahead <= self.stop_distance_threshold:
            # Stop if the distance is less than or equal to 50 cm
            self.cmd.linear.x = 0.0
            self.cmd.angular.z = 0.0
            self.get_logger().info('Too close to the wall! Stopping...')
        else:
            # In case the robot is within a controlled distance range, move forward slowly
            self.cmd.linear.x = 0.1  # Continue moving forward slowly
            self.cmd.angular.z = 0.0
            self.get_logger().info('Maintaining safe distance. Moving forward slowly...')

        # Publish the velocity command
        self.velocity_publisher.publish(self.cmd)


def main(args=None):
    rclpy.init(args=args)
    robot_controller = RobotController()
    rclpy.spin(robot_controller)
    robot_controller.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


"""
Write a Publisher and Subscriber node:

    Create a new package and call it exercise31_pkg with rclpy, std_msgs, sensor_msgs, and geometry_msgs as dependencies.
    Use the previous publisher code as a template to create a new program that makes the robot turn left using Twist messages.
    Get information from the laser and use the LaserScan messages to decide how to move.
        Turn left until you get a value lower than 5m (from the laser direction in front of the robot). Then, move straight.
        If the distance received is greater than 50cm, move in a straight line towards the wall (do not go too fast!).
        If the distance received is less than 50cm, the robot stops.



"""
