#!/usr/bin/env python

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from rclpy.qos import ReliabilityPolicy, QoSProfile
import math
import tf_transformations

class AutonomousExplorationNode(Node):
    def __init__(self):
        super().__init__('topics_quiz_node')

        # Subscriber to LaserScan
        
        self.subscriber = self.create_subscription(
            LaserScan,
            '/laser_scan',
            self.laserscan_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        )

        # Subscriber to Odometry
        self.subscriber_odom = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        )

        # Subscriber to Nasa
        self.subscriber = self.create_subscription(
            String,
            '/nasa_mission',
            self.nasa_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        )

        # Publisher for movement commands
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.status_publisher_ = self.create_publisher(String, '/mars_rover_status', 10)


        # Initialize state variables
        self.turning = False
        self.turn_direction = -0.5  # Default to turning right
        self.current_goal = None
        self.goal_tolerance = 0.1
        self.current_position = {'x': 1.0, 'y': 1.0}
       
        self.yaw = 0.0
        self.mission_active = False
        self.obstacle_detected = True
        self.home_position = {'x': 0.0, 'y': 0.0}
        self.pickup_reached = False
        self.pickup_position = {'x': -2.342, 'y': 2.432}

        # Timer to call `go_to_goal`
        self.goal_timer = self.create_timer(0.1, self.goal_timer_callback)


        self.get_logger().info("Autonomous Exploration Node Ready...")

    def goal_timer_callback(self):
        """Callback function for the timer to call go_to_goal."""
        if self.mission_active and self.current_goal:
            self.go_to_goal()

    def nasa_callback(self,msg):
        if msg.data == 'Go-Home':
            self.get_logger().info("Moving to Home")
            self.current_goal = self.home_position
            
            self.mission_active = True
        if msg.data == 'Go-Pickup':
            self.get_logger().info("Moving to PickUp")
            self.current_goal = self.pickup_position
            self.mission_active = True

    def go_to_goal(self):
        """Move the robot toward the current goal."""
        if not self.current_goal:
            return

        if not self.obstacle_detected:
            action = Twist()

            # Calculate the desired angle to the goal
            goal_x = self.current_goal['x']
            goal_y = self.current_goal['y']
            desired_yaw = math.atan2(goal_y - self.current_position['y'], goal_x - self.current_position['x'])

            # Calculate yaw error and normalize to [-pi, pi]
            yaw_error = (desired_yaw - self.yaw + math.pi) % (2 * math.pi) - math.pi

            # Rotate or move toward the goal
            if abs(yaw_error) > 0.2:  # If yaw error is significant, rotate
                action.angular.z = 0.3 if yaw_error > 0 else -0.3
                self.get_logger().info(f'Turning towards goal. Yaw error: {yaw_error:.2f}')
            else:
                # Move forward when aligned
                action.linear.x = 0.4
                self.get_logger().info(f'Moving towards goal. Position: ({self.current_position["x"]:.2f}, {self.current_position["y"]:.2f})')

            # Publish movement command
            self.publisher_.publish(action)
        else:
            # Log that movement toward the goal is paused
            self.get_logger().info("Paused goal movement due to obstacle.")

    def odom_callback(self, msg):
        # Update the current position and yaw from odometry
        self.current_position['x'] = msg.pose.pose.position.x
        self.current_position['y'] = msg.pose.pose.position.y

        orientation_q = msg.pose.pose.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, self.yaw) = tf_transformations.euler_from_quaternion(orientation_list)

        # Check if the goal has been reached
        if self.current_goal:
            distance_to_goal = math.sqrt(
                (self.current_position['x'] - self.current_goal['x'])**2 +
                (self.current_position['y'] - self.current_goal['y'])**2
            )
            if distance_to_goal < self.goal_tolerance:
                self.stop_robot()
                self.mission_active = False
                self.current_goal = None
                self.get_logger().info("Goal reached. Robot stopped.")
                self.status_publisher_.publish(String(data='goal-reached'))
            
    def stop_robot(self):
        stop = Twist()
        # Publish the action command
        self.publisher_.publish(stop)


    def is_at_position(self, target_position):
        """Check if the robot is within the tolerance of a target position."""
        distance_to_target = math.sqrt(
            (self.current_position['x'] - target_position['x'])**2 +
            (self.current_position['y'] - target_position['y'])**2
        )
        return distance_to_target < self.goal_tolerance


    def laserscan_callback(self, msg):
        # Define the sectors
        sectors = {
            "Right_Rear": (0, 33),
            "Right": (34, 66),
            "Front_Right": (67, 100),
            "Front_Left": (101, 133),
            "Left": (134, 166),
            "Left_Rear": (167, 199)
        }

        # Initialize the minimum distances for each sector
        min_distances = {key: float('inf') for key in sectors.keys()}

        # Find the minimum distance in each sector
        for sector, (start_idx, end_idx) in sectors.items():
            # Ensure the index range is within bounds and not empty
            if start_idx < len(msg.ranges) and end_idx < len(msg.ranges):
                sector_ranges = msg.ranges[start_idx:end_idx + 1]
                if sector_ranges:
                    min_distances[sector] = min(sector_ranges)

        # Define the threshold for obstacle detection
        obstacle_threshold = 0.5  # meters

        # Determine detected obstacles
        detections = {sector: min_distance < obstacle_threshold for sector, min_distance in min_distances.items()}

        # New: Determine if there's an obstacle in any sector
        self.obstacle_detected = any(detections.values())
         # Log the detections dictionary
        #self.get_logger().info(f'Obstacle detections: {detections}')
        
        action = Twist()
        # Log the detection status (optional)
        if self.obstacle_detected:
            self.get_logger().info("Obstacle detected in at least one sector.")


            # Determine suggested action based on detection
            

            # If obstacles are detected in both front sectors, continue turning
            if detections["Front_Left"] or detections["Front_Right"]:
                if not self.turning:
                    # Start turning if not already turning
                    self.turning = True
                    self.turn_direction = -0.5  # Turning right
                action.angular.z = self.turn_direction  # Continue turning
                self.get_logger().info('Obstacle ahead, turning to clear path.')
            
            # Priority 2: Side detections
            elif detections["Left"]:
                action.linear.x = 0.2  # Move forward slowly
                action.angular.z = -0.3  # Slight right turn
                self.get_logger().info('Obstacle on the left, turning slightly right.')
            elif detections["Right"]:
                action.linear.x = 0.2  # Move forward slowly
                action.angular.z = 0.3  # Slight left turn
                self.get_logger().info('Obstacle on the right, turning slightly left.')
            # Priority 3: Rear detections
            
            elif detections["Right_Rear"]:
                action.linear.x = 0.3  # Move forward
                self.get_logger().info('Obstacle on the right rear, moving forward.')
            elif detections["Left_Rear"]:
                action.linear.x = 0.3  # Move forward
                self.get_logger().info('Obstacle on the left rear, moving forward.')
            else:
                action.linear.x = 0.5  # Move forward
                self.get_logger().info('No obstacles, moving forward.')

            
            # Publish the action command
            self.publisher_.publish(action)

        else:
            self.turning = False  
            #self.get_logger().info("No obstacles detected.")
             # Only move forward if a goal is set
            if self.current_goal and self.mission_active:
                self.get_logger().info("Path is clear. Resuming goal movement.")
            else:
                # Stop the robot if no goal is active
                action.linear.x = 0.0
                action.angular.z = 0.0
                self.publisher_.publish(action)
                self.get_logger().info("No active goal. Robot stopped.")
            

        
def main(args=None):
    rclpy.init(args=args)
    node = AutonomousExplorationNode()

    # Detect the robot's initial position and decide where to move
    rclpy.spin_once(node)  # Process initial odometry
    
    if not node.is_at_position(node.home_position) and not node.is_at_position(node.pickup_position):
        node.get_logger().info("Initial position not at home or pickup. Moving to home position.")
        node.current_goal = node.home_position
        node.mission_active = True
    
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()


"""
if Publishes in the topic /nasa_mission the command Go-Home it should move to {'x': 0.0, 'y': 0.0} .
if Publishes in the topic /nasa_mission the command Go-Pickup it should move to {'x': -2.342, 'y': 2.432} .
The tolerance to reach those points has to be around 0.1 meters. 
So if the Mars rover reaches 'x': -2.442 it will be still considered that it has reached the correct position for example.

ros2 topic pub /nasa_mission std_msgs/msg/String "data: 'Go-Home'" --once

ros2 topic pub /nasa_mission std_msgs/msg/String "data: 'Go-Pickup'" --once

"""
