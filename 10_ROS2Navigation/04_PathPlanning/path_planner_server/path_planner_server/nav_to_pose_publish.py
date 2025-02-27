import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped, PoseStamped

class InitialPosePublisher(Node):
    def __init__(self):
        super().__init__('initial_pose_pub')
        self.get_logger().info("Node Inizialided")
        
        # Subscriber to /clicked_point
        self.subscription = self.create_subscription(
            PointStamped,
            '/clicked_point',
            self.clicked_point_callback,
            10)
        
        # Publisher to /initial_pose
        self.publisher = self.create_publisher(
            PoseStamped,
            '/goal_pose',
            10)
    
    def clicked_point_callback(self, msg):
        # Extract X and Y coordinates
        x = msg.point.x
        y = msg.point.y
        
        # Create and populate the PoseWithCovarianceStamped message
        initial_pose_msg = PoseStamped()
        initial_pose_msg.header.stamp = self.get_clock().now().to_msg()
        initial_pose_msg.header.frame_id = 'map'
        initial_pose_msg.pose.position.x = x
        initial_pose_msg.pose.position.y = y
        initial_pose_msg.pose.position.z = 0.0
        initial_pose_msg.pose.orientation.w = 1.0  # Neutral orientation
        
        # Publish the initial pose
        self.publisher.publish(initial_pose_msg)
        self.get_logger().info(f'Published goal pose: x={x}, y={y}')


def main(args=None):
    rclpy.init(args=args)
    node = InitialPosePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()


"""
rviz2 -d ~/ros2_ws/src/path_planner_server/rviz_path.rviz
ros2 launch localization_server localization.launch.py
ros2 launch path_planner_server pathplanner.launch.py
ros2 topic echo /clicked_point
ros2 run path_planner_server goal_clicked_node

ros2 topic pub /goal_pose geometry_msgs/msg/PoseStamped 
"header:
  stamp:
    sec: 0
    nanosec: 0
  frame_id: 'map'
pose:
  position:
    x: -0.5444
    y: 4.3941
    z: 0.0
  orientation:
    x: 0.0
    y: 0.0
    z: 0.0
    w: 1.0" --once

ros2 topic pub -1 /goal_pose geometry_msgs/PoseStamped "{header: {stamp: {sec: 0}, frame_id: 'map'}, pose: {position: {x: 2.2, y: 0.0, z: 0.0}, orientation: {w: 1.0}}}"

"""