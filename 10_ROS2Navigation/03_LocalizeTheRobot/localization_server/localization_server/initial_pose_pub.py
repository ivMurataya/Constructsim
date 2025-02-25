import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PointStamped, PoseWithCovarianceStamped

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
            PoseWithCovarianceStamped,
            '/initialpose',
            10)
    
    def clicked_point_callback(self, msg):
        # Extract X and Y coordinates
        x = msg.point.x
        y = msg.point.y
        
        # Create and populate the PoseWithCovarianceStamped message
        initial_pose_msg = PoseWithCovarianceStamped()
        initial_pose_msg.header.stamp = self.get_clock().now().to_msg()
        initial_pose_msg.header.frame_id = '/map'
        initial_pose_msg.pose.pose.position.x = x
        initial_pose_msg.pose.pose.position.y = y
        initial_pose_msg.pose.pose.position.z = 0.0
        initial_pose_msg.pose.pose.orientation.w = 1.0  # Neutral orientation
        
        # Publish the initial pose
        self.publisher.publish(initial_pose_msg)
        self.get_logger().info(f'Published initial pose: x={x}, y={y}')


def main(args=None):
    rclpy.init(args=args)
    node = InitialPosePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
