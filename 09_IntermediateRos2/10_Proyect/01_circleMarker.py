#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

class CircleMarkerPublisher(Node):
    def __init__(self, x, y, z):
        super().__init__('circle_marker_publisher')
        self.publisher = self.create_publisher(Marker, 'visualization_marker', 10)
        self.timer = self.create_timer(0.1, self.publish_marker)  # Publish at 10Hz
        self.x = x
        self.y = y
        self.z = z if z is not None else 0.0  # Default to 0 if None

    def publish_marker(self):
        marker = Marker()
        marker.header.frame_id = "base_link"  # Change this frame if needed
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "detected_circles"
        marker.id = 0
        marker.type = Marker.SPHERE  # Use a sphere to represent the detected circle
        marker.action = Marker.ADD

        # Set position
        marker.pose.position.x = self.x
        marker.pose.position.y = self.y
        marker.pose.position.z = self.z

        # Set scale (size of the marker)
        marker.scale.x = 0.29303981  # Adjust as needed
        marker.scale.y = 0.29303981
        marker.scale.z = 0.29303981

        # Set color (RGBA)
        marker.color.r = 1.0
        marker.color.g = 0.0
        marker.color.b = 0.0
        marker.color.a = 1.0  # Fully visible

        # Other marker properties
        marker.lifetime = rclpy.duration.Duration().to_msg()

        self.get_logger().info(f"Publishing circle marker at X: {self.x}, Y: {self.y}, Z: {self.z}")
        self.publisher.publish(marker)

def main(args=None):
    rclpy.init(args=args)

    # Extracted values from previous steps
    X = 9.719781346611978
    Y = -3.2793135331625534
    Z = 0.2728375209299161

    marker_publisher = CircleMarkerPublisher(X, Y, Z)
    rclpy.spin(marker_publisher)

    marker_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
