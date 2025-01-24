import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from nav_msgs.msg import Odometry
from rclpy.action import ActionServer
from custom_interfaces.action import OdomRecord  
import math
import time
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

class OdomRecorder(Node):
    def __init__(self):
        super().__init__('odom_recorder')
        self.reentrant_group_1 = ReentrantCallbackGroup()

        # Subscriber to the /odom topic
        self.odom_subscriber = self.create_subscription(Odometry, '/odom', self.odom_callback, 10,callback_group=self.reentrant_group_1)
        
        self.last_odom = Point()

        # Action server
        self.action_server = ActionServer(self,OdomRecord,'record_odom',self.action_callback,callback_group=self.reentrant_group_1)

        self.first_odom = None
        self.odom_record = []
        self.total_distance = 0.0
        self.last_x = None
        self.last_y = None
        self.getfisrtOdom = False
        self.isFirt = False
        self.distance_to_start = 1.0
        self.get_logger().info("Action Server Ready...")

    def odom_callback(self, msg):
        """Callback to handle /odom messages."""
        self.last_odom.x = msg.pose.pose.position.x
        self.last_odom.y = msg.pose.pose.position.y
        self.last_odom.z = msg.pose.pose.position.z
        if self.getfisrtOdom == False:
            self.saveFirst()
            
    def saveFirst(self):
        self.first_odom = Point()
        self.first_odom.x = self.last_odom.x
        self.first_odom.y = self.last_odom.y
        self.first_odom.z = self.last_odom.z
        self.last_x = self.last_odom.x
        self.last_y = self.last_odom.y
        self.getfisrtOdom = True
        self.odom_record.append(Point(x=self.last_odom.x, y=self.last_odom.y, z=self.last_odom.z))
        self.get_logger().info(f"Fisrt saved = Odom {self.first_odom}")
        
    async def action_callback(self, goal_handle):
        """Callback to handle action server requests."""
        self.get_logger().info('Received goal request')   
    
        feedback_msg = OdomRecord.Feedback()
        result = OdomRecord.Result() 

        while self.distance_to_start > 0.05:
            # Append the latest odometry to the record
            self.odom_record.append(Point(x=self.last_odom.x, y=self.last_odom.y, z=self.last_odom.z))
    
            # Compute the distance traveled since the last step
            distance = math.sqrt((self.last_odom.x - self.last_x) ** 2 + (self.last_odom.y - self.last_y) ** 2)
            self.total_distance += distance
            self.last_x = self.last_odom.x
            self.last_y = self.last_odom.y

            # Provide feedback
            feedback_msg.current_total = self.total_distance
            goal_handle.publish_feedback(feedback_msg)
            self.get_logger().info(f"D TOTAL is {self.total_distance}")

            # Check if the robot has returned to the starting position
            self.distance_to_start = math.sqrt((self.last_odom.x - self.first_odom.x) ** 2 + (self.last_odom.y - self.first_odom.y) ** 2)
            self.get_logger().info(f"D to Start is {self.distance_to_start}")
            time.sleep(1.0)
            
        if self.distance_to_start <= 0.05:  # 5 cm
            self.get_logger().info('Robot has returned to the starting position.')
            self.getfisrtOdom = False
            self.isFirt = False
            result.list_of_odoms = self.odom_record
            goal_handle.succeed()
            return result
            
def main(args=None):
    rclpy.init(args=args)
    odom_recorder = OdomRecorder()

    # Use MultiThreadedExecutor
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(odom_recorder)
    try:
        executor.spin()
    except KeyboardInterrupt:
        odom_recorder.get_logger().info('Node interrupted, shutting down...')
    finally:
        odom_recorder.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
