import rclpy 
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from rclpy.qos import ReliabilityPolicy, QoSProfile

 
class VelParam(Node):
    def __init__(self):
        super().__init__('param_laser_node')
        self.timer = self.create_timer(2,self.timer_callback)

        

        # Subscriber to LaserScan
        self.subscriber_laser = self.create_subscription(
            LaserScan,
            '/scan',
            self.laserscan_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        )

         # Subscriber to Odom
        self.subscriber_Odom = self.create_subscription(
            Odometry,
            '/odom',
            self.odomCallback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE)
        )

        self.publisher = self.create_publisher(Twist,'cmd_vel',10)
        self.msg = Twist()

        self.declare_parameters(
            namespace='',
            parameters=[
                ('laserParam', False, ParameterDescriptor(
                    description='Enable or disable laser sensor processing'
                )),
                ('odomParam', False, ParameterDescriptor(
                    description='Enable or disable odometry usage'
                )),
                ('velocity', 0.0, ParameterDescriptor(
                    description='Default velocity value for the robot'
                ))
            ]
        )

        
        
    def timer_callback(self):
        my_Vparam = self.get_parameter('velocity').value
        self.get_logger().info("Velocity parameter is: %f" % my_Vparam)
        self.msg.linear.x = my_Vparam
        self.publisher.publish(self.msg)
    
    def laserscan_callback(self,msg):
        my_LParam = self.get_parameter('laserParam').value
        # Get the number of measurements
        num_ranges = len(msg.ranges)
        front_index = num_ranges // 2
        front_range = msg.ranges[front_index]
        # Log the range value in front of the sensor
        if my_LParam:
            self.get_logger().info(f"Range in front: {front_range} meters")

    def odomCallback(self,msg):
        my_oParam = self.get_parameter('odomParam').value
        posX = msg.pose.pose.position.x
        posY = msg.pose.pose.position.y
        if my_oParam:
            self.get_logger().info(f"X: {posX} Y: {posY}")
    


def main(args=None):
    rclpy.init(args=args)
    node = VelParam()
    rclpy.spin(node)

if __name__ == '__main__':
    main()


"""
ros2 param dump /param_laser_node > param_vel_node.yaml
/param_laser_node:
  ros__parameters:
    laserParam: false
    odomParam: false
    use_sim_time: false
    velocity: 0.05

ros2 param set /param_laser_node laserParam False
ros2 param set /param_laser_node odomParam True
ros2 param set /param_laser_node velocity -0.05

"""
