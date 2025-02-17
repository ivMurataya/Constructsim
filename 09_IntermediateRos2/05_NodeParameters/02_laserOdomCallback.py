import rclpy 
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from rclpy.qos import ReliabilityPolicy, QoSProfile
from rcl_interfaces.msg import SetParametersResult
from rclpy.parameter import Parameter
 
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

        self.add_on_set_parameters_callback(self.parameter_callback)

        
        
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

    def parameter_callback(self, params):
        for param in params:
            if param.name == 'velocity' and param.type_ == Parameter.Type.DOUBLE:
                #self.my_param = param.value
                self.get_logger().info('Velocity parameter changed!')
            if param.name == 'laserParam' and param.type_ == Parameter.Type.BOOL:
                #self.my_param = param.value
                self.get_logger().info('Laser parameter changed!')
            if param.name == 'odomParam' and param.type_ == Parameter.Type.BOOL:
                #self.my_param = param.value
                self.get_logger().info('Odom parameter changed!')                
        return SetParametersResult(successful=True)

    


def main(args=None):
    rclpy.init(args=args)
    node = VelParam()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
