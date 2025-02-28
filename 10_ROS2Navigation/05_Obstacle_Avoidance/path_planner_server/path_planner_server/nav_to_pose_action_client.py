import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PointStamped, PoseStamped


class MyActionClient(Node):

    def __init__(self):
        super().__init__('my_action_client')
        self._action_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.subscription = self.create_subscription(PointStamped,'/clicked_point',self.clicked_point_callback,10)
        self.initial_pose_msg = None
        self.get_logger().info('Action Client Node Ready!')
        

    def clicked_point_callback(self, msg):
        # Extract X and Y coordinates
        x = msg.point.x
        y = msg.point.y
        
        # Create and populate the PoseWithCovarianceStamped message
        self.initial_pose_msg = PoseStamped()
        self.initial_pose_msg.header.stamp = self.get_clock().now().to_msg()
        self.initial_pose_msg.header.frame_id = 'map'
        self.initial_pose_msg.pose.position.x = x
        self.initial_pose_msg.pose.position.y = y
        self.initial_pose_msg.pose.position.z = 0.0
        self.initial_pose_msg.pose.orientation.w = 1.0  # Neutral orientation
    
        self.get_logger().info(f'Published goal pose: x={x}, y={y}')
        self.send_goal()



    def send_goal(self):
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = self.initial_pose_msg
        goal_msg.behavior_tree = ""

        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
 
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result()
        self.get_logger().info('Result Suceed')
        

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        #feedback.estimated_time_remaining
        self.get_logger().info(
                            f'\nTime Rem: {feedback.estimated_time_remaining.sec}\n'
                            + f'Nav Time : {feedback.navigation_time.sec}\n'
                            + f'Distance : {feedback.distance_remaining}\n'
                            + f'Num Recoveries : {feedback.number_of_recoveries}\n'
                            + f'X : {feedback.current_pose.pose.position.x}\n'
                            + f'Y : {feedback.current_pose.pose.position.y}\n'
                            )



def main(args=None):
    rclpy.init(args=args)

    action_client = MyActionClient()

    rclpy.spin(action_client)
    rclpy.shutdown()


if __name__ == '__main__':
    main()

#ros2 pkg create my_action_client --build-type ament_python --dependencies rclpy rclpy.action leo_description
"""
rviz2 -d ~/ros2_ws/src/path_planner_server/rviz_path.rviz
ros2 launch localization_server localization.launch.py
ros2 launch path_planner_server pathplanner.launch.py
ros2 run path_planner_server goal_action_node
ros2 run path_planner_server solution_action_node

"""


"""
ros2 interface show nav2_msgs/action/NavigateToPose
#goal definition
geometry_msgs/PoseStamped pose
        std_msgs/Header header
                builtin_interfaces/Time stamp
                        int32 sec
                        uint32 nanosec
                string frame_id
        Pose pose
                Point position
                        float64 x
                        float64 y
                        float64 z
                Quaternion orientation
                        float64 x 0
                        float64 y 0
                        float64 z 0
                        float64 w 1
string behavior_tree
---
#result definition
std_msgs/Empty result
---
#feedback definition
geometry_msgs/PoseStamped current_pose
        std_msgs/Header header
                builtin_interfaces/Time stamp
                        int32 sec
                        uint32 nanosec
                string frame_id
        Pose pose
                Point position
                        float64 x
                        float64 y
                        float64 z
                Quaternion orientation
                        float64 x 0
                        float64 y 0
                        float64 z 0
                        float64 w 1
builtin_interfaces/Duration navigation_time
        int32 sec
        uint32 nanosec
builtin_interfaces/Duration estimated_time_remaining
        int32 sec
        uint32 nanosec
int16 number_of_recoveries
float32 distance_remaining
"""