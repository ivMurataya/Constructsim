import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from leo_description.action import Rotate


class MyActionClient(Node):

    """
    In the constructor of the class, initialize a ROS2 node named my_action_client. Also, and very important, create an ActionClient object to which you pass three arguments:

    The ROS2 node that contains the Action Client: in this case, self.
    The type of the Action: Rotate (related to the leo_description interface of type Action).
    The Action name: rotate.

    """
    def __init__(self):
        super().__init__('my_action_client')
        self._action_client = ActionClient(self, Rotate, 'rotate')


    """
    Start by creating a Goal() object of the Rotate action type. 
    Then, access the rotation_time variable of the Action goal and assign 
    it the value of seconds (which is five in this example).
    """
    def send_goal(self, seconds):
        """
        Start by creating a Goal() object of the Rotate action type. 
        Then, access the rotation_time variable of the Action goal 
        and assign it the value of seconds
        """
        goal_msg = Rotate.Goal()
        goal_msg.rotation_time = seconds

        """
        Next, wait for the Action Server to be up and running
        And send the goal to the Server using the send_goal_async method
        You need to provide two arguments for this method:
            A goal message, in this case, goal_msg
            A callback function for the feedback, in this case, self.feedback_callback

        This send_goal_async() method returns a future to a goal handle. 
        This future goal handle will be completed when the Server has processed the goal. 
        (This means it has been accepted or rejected by the Server). 
        So, you must assign a callback method to be triggered when 
        the future is completed (the goal has been accepted or rejected). 
        In this case, this method is self.goal_response_callback
        """
        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback)

        self._send_goal_future.add_done_callback(self.goal_response_callback)


    
    """
    So, this method will be triggered when the goal has been processed. 
    First, check whether the Server has accepted the goal
    """
    def goal_response_callback(self, future):
        """
        Print a message if it has been rejected. 
        If it has been accepted, ask for the result using the get_result_async() method
        """
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        """
        Similar to sending the goal, this method will return a future that will 
        be completed when the result is ready. So, you must also assign a callback 
        method to be triggered when this future is completed (the result is ready). 
        In this case, this method is self.get_result_callback
        """
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)


    """
    You get the result (future.result().result), print, and then shut down the 
    node for a clean exit with rclpy.shutdown()
    """
    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result.success))
        rclpy.shutdown()

    """
    Here you get the feedback bool, which correspond to the elapsed time, 
    from the feedback_msg and print it to the screen.
    """
    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(
           'Received feedback: {0}'.format(feedback.elapsed_time))


def main(args=None):
    rclpy.init(args=args)

    action_client = MyActionClient()

    action_client.send_goal(5.0)

    rclpy.spin(action_client)


if __name__ == '__main__':
    main()

#ros2 pkg create my_action_client --build-type ament_python --dependencies rclpy rclpy.action leo_description
