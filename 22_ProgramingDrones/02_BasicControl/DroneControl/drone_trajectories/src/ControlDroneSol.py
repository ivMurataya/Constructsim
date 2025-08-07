#! /usr/bin/env python

import rospy
# from drone_trajectories.srv import DroneTrajectory, DroneTrajectoryResponse
from drone_trajectories.srv import Dcontrol, DcontrolResponse

from geometry_msgs.msg import Twist
from std_msgs.msg import Empty
import time

class DroneTrajectoryNode():
    
    def __init__(self):
        self._my_service = rospy.Service('/drone_trajectory_service', Dcontrol , self.my_callback)
        self._vel_publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self._takeoff_publisher = rospy.Publisher('/drone/takeoff', Empty, queue_size=1)
        self._land_publisher = rospy.Publisher('/drone/land', Empty, queue_size=1)
        self._move_msg = Twist()
        self._empty_msg = Empty()
        self._ctrl_c = False
        self._rate = rospy.Rate(1)
        rospy.on_shutdown(self.shutdownhook)

    def publish_once_in_cmd_vel(self, cmd):
        """
        This is because publishing in topics sometimes fails the first time you publish.
        In continuos publishing systems there is no big deal but in systems that publish only
        once it IS very important.
        """
        while not self._ctrl_c:
            connections = self._vel_publisher.get_num_connections()
            if connections > 0:
                self._vel_publisher.publish(cmd)
                break
            else:
                self._rate.sleep()

    # function that makes the drone stop
    def stop_drone(self):
        rospy.loginfo("Moving forward...")
        self._move_msg.linear.x = 0.0
        self._move_msg.angular.z = 0.0
        self.publish_once_in_cmd_vel(self._move_msg)

    # function that makes the drone turn
    def turn_drone(self):
        rospy.loginfo("Turning...")
        self._move_msg.linear.x = 0.0
        self._move_msg.angular.z = 0.5
        self.publish_once_in_cmd_vel(self._move_msg)

    # function that makes the drone move forward
    def move_forward_drone(self):
        rospy.loginfo("Moving forward...")
        self._move_msg.linear.x = 1.0
        self._move_msg.angular.z = 0.0
        self.publish_once_in_cmd_vel(self._move_msg)

    # function that makes the drone move in a circle
    def move_circle_drone(self):
        rospy.loginfo("Moving forward...")
        self._move_msg.linear.x = 0.5
        self._move_msg.angular.z = 0.5
        self.publish_once_in_cmd_vel(self._move_msg)

    def takeoff(self):
        rospy.loginfo("Taking off...")
        while not self._ctrl_c:
            connections = self._takeoff_publisher.get_num_connections()
            if connections > 0:
                self._takeoff_publisher.publish(self._empty_msg)
                break
            else:
                self._rate.sleep()
            
    def land(self):
        rospy.loginfo("Landing...")
        while not self._ctrl_c:
            connections = self._land_publisher.get_num_connections()
            if connections > 0:
                self._land_publisher.publish(self._empty_msg)
                break
            else:
                self._rate.sleep()
        
    def shutdownhook(self):
        # works better than the rospy.is_shutdown()
        self.stop_drone()
        self._ctrl_c = True

    def my_callback(self, request):
        rospy.loginfo("The Service drone_trajectory_service has been called")
        self.takeoff()
        time.sleep(3)

        if request.label == "square":        
            self.move_forward_drone()
            time.sleep(3)
            self.stop_drone()
            self.turn_drone()
            time.sleep(3)
            self.stop_drone()
            self.move_forward_drone()
            time.sleep(3)
            self.stop_drone()
            self.turn_drone()
            time.sleep(3)
            self.stop_drone()
            self.move_forward_drone()
            time.sleep(3)
            self.stop_drone()
            self.turn_drone()
            time.sleep(3)
            self.stop_drone()
            self.move_forward_drone()
            time.sleep(3)
            self.stop_drone()

        elif request.label == "circle":
            self.move_circle_drone()
            time.sleep(15)
            self.stop_drone()

        elif request.label == "triangle":
            self.move_forward_drone()
            time.sleep(3)
            self.stop_drone()
            self.turn_drone()
            time.sleep(5)
            self.stop_drone()
            self.move_forward_drone()
            time.sleep(2)
            self.stop_drone()
            self.turn_drone()
            time.sleep(3)
            self.stop_drone()
            self.move_forward_drone()
            time.sleep(2)
            self.stop_drone()

        self.land()
        time.sleep(3)
        
        response = DcontrolResponse()
        response.navigation_successfull = True
        response.message = "Finished trajectory"
        return response
            
if __name__ == '__main__':
    rospy.init_node('drone_trajectory_node', anonymous=True)
    drone_trajectory = DroneTrajectoryNode()
    try:
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
