#! /usr/bin/env python
import rospy
import time
from drone_trajectories.srv import Dcontrol, DcontrolResponse
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty
from std_msgs.msg import Bool

class DroneClass(object):


    def __init__(self):
        # creates the action server
        self.my_service = rospy.Service('/drone_service', Dcontrol , self.callback)
        self.take_off_pub = rospy.Publisher('/drone/takeoff', Empty, queue_size=1)
        self.land_pub = rospy.Publisher('/drone/land', Empty, queue_size=1)
        self._pub_cmd_vel = rospy.Publisher('/cmd_vel', Twist, queue_size= 10)
        self._mode = rospy.Publisher('/drone/posctrl', Bool,  queue_size= 1)
        self.ctrl_c = False
        self.rate = rospy.Rate(10)
        self.response  = DcontrolResponse()
        self._move_msg = Twist()
    
  
    def publish_once_in_cmd_vel(self, cmd):
        while not self.ctrl_c:
            connections = self._pub_cmd_vel.get_num_connections()
            if connections > 0:
                self._pub_cmd_vel.publish(cmd)
                rospy.loginfo("Publish in cmd_vel...")
                break
            else:
                self.rate.sleep()


    def publish_continuous_cmd(self, cmd, duration):
        start_time = rospy.Time.now()
        while (rospy.Time.now() - start_time).to_sec() < duration:
            self._pub_cmd_vel.publish(cmd)
            self.rate.sleep()

            
  # function that stops the drone from any movement
    def stop_drone(self):
        rospy.loginfo("Stopping...")
        self._move_msg.linear.x = 0.0
        self._move_msg.angular.z = 0.0
        self.publish_once_in_cmd_vel(self._move_msg)
        
  # function that makes the drone turn 90 degrees
    def turn_drone(self):
        rospy.loginfo("Turning...")
        self._move_msg.linear.x = 0.0
        self._move_msg.angular.z = 1.0
        # self.publish_once_in_cmd_vel(self._move_msg)
        self.publish_continuous_cmd(self._move_msg, 1.5)
    
  # function that makes the drone move forward
    def move_forward_drone(self):
        rospy.loginfo("Moving forward...")
        self._move_msg.linear.x = 1.0
        self._move_msg.angular.z = 0.0
        # self.publish_once_in_cmd_vel(self._move_msg)
        self.publish_continuous_cmd(self._move_msg, 1.0)
      
    def takeOff (self):
        req = Empty()
        self.take_off_pub.publish(req)
        rospy.loginfo("Taking Off")

    def land (self):
        req = Empty()
        self.land_pub.publish(req)
        rospy.loginfo("Landing")

    def move_square(self):
        for i in range(4): 
            self.move_forward_drone()
            self.stop_drone()
            self.turn_drone()
            self.stop_drone()

            print("moving square %i" % i)
        self.stop_drone()
        print("Done Square")

    def move_circle(self):
        rospy.loginfo("Moving Circle")
        self._move_msg.linear.x = 0.5
        self._move_msg.linear.y = 0.5
        self.publish_continuous_cmd(self._move_msg, 5.0)

    def moveTriangle(self):
        mode = Bool()
        mode.data = True
        movement = Twist()
        time.sleep(3)
        self._mode.publish(mode)
        rospy.loginfo("mode ctrl")
        time.sleep(3)

        movement.linear.x = 3.0
        movement.linear.y = 2.0
        self.publish_once_in_cmd_vel(movement)
        time.sleep(5)

        movement.linear.x = 1.0
        movement.linear.y = 4.0
        self.publish_once_in_cmd_vel(movement)
        time.sleep(5)

        movement.linear.x = 0.0
        movement.linear.y = 0.0
        self.publish_once_in_cmd_vel(movement)
        time.sleep(5)

        self.stop_drone()
        mode.data = False
        self._mode.publish(mode)
        rospy.loginfo("mode ctrl off")
  

    def callback(self, msg):
        rospy.loginfo(msg.label)
        if msg.label == "square":
            self.takeOff()
            self.move_square()
            self.land()
            self.response.navigation_successfull = True
            self.response.message = 'Completed Square Movement'
        elif msg.label == "circle":
            self.takeOff()
            self.move_circle()
            self.land() 
            self.response.navigation_successfull = True
            self.response.message = 'Competed Circle Movement' 
        elif msg.label == 'triangle':
            self.takeOff()
            self.moveTriangle()
            self.land()
            self.response.navigation_successfull = True
            self.response.message = 'Competed Triangle Movement' 

          
        else:
            self.response.navigation_successfull = False
            self.response.message = 'Service Called Failed'    

        rospy.loginfo(self.response)
        return self.response


    
if __name__ == '__main__':
  rospy.init_node('drone_movement')
  DroneClass()
  rospy.spin()


