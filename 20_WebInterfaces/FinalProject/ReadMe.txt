In this section, the development of a web interface to control a robot performing mapping in a given environment is proposed. 

You must follow the instructions and try to build it as proposed.
Create an interface to help a Turtlebot 2 operator guide the robot throughout the environment in order to map it completely.
You have to show the following content in the webpage:
    Generated map
    3D model of the robot
    Camera viewer
    A virtual joystick to allow manual driving
    Waypoint buttons to send the robot to the given positions: Home (0,0); (3,3); (3,-3); (-3,3); (-3,-3)


python -m SimpleHTTPServer 7000
roslaunch course_web_dev_ros project.launch
roslaunch course_web_dev_ros tf2_web.launch
cp -R $(rospack find kobuki_description) ~/webpage_ws/project/

