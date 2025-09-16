2.1   System Requirements
In order to be able to use the rtabmap_ros package to perform RGB-D SLAM, you need to have, at least, a Kinect-like sensor.

 Anyways, the recommended robot configuration is the following:
A 2D laser which provides sensor_msgs/LaserScan messages.
Odometry (IMU, wheel encoders, ...) which provides a nav_msgs/Odometry message.
A calibrated Kinect-like sensor compatible with openni_launch, openni2_launch or freenect_launch ros packages.
