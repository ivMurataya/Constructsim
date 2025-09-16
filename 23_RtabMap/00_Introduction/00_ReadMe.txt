RTAB-Map (Real-Time Appearance-Based Mapping) is a RGB-D SLAM approach based on a loop closure detector. 
The loop closure detector uses a bag-of-words approach in order to determinate if a new image detected by 
an RGB-D sensor it is from a new location or from a location that it has been already visited. 
Of course, this is a very summarized explanation, you will get more details on how this loop closure 
detector works inside this Course. For using this approach in ROS, there exists the following package:
rtabmap_ros
