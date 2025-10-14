#!/usr/bin/env python

"""
[visualization_msgs/Marker]:                                                                                                                                              
uint8 ARROW=0                                                                                                                                                             
uint8 CUBE=1                                                                                                                                                              
uint8 SPHERE=2                                                                                                                                                            
uint8 CYLINDER=3                                                                                                                                                          
uint8 LINE_STRIP=4                                                                                                                                                        
uint8 LINE_LIST=5                                                                                                                                                         
uint8 CUBE_LIST=6                                                                                                                                                         
uint8 SPHERE_LIST=7                                                                                                                                                       
uint8 POINTS=8                                                                                                                                                            
uint8 TEXT_VIEW_FACING=9                                                                                                                                                  
uint8 MESH_RESOURCE=10                                                                                                                                                    
uint8 TRIANGLE_LIST=11                                                                                                                                                    
uint8 ADD=0                                                                                                                                                               
uint8 MODIFY=0                                                                                                                                                            
uint8 DELETE=2                                                                                                                                                            
std_msgs/Header header                                                                                                                                                    
  uint32 seq                                                                                                                                                              
  time stamp                                                                                                                                                              
  string frame_id                                                                                                                                                         
string ns                                                                                                                                                                 
int32 id                                                                                                                                                                  
int32 type                                                                                                                                                                
int32 action                                                                                                                                                              
geometry_msgs/Pose pose                                                                                                                                                   
  geometry_msgs/Point position                                                                                                                                            
    float64 x                                                                                                                                                             
    float64 y                                                                                                                                                             
    float64 z                                                                                                                                                             
  geometry_msgs/Quaternion orientation                                                                                                                                    
    float64 x                                                                                                                                                             
    float64 y                                                                                                                                                             
    float64 z                                                                                                                                                             
    float64 w                                                                                                                                                             
geometry_msgs/Vector3 scale                                                                                                                                               
  float64 x                                                                                                                                                               
  float64 y                                                                                                                                                               
  float64 z                                                                                                                                                               
std_msgs/ColorRGBA color                                                                                                                                                  
  float32 r                                                                                                                                                               
  float32 g                                                                                                                                                               
  float32 b 
duration lifetime                                                                                                                                                         
bool frame_locked                                                                                                                                                         
geometry_msgs/Point[] points                                                                                                                                              
  float64 x                                                                                                                                                               
  float64 y                                                                                                                                                               
  float64 z                                                                                                                                                               
std_msgs/ColorRGBA[] colors                                                                                                                                               
  float32 r                                                                                                                                                               
  float32 g                                                                                                                                                               
  float32 b                                                                                                                                                               
  float32 a                                                                                                                                                               
string text                                                                                                                                                               
string mesh_resource                                                                                                                                                      
bool mesh_use_embedded_materials
"""


import rospy
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point
from std_msgs.msg import Bool

rospy.init_node('marker_subscriber')   
rospy.sleep(0.5)

marker_objectlisher = rospy.Publisher('/marker_basic', Marker, queue_size=1)

marker_object = Marker()
marker_object.header.frame_id = '/world'
marker_object.header.stamp = rospy.get_rostime()
marker_object.ns = 'haro'
marker_object.id = 1 
marker_object.type = Marker.SPHERE
marker_object.action = Marker.ADD

my_point = Point()
my_point.z = 3
marker_object.pose.position = my_point
marker_object.pose.orientation.x = 0
marker_object.pose.orientation.y = 0
marker_object.pose.orientation.z = 0
marker_object.pose.orientation.w = 1.0
marker_object.scale.x = 5.0
marker_object.scale.y = 5.0
marker_object.scale.z = 5.0
marker_object.color.r = 0.0
marker_object.color.g = 0.0
marker_object.color.b = 1.0
# This has to be; otherwise, it will be transparent
marker_object.color.a = 1.0

# If we want it forever, 0, otherwise seconds before disappearing
marker_object.lifetime = rospy.Duration(0)


marker_objectlisher.publish(marker_object)

# publish repeatedly so RViz catches it
rate = rospy.Rate(1)  # 1 Hz
while not rospy.is_shutdown():
    marker_object.header.stamp = rospy.get_rostime()
    marker_objectlisher.publish(marker_object)
    rate.sleep()
