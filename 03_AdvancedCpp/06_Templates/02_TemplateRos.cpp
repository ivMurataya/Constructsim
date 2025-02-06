//catkin_ws/src/template_exercises/include/template_exercises/subscriber_basic.h
#ifndef MAGIC_SUBSCRIBER_H
#define MAGIC_SUBSCRIBER_H

#include <ros/ros.h>
#include <sensor_msgs/Image.h>

using namespace std;

class MagicSubscriber {
public:
  MagicSubscriber();
  ~MagicSubscriber();

  template <typename ROSMessageType>
  void init(ros::NodeHandle &ros_node, const string subscriber_topic) {
    // We used an initialiser list
    this->m_subscriber_topic = subscriber_topic;
    this->m_ros_node_object = &ros_node;

    if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info)) {
      ros::console::notifyLoggerLevelsChanged();
    }

    image_sub_ = this->m_ros_node_object->subscribe(this->m_subscriber_topic, 1,&MagicSubscriber::CallbackToTopic<ROSMessageType>, this);
  };

  template <typename ROSMessageType>
  void CallbackToTopic(const typename ROSMessageType::ConstPtr &msg) {
    // the uint8 is an alias of unsigned char, therefore needs casting to int
    ROS_INFO_STREAM("Call Back Topic Image Data[0]=" << static_cast<int>(msg->data[0]));
  };

  template <typename T> 
  void PrintGeneric(T in_value) {
        ROS_INFO_STREAM("Generic Value=" << in_value);
    };

private:
  ros::NodeHandle *m_ros_node_object;
  string m_subscriber_topic;
  ros::Subscriber image_sub_;
};

#endif
//------------------------------------------------------------------------------------------------------------------------------
//catkin_ws/src/template_exercises/src/subscriber_basic.cpp
#include "template_exercises/subscriber_basic.h"

MagicSubscriber::MagicSubscriber() {
  cout << "MagicSubscriber Constructor is called" << endl;
}

// Destructor
MagicSubscriber::~MagicSubscriber() {
  cout << "MagicSubscriber Destructor is called" << endl;
}

//------------------------------------------------------------------------------------------------------------------------------
//catkin_ws/src/template_exercises/src/main_subscriber_basic.cpp
  #include "template_exercises/subscriber_basic.h"
//#include "unit6_exercises/magic_subscriber_function.h"
#include <sensor_msgs/Image.h>

int main(int argc, char **argv) {

  ros::init(argc, argv, "magic_susbcriber_main_node");

  ros::NodeHandle _n("magic_susbcriber_main_ns");
  string topic_name = "/camera/rgb/image_raw";

  MagicSubscriber magic_subscriber_object;

  magic_subscriber_object.init<sensor_msgs::Image>(_n, topic_name);

  string vallue_string = "Adventure Time!";
  int value_integer = 42;
  magic_subscriber_object.PrintGeneric<int>(value_integer);
  magic_subscriber_object.PrintGeneric<string>(vallue_string);

  ros::spin();

  return 0;
}
//------------------------------------------------------------------------------------------------------------------------------
//catkin_ws/src/template_exercises/CMakeLists.txt
cmake_minimum_required(VERSION 3.0.2)
project(template_exercises)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++17)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES template_exercises
  CATKIN_DEPENDS roscpp sensor_msgs
#  DEPENDS system_lib
)

include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)


add_library(subscriber_basic_library
  src/subscriber_basic.cpp
)

## Create executables
add_executable(main_subscriber_basic_node src/main_subscriber_basic.cpp)

target_link_libraries(main_subscriber_basic_node
	subscriber_basic_library
	${catkin_LIBRARIES}
)



