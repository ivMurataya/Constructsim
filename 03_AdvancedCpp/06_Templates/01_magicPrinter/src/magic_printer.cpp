#include "unit6_exercises/magic_printer.h"

MagicPrinter::MagicPrinter(ros::NodeHandle &ros_node) {

  this->m_ros_node_object = &ros_node;

  if (ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME,
                                     ros::console::levels::Info)) {
    ros::console::notifyLoggerLevelsChanged();
  }
}

void MagicPrinter::PrintInteger(const int in_value) {

  ROS_INFO_STREAM("Integer Value: " << in_value);
}

void MagicPrinter::PrintString(const string in_value) {

  ROS_INFO_STREAM("String Value: " << in_value);
}


// Destructor
MagicPrinter::~MagicPrinter() {
  cout << "MagicPrinter Destructor is called" << endl;
}
