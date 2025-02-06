cmake_minimum_required(VERSION 3.0.2)
project(robot_manager_inheritance)

find_package(catkin REQUIRED COMPONENTS
  nav_msgs
  roscpp
  std_srvs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS nav_msgs roscpp std_srvs
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(${PROJECT_NAME}
  src/base_robot_manager.cpp
)

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}
${catkin_LIBRARIES}
)

# Add standalone executable file
add_executable(robot_manager_base_class_test_node 
	src/base_robot_manager_node.cpp 
	src/base_robot_manager.cpp)
	
target_link_libraries(robot_manager_base_class_test_node ${catkin_LIBRARIES} ${PROJECT_NAME})

# add new standalone executable file
add_executable(mobile_robot_robot_manager_node
  src/mobile_robot_robot_manager_node.cpp
  src/mobile_robot_robot_manager.cpp
  src/base_robot_manager.cpp)

target_link_libraries(mobile_robot_robot_manager_node ${catkin_LIBRARIES} ${PROJECT_NAME})
