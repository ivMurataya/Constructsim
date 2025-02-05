cmake_minimum_required(VERSION 3.0.2)
project(laser_reader)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  sensor_msgs
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS  roscpp sensor_msgs
)

## Declare a C++ library
add_library(${PROJECT_NAME}
  src/laser_reader.cpp
)

include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}
${catkin_LIBRARIES}
)
# Add standalone executable file
add_executable(laser_reader_node src/laser_reader.cpp )
	
target_link_libraries(laser_reader_node ${catkin_LIBRARIES} ${PROJECT_NAME})