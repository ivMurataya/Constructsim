/*
cd ~/catkin_ws/src
catkin_create_pkg test_library roscpp
cd test_library
touch src/test_library.cpp
touch include/test_library/test_library.h

-----------------------------------------------------------------------------
Copy the following content in test_library.h

#ifndef TEST_LIBRARY_H
#define TEST_LIBRARY_H

#include <ros/ros.h>

void display_pos(float x, float y);

#endif
-----------------------------------------------------------------------------
Copy the following content in test_library.cpp
#include "../include/test_library/test_library.h"

void display_pos(float x, float y) {
     
    float pos_x = x;
    float pos_y = y;
    std::cout << "Position X: " << pos_x << " Position Y: " << pos_y << std::endl;
     
}
-----------------------------------------------------------------------------
So far we have created a header file and a source file. Copy the following content for the CMakeLists.txt
cmake_minimum_required(VERSION 3.0.2)
project(test_library)


find_package(catkin REQUIRED COMPONENTS
  roscpp
)


catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
  CATKIN_DEPENDS roscpp
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(${PROJECT_NAME}
  src/${PROJECT_NAME}.cpp
)


## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}
${catkin_LIBRARIES}
)

install(TARGETS ${PROJECT_NAME}
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})
-----------------------------------------------------------------------------
Now build the package by navigating to the catkin_ws directory and running the below command:
catkin_make
-----------------------------------------------------------------------------

*/
