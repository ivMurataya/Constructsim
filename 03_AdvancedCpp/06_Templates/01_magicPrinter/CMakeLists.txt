cmake_minimum_required(VERSION 3.0.2)
project(unit6_exercises)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++17) 

find_package(catkin REQUIRED COMPONENTS
  roscpp
)

catkin_package(
  INCLUDE_DIRS include
 LIBRARIES unit6_exercises
 CATKIN_DEPENDS roscpp
# DEPENDS system_lib
)


include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)


## Declare a C++ library
add_library(magic_printer_library
  src/magic_printer.cpp
)

## Create executables
add_executable(main_magic_printer_node src/main_magic_printer.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(main_magic_printer_node
    magic_printer_library
    ${catkin_LIBRARIES}
)