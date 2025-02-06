cmake_minimum_required(VERSION 3.0.2)
project(robot_manager_composition)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_srvs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES robot_manager_composition
#  CATKIN_DEPENDS roscpp std_srvs
#  DEPENDS system_lib
)

catkin_package(
 INCLUDE_DIRS include/${PROJECT_NAME}/
  LIBRARIES robot_manager_composition
  CATKIN_DEPENDS roscpp  std_srvs
  #DEPENDS system_lib
) 

include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)


## Declare a C++ library
add_library(${PROJECT_NAME}_lib
   src/robot_manager_composition.cpp
 )


# add the executable
add_executable(robot_manager_composition_node src/robot_manager_composition_node.cpp src/robot_manager_composition.cpp src/system_information.cpp)
target_link_libraries(robot_manager_composition_node ${catkin_LIBRARIES} robot_manager_composition_lib)


# Add standalone executable file
add_executable(system_information_test src/system_information_test_executable.cpp src/system_information.cpp)
target_link_libraries(system_information_test ${catkin_LIBRARIES})


