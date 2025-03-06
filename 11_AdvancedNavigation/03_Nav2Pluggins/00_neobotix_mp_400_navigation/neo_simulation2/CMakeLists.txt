cmake_minimum_required(VERSION 3.5)
project(neo_simulation2)

find_package(ament_cmake REQUIRED)
#find_package(nav2_bringup REQUIRED)
#find_package(navigation2 REQUIRED)

install(DIRECTORY
  robots
  DESTINATION share/${PROJECT_NAME})

ament_package()
