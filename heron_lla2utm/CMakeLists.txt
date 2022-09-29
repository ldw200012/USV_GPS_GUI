cmake_minimum_required(VERSION 2.8.3)
project(heron_lla2utm)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  roscpp
  rospy
  std_msgs
  tf
)

#test on 1.34
set(CMAKE_MODULE_PATH "${CMAKE_MODULE_PATH};/usr/share/cmake/geographiclib")
find_package (GeographicLib REQUIRED)

catkin_package(
 INCLUDE_DIRS include
#  LIBRARIES heron_lla2utm
#  CATKIN_DEPENDS geometry_msgs roscpp rospy std_msgs tf
#  DEPENDS system_lib
)

include_directories(
 include
  ${catkin_INCLUDE_DIRS}
)

add_executable(lla2utm_converter src/convert_main.cpp src/lla2utm_converter.cpp)

target_link_libraries(lla2utm_converter
  ${catkin_LIBRARIES}
  ${GeographicLib_LIBRARIES}
)