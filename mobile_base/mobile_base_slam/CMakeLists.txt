cmake_minimum_required(VERSION 2.8.3)
project(mobile_base_slam)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  sensor_msgs
  tf
)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES mobile_base_slam
#  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy sensor_msgs tf
#  DEPENDS system_lib
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)


