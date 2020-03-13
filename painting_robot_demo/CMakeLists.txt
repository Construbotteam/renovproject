cmake_minimum_required(VERSION 2.8.3)
project(painting_robot_demo)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  nav_msgs
  geometry_msgs
  tf
  tf2
  tf2_ros
)

find_package(yaml-cpp REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES painting_robot_demo
#  CATKIN_DEPENDS roscpp rospy std_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_executable(straight_line_path src/straight_line_path.cpp)
target_link_libraries(straight_line_path yaml-cpp ${catkin_LIBRARIES})
