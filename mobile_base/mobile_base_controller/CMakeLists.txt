cmake_minimum_required(VERSION 2.8.3)
project(mobile_base_controller)

add_compile_options(-std=c++11)


find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  sensor_msgs
  roscpp
  rospy
  std_msgs
  tf
)

find_package(yaml-cpp REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES mobile_base_controller
#  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy std_msgs tf
#  DEPENDS system_lib
)


include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(mobile_control src/parallel_turn_controller.cpp)
target_link_libraries(mobile_control 
  yaml-cpp 
  ${catkin_LIBRARIES}
)

add_executable(parallel_turn_controller_node src/parallel_turn_controller_node.cpp)
target_link_libraries(parallel_turn_controller_node mobile_control yaml-cpp ${catkin_LIBRARIES})