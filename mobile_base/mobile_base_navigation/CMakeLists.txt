cmake_minimum_required(VERSION 2.8.3)
project(mobile_base_navigation)

add_compile_options(-std=c++11)


find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  nav_core
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
  tf2_ros
  costmap_2d
  global_planner
  base_local_planner
)

find_package(yaml-cpp REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES mobile_base_navigation
#  CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy sensor_msgs std_msgs tf
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(lsd
  src/LSD.cpp
)

add_library(detect
  src/WallDetect.cpp
)
target_link_libraries(detect
  lsd
  ${catkin_LIBRARIES}
)

add_library(path_smoother
  src/path_smoother.cpp
)
target_link_libraries(path_smoother
  ${catkin_LIBRARIES}
)

add_executable(detect_node
  src/detect_node.cpp
)
target_link_libraries(detect_node
  detect
  lsd
  ${catkin_LIBRARIES}
)

add_executable(mobile_base_planner
  src/mobile_base_planner.cpp
)
target_link_libraries(mobile_base_planner
  path_smoother
  ${catkin_LIBRARIES}
)

add_executable(init_scan_match
  src/init_scan_match.cpp
)
target_link_libraries(init_scan_match
  yaml-cpp
  ${catkin_LIBRARIES}
)

add_executable(test_map
  src/test_map.cpp
)
target_link_libraries(test_map
  ${catkin_LIBRARIES}
)

add_executable(show_traj
  src/show_traj.cpp
)
target_link_libraries(show_traj
  ${catkin_LIBRARIES}
)
