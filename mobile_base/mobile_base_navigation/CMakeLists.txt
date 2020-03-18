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

catkin_package()

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(diff_planner 
  src/diff_planner.cpp 
  src/velo_generator.cpp
  src/broke_line.cpp
)
target_link_libraries(diff_planner  ${catkin_LIBRARIES})


# add_executable(diff_planner src/diff_base_planner.cpp)
# target_link_libraries(diff_planner ${catkin_LIBRARIES})

add_executable(diff_base_planner src/diff_base_planner.cpp)
target_link_libraries(diff_base_planner diff_planner ${catkin_LIBRARIES})
