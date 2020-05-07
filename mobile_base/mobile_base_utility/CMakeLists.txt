cmake_minimum_required(VERSION 2.8.3)
project(mobile_base_utility)

add_compile_options(-std=c++11)

find_package(
  catkin REQUIRED
  COMPONENTS geometry_msgs
             nav_msgs
             roscpp
             rospy
             std_msgs
             tf
             tf2
             tf2_ros
             map_server)

find_package(yaml-cpp REQUIRED)

catkin_package(
  # INCLUDE_DIRS include
  # LIBRARIES mobile_base_utility
  # CATKIN_DEPENDS geometry_msgs nav_msgs roscpp rospy std_msgs tf tf2 tf2_ros
  # DEPENDS system_lib
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(auto_follow
  src/auto_planner/wall_follow_ros.cpp
  src/auto_planner/room_line_extractor.cpp
  src/auto_planner/lsd.cpp
)
target_link_libraries(auto_follow ${catkin_LIBRARIES})

add_executable(way_point_note src/resolution_test/way_point_note.cpp)
target_link_libraries(way_point_note ${catkin_LIBRARIES})

add_executable(show_trajectory src/resolution_test/show_traj.cpp)
target_link_libraries(show_trajectory ${catkin_LIBRARIES})

add_executable(odom_compute src/test_straight_ctrller/odom_compute.cpp)
target_link_libraries(odom_compute ${catkin_LIBRARIES})

add_executable(provide_path src/test_straight_ctrller/provide_path.cpp)
target_link_libraries(provide_path ${catkin_LIBRARIES})

add_executable(bim_map2scan src/bim_map_transformation/bim_map2scan.cpp)
target_link_libraries(bim_map2scan yaml-cpp ${catkin_LIBRARIES})

add_executable(send_road_point src/road_point/send_road_point.cpp)
target_link_libraries(send_road_point yaml-cpp ${catkin_LIBRARIES})

add_executable(wall_follow_ros_node src/auto_planner/wall_follow_ros_node.cpp)
target_link_libraries(wall_follow_ros_node auto_follow ${catkin_LIBRARIES})

add_executable(pure_rotater src/auto_planner/pure_rotater.cpp)
target_link_libraries(pure_rotater ${catkin_LIBRARIES})