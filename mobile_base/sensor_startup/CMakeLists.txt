cmake_minimum_required(VERSION 2.8.3)
project(sensor_startup)

add_compile_options(-std=c++11)


find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
  serial
)

find_package(Threads)
find_package(yaml-cpp REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES sensor_startup
#  CATKIN_DEPENDS roscpp rospy sensor_msgs std_msgs tf
#  DEPENDS system_lib
)

include_directories(include lib ${catkin_INCLUDE_DIRS})
link_directories(lib)

# define the library used in sensor communication
add_library(sensor_reader
  src/imu_reader.cpp
  src/kinco/can_application.cpp
  src/kinco/driver_controller.cpp
  src/kinco/driver_controller_ros.cpp
)
target_link_libraries(sensor_reader
  yaml-cpp
  usbcan
  ${catkin_LIBRARIES}
)

# define the imu node
add_executable(imu_reader_node src/imu_reader_node.cpp)
target_link_libraries(imu_reader_node
  sensor_reader
  usbcan
  ${catkin_LIBRARIES}
)

add_executable(abs_encoder src/test/abs_encoder.cpp)
target_link_libraries(abs_encoder usbcan ${catkin_LIBRARIES})

add_executable(stop_motor src/test/stop_motor.cpp)
target_link_libraries(stop_motor ${catkin_LIBRARIES})

add_executable(send_test src/test/send_test.cpp  )
target_link_libraries(send_test ${catkin_LIBRARIES})

add_executable(vary_velo_test src/test/vary_velo_test.cpp  )
target_link_libraries(vary_velo_test ${catkin_LIBRARIES})

add_executable(test_can src/test/test_can.cpp  )
target_link_libraries(test_can usbcan ${catkin_LIBRARIES})


add_executable(walking_demo src/test/walking_demo)
target_link_libraries(walking_demo yaml-cpp ${catkin_LIBRARIES})

add_executable(limit_switch src/limit_switch.cpp)
target_link_libraries(limit_switch ${catkin_LIBRARIES})

add_executable(send_teleop_signal src/send_teleop_signal.cpp)
target_link_libraries(send_teleop_signal yaml-cpp ${catkin_LIBRARIES})

add_executable(driver_controller_ros_node src/kinco/driver_controller_ros_node.cpp)
target_link_libraries(driver_controller_ros_node sensor_reader yaml-cpp ${catkin_LIBRARIES})

add_executable(rot_traj src/test/pure_rot_traj.cpp)
target_link_libraries(rot_traj sensor_reader yaml-cpp ${catkin_LIBRARIES})

add_executable(param_list_test src/test/param_list_test.cpp)
target_link_libraries(param_list_test sensor_reader yaml-cpp ${catkin_LIBRARIES})

add_executable(test_can_com test/test_can_com.cpp)
target_link_libraries(test_can_com sensor_reader ${catkin_LIBRARIES})
