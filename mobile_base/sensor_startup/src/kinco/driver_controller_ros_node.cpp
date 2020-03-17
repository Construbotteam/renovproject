#include "sensor_startup/kinco/driver_controller_ros.h"

using mobile_base::DriverControllerROS;

int main(int argc, char** argv) {
  ros::init(argc, argv, "driver_controller_ros_node");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  DriverControllerROS driver_controller(nh, nh_private);
  ros::spin();

  return 0;
}
