#include "sensor_startup/imu_reader.h"

using mobile_base::ImuReader;

int main(int argc, char** argv) {
  ros::init(argc, argv, "imu_reader");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  ImuReader imu_reader(nh, nh_private);
  ros::spin();
  return 0;
}
