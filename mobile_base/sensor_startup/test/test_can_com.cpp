#include "ros/ros.h"
#include "sensor_startup/kinco/can_application.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "test_can_com");
  ros::NodeHandle nh;

  mobile_base::CanApplication can_app;
  std::string can_config_addr;
  int driver_num, pdo_num;
  int read_buffer_length, loop_limit;
  double rate;
  nh.param("can_config_addr", can_config_addr, std::string(""));
  nh.param("read_buffer_length", read_buffer_length, 200);
  nh.param("driver_num", driver_num, 8);
  nh.param("pdo_num", pdo_num, 3);
  nh.param("rate", rate, 10.0);
  nh.param("loop_limit", loop_limit, 10);

  can_app.ActivateCAN(can_config_addr);
  PVCI_CAN_OBJ obj = can_app.GetVciObject(1, 0x80);
  obj->DataLen = 0;

  int loop_count = 0;
  bool buffer_enough = true;
  ros::Rate r(rate);
  while (ros::ok()) {
    if (loop_count > loop_limit) {
      break;
    }

    can_app.SendCommand(obj, 1);

    PVCI_CAN_OBJ data_obj;
    can_app.GetData(data_obj, read_buffer_length);

    int count = 0;
    for (size_t i = 0; i < read_buffer_length; i++) {
      if (data_obj[i].ID != 0x00) {
        std::cout << std::hex << "0x" << (int)data_obj[i].ID << "  ";
        count++;
      }
    }
    std::cout << count << std::endl;
    if (count != pdo_num * driver_num) {
      buffer_enough = buffer_enough && false;
    }
    loop_count++;

    ros::spinOnce();
    r.sleep();
  }
  if (!buffer_enough) {
    std::cout << std::boolalpha << buffer_enough << std::endl;
  }

  can_app.CloseCAN();
  ROS_INFO("Close CAN");
  return 0;
}
