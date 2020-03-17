#include <iostream>
#include <string>

#include "ros/ros.h"
#include "serial/serial.h"

serial::Serial switch_ser;
std::string port = "/dev/ttyUSB0";
int baud = 9600;

uint8_t ASK_FOR_DATA[8] = {0xfe, 0x02, 0x00, 0x00, 0x00, 0x06, 0xec, 0x07}; 

bool InitPort() {
  try {
    switch_ser.setPort(port);
    switch_ser.setBaudrate(baud);
    switch_ser.setParity(serial::parity_none);
    serial::Timeout time_out = serial::Timeout::simpleTimeout(1000);
    switch_ser.setTimeout(time_out);
    switch_ser.open();
  } catch (const serial::IOException& ex) {
    ROS_ERROR("open port failure !!!: %s", ex.what());
    return false;
  }
  return switch_ser.isOpen();
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "limit_switch");
  ros::NodeHandle nh;

  std::vector<uint8_t> cmd;
  for (size_t i = 0; i < 8; i++) {
    cmd.push_back(ASK_FOR_DATA[i]);
  }
  if (InitPort()) {
    while (ros::ok()) {
      // switch_ser.write(cmd); 
      switch_ser.write(ASK_FOR_DATA, sizeof(ASK_FOR_DATA));

      std::vector<uint8_t> data;
      data.clear();
      if (switch_ser.available()) {
        switch_ser.read(data, switch_ser.available());
      } else {
        ROS_WARN("nothing in buffer after sending command");
      }
      
      std::cout << "Data i get is : ";
      for (size_t i = 0; i < data.size(); i++) {
        std::cout << std::hex << "0x" << (int)data[i] << "  ";
      }
      std::cout << std::endl;
    }
  }
  return 0;

}