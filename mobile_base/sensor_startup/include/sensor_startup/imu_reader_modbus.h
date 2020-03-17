#ifndef IMU_READER_H
#define IMU_READER_H

#include <string>
#include <sstream>
#include <eigen3/Eigen/Dense>

#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "serial/serial.h"

#define GRAVITY 9.80665

// considering neccessity of null drift compensation 

namespace mobile_base {

struct ImuCommand {
  uint8_t OUTPUT_FREQUENCY_00HZ[6] = {0x68, 0x05, 0x00, 0x0c, 0x00, 0x11};
  uint8_t OUTPUT_FREQUENCY_05HZ[6] = {0x68, 0x05, 0x00, 0x0c, 0x00, 0x12};
  uint8_t OUTPUT_FREQUENCY_15HZ[6] = {0x68, 0x05, 0x00, 0x0c, 0x00, 0x13};
  uint8_t OUTPUT_FREQUENCY_25HZ[6] = {0x68, 0x05, 0x00, 0x0c, 0x03, 0x14};
  uint8_t OUTPUT_FREQUENCY_35HZ[6] = {0x68, 0x05, 0x00, 0x0c, 0x00, 0x15};
  uint8_t OUTPUT_FREQUENCY_50HZ[6] = {0x68, 0x05, 0x00, 0x0c, 0x00, 0x16};
  uint8_t OUTPUT_FREQUENCY_100HZ[6] = {0x68, 0x05, 0x00, 0x0c, 0x00, 0x17};
  uint8_t ASK_FOR_DATA[5] = {0x68, 0x04, 0x00, 0x04, 0x08};

  uint8_t BAUDRATE_9600[6] = {0x68, 0x05, 0x00, 0x0b, 0x02, 0x12};
  uint8_t BAUDRATE_19200[6] = {0x68, 0x05, 0x00, 0x0b, 0x02, 0x13};
  uint8_t BAUDRATE_38400[6] = {0x68, 0x05, 0x00, 0x0b, 0x02, 0x14};
  uint8_t BAUDRATE_115200[6] = {0x68, 0x05, 0x00, 0x0b, 0x02, 0x15};

  uint8_t RESET_EULER_ANGLE[5] = {0x68, 0x04, 0x00, 0x28, 0x0c};
};  // struct ImuCommand

class ImuReader {
 public:
  ImuReader(ros::NodeHandle& nh, ros::NodeHandle& nh_private);
  ~ImuReader() {}
  void SerialInit();
  void ParamInit(ros::NodeHandle& nh_private);
  void Setup();
  void ReadData();
  void DataParser(const std::vector<uint8_t>& data);
  int Converter(const uint8_t a, const uint8_t b, const uint8_t c);
 private:
  /* PARAMETERS */
  std::string port_id;
  int baud_rate;
  std::string imu_frame_id;
  std::string imu_pub_topic;
  bool use_request;
  int output_freq;
  bool use_debug;

  ImuCommand cmd;
  serial::Serial imu_ser;
  ros::NodeHandle n_private;
  ros::NodeHandle nh;
  ros::Publisher imu_pub;
};  // class ImuReader

}  // namespace mobile_base




#endif
