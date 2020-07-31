#ifndef IMU_READER_H
#define IMU_READER_H

#include <yaml-cpp/yaml.h>

#include <eigen3/Eigen/Dense>
#include <fstream>
#include <sstream>
#include <string>
#include <thread>

#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "serial/serial.h"
#include "tf/tf.h"

#define GRAVITY 9.80665
#define DATA_HEADER 0x68

// considering neccessity of null drift compensation

namespace mobile_base {

struct Calibrator {
  Calibrator() {
    misalignment_.setIdentity();
    scale_.setIdentity();
    bias_.setZero();
  }
  Eigen::Matrix3d misalignment_;
  Eigen::Matrix3d scale_;
  Eigen::Vector3d bias_;
};  // struct Calibrator

struct ImuCommand {
  uint8_t OUTPUT_FREQUENCY_00HZ[6] = {0x68, 0x05, 0x00, 0x0c, 0x00, 0x11};
  uint8_t OUTPUT_FREQUENCY_05HZ[6] = {0x68, 0x05, 0x00, 0x0c, 0x01, 0x12};
  uint8_t OUTPUT_FREQUENCY_15HZ[6] = {0x68, 0x05, 0x00, 0x0c, 0x02, 0x13};
  uint8_t OUTPUT_FREQUENCY_25HZ[6] = {0x68, 0x05, 0x00, 0x0c, 0x03, 0x14};
  uint8_t OUTPUT_FREQUENCY_35HZ[6] = {0x68, 0x05, 0x00, 0x0c, 0x04, 0x15};
  uint8_t OUTPUT_FREQUENCY_50HZ[6] = {0x68, 0x05, 0x00, 0x0c, 0x05, 0x16};
  uint8_t OUTPUT_FREQUENCY_100HZ[6] = {0x68, 0x05, 0x00, 0x0c, 0x06, 0x17};
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
  ~ImuReader() {
    if (th_) {
      delete th_;
    }
  }
  void initSerial();
  void initParam(ros::NodeHandle& nh_private);

  void readLoop();
  void readData();

  void parseData(const std::vector<uint8_t>& data);
  int convert(const uint8_t& a, const uint8_t& b, const uint8_t& c);
  bool validDataHeader();
  bool validCheckSum(const std::vector<uint8_t>& data);
  void calibrate(Eigen::Vector3d& data, const Calibrator& calibrator);

 private:
  std::string port_id_, correction_file_addr_;
  int baud_rate_, output_freq_;
  std::string imu_frame_, imu_pub_topic_;
  bool use_request_, get_first_imu_data_;

  ImuCommand cmd_;
  serial::Serial imu_ser_;
  ros::Publisher imu_pub_;
  ros::Timer imu_timer_;

  std::thread* th_;

  Calibrator acc_calibrator_, gyro_calibrator_;
  Eigen::Vector3d gyro_cov_, angular_velo_cov_, acc_cov_;

};  // class ImuReader

}  // namespace mobile_base

#endif
