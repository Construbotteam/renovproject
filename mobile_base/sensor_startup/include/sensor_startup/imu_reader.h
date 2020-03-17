#ifndef IMU_READER_H
#define IMU_READER_H

#include <eigen3/Eigen/Dense>
#include <sstream>
#include <string>

#include "angles/angles.h"
#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "serial/serial.h"
#include "tf/tf.h"

#define GRAVITY 9.80665

// considering neccessity of null drift compensation

namespace mobile_base {

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
  ~ImuReader() {}
  /**
   * @brief initialize serial port by pre-set parameters
   */
  void serialInit();
  /**
   * @brief Initialize parameters
   * @param nh_private Private node handle used to read params from ros param
   * list
   */
  void paramInit(ros::NodeHandle& nh_private);
  /**
   * @brief Loop of reading data from imu 
   * @param rostimer
   */
  void readDataCallback(const ros::TimerEvent&);
  /**
   * @brief Transform data from hex to readable dec and publish them
   * @param data hex data read from imu buffer
   */
  void dataParser(const std::vector<uint8_t>& data);
  /**
   * @brief Transform three-byte data to integer 
   * @return Readable angular parameters 
   */
  int converter(const uint8_t a, const uint8_t b, const uint8_t c);

 private:
  std::string port_id_;
  int baud_rate_;
  std::string imu_frame_id_;
  std::string imu_pub_topic_;
  bool use_request_;
  int output_freq_;
  bool use_debug_;

  ImuCommand cmd_;
  serial::Serial imu_ser_;
  ros::Publisher imu_pub_;
  ros::Timer imu_timer_;
};  // class ImuReader

}  // namespace mobile_base

#endif
