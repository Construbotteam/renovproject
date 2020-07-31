#include "sensor_startup/imu_reader.h"

namespace mobile_base {

ImuReader::ImuReader(ros::NodeHandle& nh, ros::NodeHandle& nh_private)
    : get_first_imu_data_(false) {
  initParam(nh_private);
  initSerial();

  ROS_INFO("IMU port id : %s ", port_id_.c_str());
  ROS_INFO("IMU baud rate : %d", baud_rate_);
  ROS_INFO("IMU frame : %s", imu_frame_.c_str());
  ROS_INFO("IMU publish topic : %s", imu_pub_topic_.c_str());
  ROS_INFO("IMU output frequency : %d", output_freq_);

  imu_pub_ = nh.advertise<sensor_msgs::Imu>(imu_pub_topic_, 100);
  th_ = new std::thread(&ImuReader::readLoop, this);
}

void ImuReader::initSerial() {
  try {
    imu_ser_.setPort(port_id_);
    imu_ser_.setBaudrate(baud_rate_);
    imu_ser_.setParity(serial::parity_none);
    serial::Timeout time_out = serial::Timeout::simpleTimeout(1000);
    imu_ser_.setTimeout(time_out);
    imu_ser_.open();
  } catch (const serial::IOException& e) {
    ROS_ERROR("Unable to open port on %s -> [%s]", port_id_.c_str(), e.what());
    return;
  }

  if (imu_ser_.isOpen()) {
    ROS_INFO("Open serial port %s successfully", port_id_.c_str());

    if (use_request_) {
      imu_ser_.write(cmd_.OUTPUT_FREQUENCY_00HZ,
                     sizeof(cmd_.OUTPUT_FREQUENCY_00HZ));
      ROS_INFO("Set the frequency as 0 Hz");
      ros::Duration(0.01).sleep();
    } else {
      switch (output_freq_) {
        case 5: {
          imu_ser_.write(cmd_.OUTPUT_FREQUENCY_05HZ,
                         sizeof(cmd_.OUTPUT_FREQUENCY_05HZ));
          ROS_INFO("Set the frequency as 5 Hz");
          ros::Duration(0.01).sleep();
          break;
        }
        case 15: {
          imu_ser_.write(cmd_.OUTPUT_FREQUENCY_15HZ,
                         sizeof(cmd_.OUTPUT_FREQUENCY_15HZ));
          ROS_INFO("Set the frequency as 15 Hz");
          ros::Duration(0.01).sleep();
          break;
        }
        case 25: {
          imu_ser_.write(cmd_.OUTPUT_FREQUENCY_25HZ,
                         sizeof(cmd_.OUTPUT_FREQUENCY_25HZ));
          ROS_INFO("Set the frequency as 25 Hz");
          ros::Duration(0.01).sleep();
          break;
        }
        case 35: {
          imu_ser_.write(cmd_.OUTPUT_FREQUENCY_35HZ,
                         sizeof(cmd_.OUTPUT_FREQUENCY_35HZ));
          ROS_INFO("Set the frequency as 35 Hz");
          ros::Duration(0.01).sleep();
          break;
        }
        case 50: {
          imu_ser_.write(cmd_.OUTPUT_FREQUENCY_50HZ,
                         sizeof(cmd_.OUTPUT_FREQUENCY_50HZ));
          ROS_INFO("Set the frequency as 50 Hz");
          ros::Duration(0.01).sleep();
          break;
        }
        case 100: {
          imu_ser_.write(cmd_.OUTPUT_FREQUENCY_100HZ,
                         sizeof(cmd_.OUTPUT_FREQUENCY_100HZ));
          ROS_INFO("Set the frequency as 100 Hz");
          ros::Duration(0.01).sleep();
          break;
        }
        default: {
          ROS_WARN(
              "Incorrect frequency number! Value of Hz should be chosen within "
              "range [5, 15, 25, 35, 50, 100]");
          exit(0);
          break;
        }
      }
    }
  } else {
    ROS_WARN("Open port %s failure!", port_id_.c_str());
    return;
  }
}

void ImuReader::initParam(ros::NodeHandle& nh_private) {
  nh_private.param("port_id", port_id_, std::string("/dev/ttyUSB0"));
  nh_private.param("baud_rate", baud_rate_, 115200);
  nh_private.param("imu_frame", imu_frame_, std::string("base_imu"));
  nh_private.param("imu_pub_topic", imu_pub_topic_, std::string("imu"));
  nh_private.param("use_request", use_request_, true);
  nh_private.param("output_freq", output_freq_, 50);
  nh_private.param("correction_file_addr", correction_file_addr_,
                   std::string(""));

  YAML::Node correction = YAML::LoadFile(correction_file_addr_);
  for (size_t i = 0; i < 3; i++) {
    gyro_cov_(i) = correction["gyro_cov"][i].as<double>();
    angular_velo_cov_(i) = correction["angular_velo_cov"][i].as<double>();
    acc_cov_(i) = correction["acc_cov"][i].as<double>();

    acc_calibrator_.bias_(i) = correction["acc_bias"][i].as<double>();
    gyro_calibrator_.bias_(i) = correction["gyro_bias"][i].as<double>();
  }

  for (size_t i = 0; i < 9; i++) {
    acc_calibrator_.misalignment_(i) =
        correction["acc_misalign"][i].as<double>();
    gyro_calibrator_.misalignment_(i) =
        correction["gyro_misalign"][i].as<double>();
    acc_calibrator_.scale_(i) = correction["acc_scale"][i].as<double>();
    gyro_calibrator_.scale_(i) = correction["gyro_scale"][i].as<double>();
  }

  Eigen::Matrix3d mat_tmp;
  acc_calibrator_.misalignment_.isZero()
      ? acc_calibrator_.misalignment_.setIdentity()
      : mat_tmp;
  acc_calibrator_.scale_.isZero() ? acc_calibrator_.scale_.setIdentity()
                                  : mat_tmp;
  gyro_calibrator_.misalignment_.isZero()
      ? gyro_calibrator_.misalignment_.setIdentity()
      : mat_tmp;
  gyro_calibrator_.scale_.isZero() ? gyro_calibrator_.scale_.setIdentity()
                                   : mat_tmp;

}

void ImuReader::readLoop() {
  ros::Rate rate(output_freq_);
  while (ros::ok()) {
    if (use_request_) {
      imu_ser_.write(cmd_.ASK_FOR_DATA, sizeof(cmd_.ASK_FOR_DATA));
      ros::Duration(0.005).sleep();

      readData();
    } else {
      readData();
    }

    rate.sleep();
  }
}

void ImuReader::readData() {
  std::vector<uint8_t> imu_data;
  std::vector<uint8_t> imu_data_tmp;

  if (validDataHeader()) {
    uint8_t* data_len = new uint8_t;

    imu_ser_.read(data_len, 1);
    imu_ser_.read(imu_data_tmp, data_len[0] - (uint8_t)1);

    imu_data.push_back(DATA_HEADER);
    imu_data.push_back(data_len[0]);

    for (size_t i = 0; i < imu_data_tmp.size(); i++) {
      imu_data.push_back(imu_data_tmp[i]);
    }
    parseData(imu_data);
    delete data_len;
  } else {
    ROS_WARN("spend too much time to find data header");
  }
}

void ImuReader::parseData(const std::vector<uint8_t>& data) {
  if (!validCheckSum(data)) {
    ROS_WARN("Incorrect check sum");
    return;
  }

  if (data.size() == 32) {
    Eigen::VectorXd vec(9);
    Eigen::Vector3d gyro_vec, angular_velo_vec, acc_vec;

    gyro_vec(0) = (double)convert(data[4], data[5], data[6]) / 100.0;
    gyro_vec(1) = (double)convert(data[7], data[8], data[9]) / 100.0;
    gyro_vec(2) = (double)convert(data[10], data[11], data[12]) / 100.0;
    // get orientation in quaternion form
    gyro_vec = gyro_vec / 180.0 * M_PI;

    // attention : computing acceleration should divide 1000
    acc_vec(0) = (double)convert(data[13], data[14], data[15]) / 1000.0;
    acc_vec(1) = (double)convert(data[16], data[17], data[18]) / 1000.0;
    acc_vec(2) = (double)convert(data[19], data[20], data[21]) / 1000.0;

    angular_velo_vec(0) = (double)convert(data[22], data[23], data[24]) / 100.0;
    angular_velo_vec(1) = (double)convert(data[25], data[26], data[27]) / 100.0;
    angular_velo_vec(2) = (double)convert(data[28], data[29], data[30]) / 100.0;
    angular_velo_vec = angular_velo_vec / 180.0 * M_PI;

    sensor_msgs::Imu imu_msg;
    imu_msg.header.frame_id = imu_frame_;
    imu_msg.header.stamp = ros::Time::now();

    calibrate(gyro_vec, gyro_calibrator_);
    imu_msg.orientation = tf::createQuaternionMsgFromRollPitchYaw(
        gyro_vec(0), gyro_vec(1), gyro_vec(2));
    // imu_msg.orientation_covariance = {2e-8, 0, 0, 0, 2e-8, 0, 0, 0, 1.3e-7};
    imu_msg.orientation_covariance = {
        gyro_cov_(0), 0, 0, 0, gyro_cov_(1), 0, 0, 0, gyro_cov_(2)};

    // get angular velocity
    imu_msg.angular_velocity.x = angular_velo_vec(0);
    imu_msg.angular_velocity.y = angular_velo_vec(1);
    imu_msg.angular_velocity.z = angular_velo_vec(2);

    // imu_msg.angular_velocity_covariance = {5e-7, 0, 0, 0,     5e-7,
    //                                        0,    0, 0, 1.3e-7};
    imu_msg.angular_velocity_covariance = {angular_velo_cov_(0), 0, 0, 0,
                                           angular_velo_cov_(1), 0, 0, 0,
                                           angular_velo_cov_(2)};

    // get linear acceleration (value * g)
    calibrate(acc_vec, acc_calibrator_);
    imu_msg.linear_acceleration.x = acc_vec(0) * GRAVITY;
    imu_msg.linear_acceleration.y = acc_vec(1) * GRAVITY;
    imu_msg.linear_acceleration.z = acc_vec(2) * GRAVITY;

    // imu_msg.linear_acceleration_covariance = {3.511e-5, 0, 0,
    // 0,       3.339e-5,
    //                                           0,        0, 0, 3.989e-5};
    imu_msg.linear_acceleration_covariance = {
        acc_cov_(0), 0, 0, 0, acc_cov_(1), 0, 0, 0, acc_cov_(2)};

    imu_pub_.publish(imu_msg);
  } else {
    ROS_WARN("imu data size = %d is not correct", (int)data.size());
  }
}

int ImuReader::convert(const uint8_t& a, const uint8_t& b, const uint8_t& c) {
  std::stringstream int2str, str2int;
  int2str << (int)a % 16 << (int)b / 16 << (int)b % 16 << (int)c / 16
          << (int)c % 16;
  int int_result;
  std::string str_result;
  int2str >> str_result;
  str2int << str_result;
  str2int >> int_result;

  if (1 == (int)a / 16) {
    int_result = -int_result;
  } else {
    int_result = abs(int_result);
  }

  return int_result;
}

bool ImuReader::validDataHeader() {
  std::vector<uint8_t> imu_one_byte;

  ros::Time start = ros::Time::now();
  while (true) {
    imu_one_byte.clear();
    imu_ser_.read(imu_one_byte, 1);

    if (DATA_HEADER == imu_one_byte[0]) {
      return true;
    }

    if ((ros::Time::now() - start).toSec() > 1.0 / output_freq_) {
      return false;
    }
  }
}

bool ImuReader::validCheckSum(const std::vector<uint8_t>& data) {
  int check_sum = 0;
  for (size_t i = 1; i < data.size() - 1; i++) {
    check_sum += data[i];
  }

  if ((check_sum & 0xff) == data.back()) {
    return true;
  } else {
    ROS_WARN("The check sum is incorrect");
  }
}

void ImuReader::calibrate(Eigen::Vector3d& data, const Calibrator& calibrator) {
  data =
      calibrator.misalignment_ * calibrator.scale_ * (data - calibrator.bias_);
}

}  // namespace mobile_base
