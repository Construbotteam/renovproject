#include "sensor_startup/imu_reader.h"

#include <fstream>

using mobile_base::ImuReader;

ImuReader::ImuReader(ros::NodeHandle& nh, ros::NodeHandle& nh_private) {
  paramInit(nh_private);
  serialInit();

  imu_pub_ = nh.advertise<sensor_msgs::Imu>(imu_pub_topic_, 100);
  imu_timer_ = nh.createTimer(ros::Duration(1.0 / output_freq_),
                              &ImuReader::readDataCallback, this);
}

void ImuReader::serialInit() {
  // initialize the serial port and catch potential exceptions
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
    ROS_INFO("Open serial port successfully");
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
          ROS_WARN("Incorrect frequency number");
          exit(1);
          break;
        }
      }
    }
  } else {
    ROS_WARN("Open port failure!");
    return;
  }
}

void ImuReader::paramInit(ros::NodeHandle& nh_private) {
  nh_private.param("port_id_", port_id_, std::string("/dev/ttyUSB0"));
  nh_private.param("baud_rate", baud_rate_, 115200);
  nh_private.param("imu_frame_id", imu_frame_id_, std::string("imu_link"));
  nh_private.param("imu_pub_topic", imu_pub_topic_, std::string("imu"));
  nh_private.param("use_request", use_request_, true);
  nh_private.param("output_freq", output_freq_, 50);
  nh_private.param("use_debug", use_debug_, false);
}

void ImuReader::readDataCallback(const ros::TimerEvent&) {
  if (use_request_) {
    imu_ser_.write(cmd_.ASK_FOR_DATA, sizeof(cmd_.ASK_FOR_DATA));

    ros::Duration(0.01).sleep();

    std::vector<uint8_t> imu_data;
    std::vector<uint8_t> empty_data;
    if (imu_ser_.available()) {
      imu_data.clear();
      imu_ser_.read(imu_data, 32);

      imu_ser_.read(empty_data, imu_ser_.available());
      dataParser(imu_data);
    }
  } else {
    std::vector<uint8_t> imu_data;
    std::vector<uint8_t> imu_data_tmp;
    while (true) {
      if (imu_ser_.available() < 32) {
        ROS_WARN("Data in buffer is : %d which is not enough",
                 (int)imu_ser_.available());
        break;
      }
      imu_ser_.read(imu_data_tmp, 1);
      if (0x68 == imu_data_tmp[0]) {
        imu_data_tmp.clear();
        imu_ser_.read(imu_data_tmp, 1);
        if (0x1f == imu_data_tmp[0]) {
          imu_data_tmp.clear();
          imu_ser_.read(imu_data_tmp, 30);
          imu_data.push_back(0x68);
          imu_data.push_back(0x1f);
          // imu_data.push_back(0x00);
          // imu_data.push_back(0x84);
          for (size_t i = 0; i < imu_data_tmp.size(); i++) {
            imu_data.push_back(imu_data_tmp[i]);
          }

          imu_ser_.flushInput();
          dataParser(imu_data);
          break;
        }
      } else {
        imu_data_tmp.clear();
      }
    }
  }
}

void ImuReader::dataParser(const std::vector<uint8_t>& data) {
  // vec = {roll/pitch/yaw, acc_x/_y/_z, angular_vx/_vy/_vz}
  Eigen::VectorXd vec(9);
  if (data.size() == 32) {
    vec(0) = (double)converter(data[4], data[5], data[6]) / 100.0;
    vec(1) = (double)converter(data[7], data[8], data[9]) / 100.0;
    vec(2) = (double)converter(data[10], data[11], data[12]) / 100.0;

    if (use_debug_) {
      ROS_INFO("Euler angle in degree : %.2f, %.2f, %.2f", vec(0), vec(1),
               vec(2));
    }

    // attention : computing acceleration should divide 1000
    vec(3) = (double)converter(data[13], data[14], data[15]) / 1000.0;
    vec(4) = (double)converter(data[16], data[17], data[18]) / 1000.0;
    vec(5) = (double)converter(data[19], data[20], data[21]) / 1000.0;

    vec(6) = (double)converter(data[22], data[23], data[24]) / 100.0;
    vec(7) = (double)converter(data[25], data[26], data[27]) / 100.0;
    vec(8) = (double)converter(data[28], data[29], data[30]) / 100.0;

    sensor_msgs::Imu imu_msg;
    imu_msg.header.frame_id = imu_frame_id_;
    imu_msg.header.stamp = ros::Time::now();

    // get orientation in quaternion form
    vec(0) = angles::from_degrees(vec(0));
    vec(1) = angles::from_degrees(vec(1));
    vec(2) = angles::from_degrees(vec(2));

    imu_msg.orientation =
        tf::createQuaternionMsgFromRollPitchYaw(vec(0), vec(1), vec(2));
    // imu_msg.orientation_covariance = {1e6, 0, 0, 0, 1e6, 0, 0, 0, 1e-6};
    imu_msg.orientation_covariance = {2e-8, 0, 0, 0, 2e-8, 0, 0, 0, 1.3e-7};

    // get angular velocity

    imu_msg.angular_velocity.x = angles::from_degrees(vec(6));
    imu_msg.angular_velocity.y = angles::from_degrees(vec(7));
    imu_msg.angular_velocity.z = angles::from_degrees(vec(8));
    imu_msg.angular_velocity_covariance = {5e-7, 0, 0, 0,     5e-7,
                                           0,    0, 0, 1.3e-7};

    // get linear acceleration (value * g)
    imu_msg.linear_acceleration.x = vec(3) * GRAVITY;
    imu_msg.linear_acceleration.y = vec(4) * GRAVITY;
    imu_msg.linear_acceleration.z = vec(5) * GRAVITY;
    imu_msg.linear_acceleration_covariance = {3.511e-5, 0, 0, 0,       3.339e-5,
                                              0,        0, 0, 3.989e-5};

    imu_pub_.publish(imu_msg);
  } else {
    ROS_WARN("Imu data size = %d is not correct", (int)data.size());
  }
}

int ImuReader::converter(const uint8_t a, const uint8_t b, const uint8_t c) {
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
