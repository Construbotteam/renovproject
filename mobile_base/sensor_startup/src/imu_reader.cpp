#include "sensor_startup/imu_reader.h"
#include <fstream>

using mobile_base::ImuReader;

ImuReader::ImuReader(ros::NodeHandle& nh, ros::NodeHandle& nh_private) {
  ParamInit(nh_private);
  SerialInit();

  ROS_INFO("port_id : %s ", port_id.c_str());
  ROS_INFO("baud_rate : %d", baud_rate);
  ROS_INFO("imu_frame_id : %s", imu_frame_id.c_str());
  ROS_INFO("imu_pub_topic : %s", imu_pub_topic.c_str());

  imu_pub = nh.advertise<sensor_msgs::Imu>(imu_pub_topic, 100);
  imu_timer = nh.createTimer(ros::Duration(1.0 / output_freq),
                             &ImuReader::ReadDataCallback, this);
}

void ImuReader::TestReader() {
  ROS_INFO("data num in buffer is : %d ", (int)imu_ser.available());
  std::vector<uint8_t> data;
  data.clear();

  imu_ser.read(data, imu_ser.available());
  std::ofstream out("/home/glh/Desktop/test_imu.txt", std::ios::app);
  out << ros::Time::now().toSec() << " : ";
  for (size_t i = 0; i < data.size(); i++) {
    out << std::hex << "0x" << (int)data[i] << "  ";
  }
  out << std::endl;
  out.close();
}

void ImuReader::SerialInit() {
  // get imu port id from ros parameter lsit
  /*
  ros::NodeHandle nh_getport("search_port");
  nh_getport.setParam("imu_port_ok_fl", 2);
  int imu_port_ok_flag = false;
  while (true) {
    if (!nh_getport.getParam("imu_port_ok_flag", imu_port_ok_flag)) {
      std::cout << "imu port id flag is : " << imu_port_ok_flag << " not ok!!"
  << std::endl; ros::Duration(0.5).sleep(); imu_port_ok_flag = false; continue;
    }
    if (imu_port_ok_flag == 1 && nh_getport.getParam("imu_port", port_id)) {
      ROS_INFO("get imu port id : %s", port_id.c_str());
      break;
    }
  }
  */

  try {
    imu_ser.setPort(port_id);
    imu_ser.setBaudrate(baud_rate);
    imu_ser.setParity(serial::parity_none);
    serial::Timeout time_out = serial::Timeout::simpleTimeout(1000);
    imu_ser.setTimeout(time_out);
    imu_ser.open();
  } catch (const serial::IOException& e) {
    ROS_ERROR("Unable to open port on %s -> [%s]", port_id.c_str(), e.what());
    return;
  }
  if (imu_ser.isOpen()) {
    ROS_INFO("Open serial port successfully");

    if (use_request) {
      imu_ser.write(cmd.OUTPUT_FREQUENCY_00HZ,
                    sizeof(cmd.OUTPUT_FREQUENCY_00HZ));
      ROS_INFO("set the frequency as 0 Hz");
      ros::Duration(0.01).sleep();
    } else {
      switch (output_freq) {
        case 5: {
          imu_ser.write(cmd.OUTPUT_FREQUENCY_05HZ,
                        sizeof(cmd.OUTPUT_FREQUENCY_05HZ));
          ROS_INFO("set the frequency as 5 Hz");
          ros::Duration(0.01).sleep();
          break;
        }
        case 15: {
          imu_ser.write(cmd.OUTPUT_FREQUENCY_15HZ,
                        sizeof(cmd.OUTPUT_FREQUENCY_15HZ));
          ROS_INFO("set the frequency as 15 Hz");
          ros::Duration(0.01).sleep();
          break;
        }
        case 25: {
          imu_ser.write(cmd.OUTPUT_FREQUENCY_25HZ,
                        sizeof(cmd.OUTPUT_FREQUENCY_25HZ));
          ROS_INFO("set the frequency as 25 Hz");
          ros::Duration(0.01).sleep();
          break;
        }
        case 35: {
          imu_ser.write(cmd.OUTPUT_FREQUENCY_35HZ,
                        sizeof(cmd.OUTPUT_FREQUENCY_35HZ));
          ROS_INFO("set the frequency as 35 Hz");
          ros::Duration(0.01).sleep();
          break;
        }
        case 50: {
          imu_ser.write(cmd.OUTPUT_FREQUENCY_50HZ,
                        sizeof(cmd.OUTPUT_FREQUENCY_50HZ));
          ROS_INFO("set the frequency as 50 Hz");
          ros::Duration(0.01).sleep();
          break;
        }
        case 100: {
          imu_ser.write(cmd.OUTPUT_FREQUENCY_100HZ,
                        sizeof(cmd.OUTPUT_FREQUENCY_100HZ));
          ROS_INFO("set the frequency as 100 Hz");
          ros::Duration(0.01).sleep();
          break;
        }
        default: {
          ROS_WARN("incorrect frequency number");
          exit(1);
          break;
        }
      }
    }
    /*(
    std::cout << "I got reply as : ";
    for (size_t i = 0; i < imu_reply.size(); i++) {
      std::cout << std::hex << "0x" << (int)imu_reply[i] << "  ";
    }
    std::cout << std::endl;
    */
  } else {
    ROS_WARN("Port opening failed!");
    return;
  }
}

void ImuReader::ParamInit(ros::NodeHandle& nh_private) {
  if (!nh_private.getParam("port_id", port_id)) {
    port_id = "/dev/ttyUSB0";
  }
  if (!nh_private.getParam("baud_rate", baud_rate)) {
    baud_rate = 115200;
  }
  if (!nh_private.getParam("imu_frame_id", imu_frame_id)) {
    imu_frame_id = "base_imu";
  }
  if (!nh_private.getParam("imu_pub_topic", imu_pub_topic)) {
    imu_pub_topic = "imu";
  }
  if (!nh_private.getParam("use_request", use_request)) {
    use_request = true;
  }
  if (!nh_private.getParam("output_freq", output_freq)) {
    output_freq = 50;
  }
  if (!nh_private.getParam("use_debug", use_debug)) {
    use_debug = false;
  }
}

void ImuReader::ReadDataCallback(const ros::TimerEvent&) {
  if (use_request) {
    imu_ser.write(cmd.ASK_FOR_DATA, sizeof(cmd.ASK_FOR_DATA));
    // std::cout << "============" << std::endl;
    // std::cout << "cmd data : " << std::endl;
    // for (size_t i = 0; i < sizeof(cmd.ASK_FOR_DATA); i++) {
    //   std::cout << std::hex << "0x" << (int)cmd.ASK_FOR_DATA[i] << "  ";
    // }
    // std::cout << "============" << std::endl;
    // std::cout << std::endl;

    ros::Duration(0.01).sleep();

    std::vector<uint8_t> imu_data;
    std::vector<uint8_t> empty_data;
    if (imu_ser.available()) {
      imu_data.clear();
      imu_ser.read(imu_data, 32);

      // std::cout << "============" << std::endl;
      // std::cout << "imu data : " << std::endl;
      // for (size_t i = 0; i < imu_data.size(); i++) {
      //   std::cout << std::hex << "0x" << (int)imu_data[i] << "  ";
      // }
      // std::cout << "============" << std::endl;
      // std::cout << std::endl;

      imu_ser.read(empty_data, imu_ser.available());
      DataParser(imu_data);
    }
  } else {
    std::vector<uint8_t> imu_data;
    std::vector<uint8_t> imu_data_tmp;
    while (true) {
    /*
      if (imu_ser.available() < 32) {
        ROS_WARN("data in buffer is : %d which is not enough",
                 (int)imu_ser.available());
        break;
      }
      */
      imu_ser.read(imu_data_tmp, 1);
      if (0x68 == imu_data_tmp[0]) {
        imu_data_tmp.clear();
        imu_ser.read(imu_data_tmp, 1);
        if (0x1f == imu_data_tmp[0]) {
          imu_data_tmp.clear();
          imu_ser.read(imu_data_tmp, 30);
          imu_data.push_back(0x68);
          imu_data.push_back(0x1f);
          // imu_data.push_back(0x00);
          // imu_data.push_back(0x84);
          for (size_t i = 0; i < imu_data_tmp.size(); i++) {
            imu_data.push_back(imu_data_tmp[i]);
          }

          imu_ser.flushInput();
          DataParser(imu_data);
          break;
        }
      } else {
        imu_data_tmp.clear();
      }
    }

    /*
    std::cout << "==========================" << std::endl;
    std::cout << "raw hex i got is : ";
    for (size_t i = 0; i < imu_data.size(); i++) {
      std::cout << std::hex << "0x" << (int)imu_data[i] << "  ";
    }
    std::cout << std::endl;
    */
  }
}

void ImuReader::DataParser(const std::vector<uint8_t>& data) {
  // vec = {roll/pitch/yaw, acc_x/_y/_z, angular_vx/_vy/_vz}
  Eigen::VectorXd vec(9);
  if (data.size() == 32) {
    vec(0) = (double)Converter(data[4], data[5], data[6]) / 100.0;
    vec(1) = (double)Converter(data[7], data[8], data[9]) / 100.0;
    vec(2) = (double)Converter(data[10], data[11], data[12]) / 100.0;

    if (use_debug) {
      ROS_INFO("euler angle in degree : %.2f, %.2f, %.2f", vec(0), vec(1),
               vec(2));
    }

    // attention : computing acceleration should divide 1000
    vec(3) = (double)Converter(data[13], data[14], data[15]) / 1000.0;
    vec(4) = (double)Converter(data[16], data[17], data[18]) / 1000.0;
    vec(5) = (double)Converter(data[19], data[20], data[21]) / 1000.0;

    vec(6) = (double)Converter(data[22], data[23], data[24]) / 100.0;
    vec(7) = (double)Converter(data[25], data[26], data[27]) / 100.0;
    vec(8) = (double)Converter(data[28], data[29], data[30]) / 100.0;

    sensor_msgs::Imu imu_msg;
    imu_msg.header.frame_id = imu_frame_id;
    imu_msg.header.stamp = ros::Time::now();

    // get orientation in quaternion form
    vec(0) = vec(0) / 180.0 * M_PI;
    vec(1) = vec(1) / 180.0 * M_PI;
    vec(2) = vec(2) / 180.0 * M_PI;
    Eigen::Quaterniond q = Eigen::AngleAxisd(vec(0), Eigen::Vector3d::UnitX()) *
                           Eigen::AngleAxisd(vec(1), Eigen::Vector3d::UnitY()) *
                           Eigen::AngleAxisd(vec(2), Eigen::Vector3d::UnitZ());
    imu_msg.orientation.x = q.x();
    imu_msg.orientation.y = q.y();
    imu_msg.orientation.z = q.z();
    imu_msg.orientation.w = q.w();
    // imu_msg.orientation_covariance = {1e6, 0, 0, 0, 1e6, 0, 0, 0, 1e-6};
    imu_msg.orientation_covariance = {2e-8, 0, 0, 0, 2e-8, 0, 0, 0, 1.3e-7};

    // get angular velocity
    imu_msg.angular_velocity.x = vec(6) / 180.0 * M_PI;
    imu_msg.angular_velocity.y = vec(7) / 180.0 * M_PI;
    imu_msg.angular_velocity.z = vec(8) / 180.0 * M_PI;
    imu_msg.angular_velocity_covariance = {5e-7, 0, 0, 0,     5e-7,
                                           0,    0, 0, 1.3e-7};

    // get linear acceleration (value * g)
    imu_msg.linear_acceleration.x = vec(3) * GRAVITY;
    imu_msg.linear_acceleration.y = vec(4) * GRAVITY;
    imu_msg.linear_acceleration.z = vec(5) * GRAVITY;
    imu_msg.linear_acceleration_covariance = {3.511e-5, 0, 0, 0,       3.339e-5,
                                              0,        0, 0, 3.989e-5};

    imu_pub.publish(imu_msg);
  } else {
    ROS_WARN("imu data size = %d is not correct", (int)data.size());
  }
}

int ImuReader::Converter(const uint8_t a, const uint8_t b, const uint8_t c) {
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
