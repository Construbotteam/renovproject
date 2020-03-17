#include <string>
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "yaml-cpp/yaml.h"

void ReadFile(const std::string& addr, double& wheel_dis_len,
              double& wheel_dis_wid) {
  YAML::Node config = YAML::LoadFile(addr);
  wheel_dis_len = config["front_rear_track"].as<double>();
  wheel_dis_wid = config["left_right_track"].as<double>();
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "init_scan_match");
  ros::NodeHandle nh;

  ros::Publisher control_signal_pub =
      nh.advertise<sensor_msgs::JointState>("cmd_base_joint", 10);

  double wheel_dis_len, wheel_dis_wid;
  std::string addr =
      "/data/ros/yue_wk_2019/src/renov_robot/sensor_startup/config/kinco/"
      "driver_config.yaml";
  YAML::Node config = YAML::LoadFile(addr);
  wheel_dis_len = config["front_rear_track"].as<double>();
  wheel_dis_wid = config["left_right_track"].as<double>();

  sensor_msgs::JointState control_signal;

  control_signal.header.frame_id = "motor";
  control_signal.name.resize(8);
  control_signal.name = {"front_left_walking",  "front_right_walking",
                         "rear_left_walking",   "rear_right_walking",
                         "front_left_steering", "front_right_steering",
                         "rear_left_steering",  "rear_right_steering"};
  control_signal.position.resize(8);
  control_signal.velocity.resize(8);

  double hypotenuse = sqrt(pow(wheel_dis_len, 2) + pow(wheel_dis_wid, 2));
  double r = 0.5 * hypotenuse;

  double sa, vl;
  sa = fabs(atan(wheel_dis_len / wheel_dis_wid));

  double az;
  double duration_t;
  if (!ros::param::get("az", az)) {
    az = 1.88;
  }
  if (!ros::param::get("duration_t", duration_t) || duration_t < 2.0) {
    duration_t = 15.0;
  }
  ROS_INFO("duration time: %.4fs", duration_t);
  ROS_INFO("angular velocity is : %.4f", az);
  vl = az * r / (double)0.15;

  ros::Rate rate(10);
  double start_t = ros::Time::now().toSec();
  while (ros::ok()) {
    double cur_t = ros::Time::now().toSec();
    double delta_t = fabs(cur_t - start_t);
    if (delta_t < 2.0) {
      control_signal.velocity = {0, 0, 0, 0, 0, 0, 0, 0};
      control_signal.position = {0, 0, 0, 0, -sa, sa, sa, -sa}; 
    } else if (delta_t < duration_t) {
      control_signal.velocity = {-vl, vl, -vl, vl, 0, 0, 0, 0};
      control_signal.position = {0, 0, 0, 0, -sa, sa, sa, -sa};
    } else if (delta_t < duration_t + 2.0) {
      control_signal.velocity = {0, 0, 0, 0, 0, 0, 0, 0};
      control_signal.position = {0, 0, 0, 0, 0, 0, 0, 0};
    } else {
      ROS_INFO("init scan match finish");
      exit(0);
    }
    control_signal_pub.publish(control_signal);

    ros::spinOnce();
    rate.sleep();
  }
}
