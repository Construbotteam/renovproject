#include <iostream>

#include <yaml-cpp/yaml.h>

#include "geometry_msgs/Twist.h"
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"

class Teleop {
 public:
  Teleop(ros::NodeHandle nh);
  virtual ~Teleop() {}
  void ReadFile(const std::string& file_address);
  void SendTeleopSignal(const geometry_msgs::TwistConstPtr& twist_msg);

 private:
  double wheel_dis_len;
  double wheel_dis_wid;

  double pre_steer_angle;

  ros::Publisher control_signal_pub;
  sensor_msgs::JointState control_signal;
};

Teleop::Teleop(ros::NodeHandle nh) {
  // ReadFile(
  //     "/home/renov_robot/catkin_ws/src/renov_robot/sensor_startup/config/"
  //     "motor_config.yaml");
  ReadFile("/data/ros/yue_wk_2019/src/renov_robot/sensor_startup/config/motor_config.yaml");
  control_signal_pub =
      nh.advertise<sensor_msgs::JointState>("cmd_base_joint", 100);

  control_signal.header.frame_id = "motor";
  control_signal.name.resize(8);
  control_signal.name = {"front_left_walking",  "front_right_walking",
                         "rear_left_walking",   "rear_right_walking",
                         "front_left_steering", "front_right_steering",
                         "rear_left_steering",  "rear_right_steering"};
  control_signal.position.resize(8);
  control_signal.velocity.resize(8);

  pre_steer_angle = 0.0;
}

void Teleop::ReadFile(const std::string& file_address) {
  YAML::Node config = YAML::LoadFile(file_address);
  wheel_dis_len = config["front_rear_track"].as<double>();
  wheel_dis_wid = config["left_right_track"].as<double>();
}

void Teleop::SendTeleopSignal(const geometry_msgs::TwistConstPtr& twist_msg) {
  double vx = twist_msg->linear.x;
  double az = twist_msg->angular.z;

  if (vx != 0 && az == 0) {
    control_signal.velocity = {vx, vx, vx, vx, 0, 0, 0, 0};
    control_signal.position = {0, 0, 0, 0, 0, 0, 0, 0};
    control_signal.header.stamp = ros::Time::now();
    control_signal_pub.publish(control_signal);

    pre_steer_angle = 0;
  } else if (vx == 0 && az != 0) {
    double hypotenuse = sqrt(pow(wheel_dis_len, 2) + pow(wheel_dis_wid, 2));
    double r = 0.5 * hypotenuse;

    double steer_angle = fabs(atan(wheel_dis_len / wheel_dis_wid));
    std::vector<double> velocity;
    double v_linear = az * r / (double)0.15;

    double sa = steer_angle;
    double vl = v_linear;
    if (fabs(pre_steer_angle < 1e-4)) {
      control_signal.velocity = {0, 0, 0, 0, 0, 0, 0, 0};
      control_signal.position = {0, 0, 0, 0, -sa, sa, sa, -sa};
    } else {
      control_signal.velocity = {-vl, vl, -vl, vl, 0, 0, 0, 0};
      control_signal.position = {0, 0, 0, 0, -sa, sa, sa, -sa};
    }
    control_signal.header.stamp = ros::Time::now();
    control_signal_pub.publish(control_signal);
    pre_steer_angle = sa;
  } else if (0 == vx && 0 == az) {
    control_signal.velocity = {0, 0, 0, 0, 0, 0, 0, 0};
    control_signal.position = {0, 0, 0, 0, 0, 0, 0, 0};
    control_signal.header.stamp = ros::Time::now();
    control_signal_pub.publish(control_signal);
    pre_steer_angle = 0.0;
  }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "send_teleop_signal");
  ros::NodeHandle nh;

  Teleop teleop(nh);
  ros::Subscriber twist_sub =
      nh.subscribe("cmd_vel", 100, &Teleop::SendTeleopSignal, &teleop);
  ros::spin();
  return 0;
}
