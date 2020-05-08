#include <iostream>
#include <string>
#include <vector>

#include "geometry_msgs/Twist.h"
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"

std::string js_topic = std::string("");
std::string diff_topic = std::string("");
std::string rot_switch_param = std::string("");
double capture_velo = 0.0;
double capture_duration = 0.0;
bool use_diff = false;

void initParam(ros::NodeHandle& nh_private) {
  nh_private.param("js_topic", js_topic, std::string(""));
  nh_private.param("diff_topic", diff_topic, std::string(""));
  nh_private.param("rot_switch_param", rot_switch_param, std::string(""));
  nh_private.param("use_diff", use_diff, false);
  nh_private.param("capture_velo", capture_velo, 0.1);
  capture_duration = 1.25 * M_PI / capture_velo;
}

void fourWheelMotion(const ros::Publisher& cmd_pub) {
  sensor_msgs::JointState js;
  ros::Time pre_time = ros::Time::now();
  double wheel_dis_len = 0.5;
  double wheel_dis_wid = 0.395;
  double hypotenuse = hypot(wheel_dis_len, wheel_dis_wid);
  double r = 0.5 * hypotenuse;

  double steer_angle, v_wheel;
  steer_angle = fabs(atan(wheel_dis_len / wheel_dis_wid));
  v_wheel = capture_velo * r / 0.15;
  js.position.resize(8);
  js.position = {0,           0,           0,           0, -steer_angle,
                 steer_angle, steer_angle, -steer_angle};
  js.velocity.resize(8);
  js.velocity = {0, 0, 0, 0, 0, 0, 0, 0};

  cmd_pub.publish(js);
  ros::Duration(2.0).sleep();

  while (true) {
    js.velocity = {-v_wheel, v_wheel, -v_wheel, v_wheel, 0, 0, 0, 0};
    cmd_pub.publish(js);
    if (ros::Time::now().toSec() - pre_time.toSec() > capture_duration) {
      js.velocity = {0, 0, 0, 0, 0, 0, 0, 0};
      cmd_pub.publish(js);
      ros::param::set(rot_switch_param, false);
      break;
    }

    ros::Duration(0.02).sleep();
  }
}

void diffMotion(const ros::Publisher& cmd_pub) {
  geometry_msgs::Twist twist;
  twist.angular.z = capture_velo;
  ros::Time pre_time = ros::Time::now();

  while (true) {
    cmd_pub.publish(twist);
    if (ros::Time::now().toSec() - pre_time.toSec() > capture_duration) {
      twist.angular.z = 0.0;
      cmd_pub.publish(twist);
      ros::param::set(rot_switch_param, false);
      break;
    }

    ros::Duration(0.02).sleep();
  }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "pure_rotater");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  initParam(nh_private);
  ros::Publisher js_cmd_pub =
      nh.advertise<sensor_msgs::JointState>(js_topic, 10);
  ros::Publisher diff_cmd_pub =
      nh.advertise<geometry_msgs::Twist>(diff_topic, 10);

  while (ros::ok()) {
    bool rot_switch = false;
    ros::param::get(rot_switch_param, rot_switch);
    if (rot_switch) {
      if (!use_diff) {
        fourWheelMotion(js_cmd_pub);
      } else {
        diffMotion(diff_cmd_pub);
      }
    }

    ros::spinOnce();
  }

  return 0;
}