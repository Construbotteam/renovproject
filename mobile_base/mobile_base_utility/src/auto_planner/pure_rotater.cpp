#include <iostream>
#include <string>
#include <vector>

#include "ros/ros.h"
#include "sensor_msgs/JointState.h"

std::string js_topic = std::string("");
std::string rot_switch_param = std::string("");
double capture_velo = 0.0;
double capture_duration = 0.0;

void initParam(ros::NodeHandle& nh_private) {
  nh_private.param("js_topic", js_topic, std::string(""));
  nh_private.param("rot_switch_param", rot_switch_param, std::string(""));
  nh_private.param("capture_velo", capture_velo, 0.1);
  capture_duration = 1.25 * M_PI / capture_velo;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "pure_rotater");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  initParam(nh_private);
  ros::Publisher cmd_pub = nh.advertise<sensor_msgs::JointState>(js_topic, 10);

  while (ros::ok()) {
    bool rot_switch = false;
    ros::param::get(rot_switch_param, rot_switch);
    if (rot_switch) {
      sensor_msgs::JointState js;
      ros::Time pre_time = ros::Time::now();
      /* TO DO */
      // turn the wheels first and stop for a while
      while (true) {
        cmd_pub.publish(js);
        if (ros::Time::now().toSec() - pre_time.toSec() > capture_duration) {
          // cmd_pub.publish(0);
          ros::param::set(rot_switch_param, false);
          break;
        }

        ros::Duration(0.02).sleep();
      }
    }

    ros::spinOnce();
  }

  return 0;
}