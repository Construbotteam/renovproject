#include <iostream>
#include <string>
#include "ros/ros.h"

using std::cin;
using std::cout;
using std::endl;

int main(int argc, char** argv) {
  ros::init(argc, argv, "param_list_test");
  ros::NodeHandle nh("search_port");

  std::string flag_name = "imu_port_ok_flag"; 
  if (ros::param::has(flag_name)) {
    cout << "ros param has param " << flag_name << endl;
  } else {
    ROS_WARN("ros param hasnt param");
  }

  cout << endl;
  if (ros::param::has("search_port/" + flag_name)) {
    bool f = false;
    if (!ros::param::get("search_port/" + flag_name, f)) {
      cout << "i dont get flag" << endl;
    } 
    cout << "ok flag is " << f << endl;
    cout << "ros param with ns has param " << flag_name << endl;
  } else {
    ROS_WARN("ros param with ns hasnt param");
  }

  cout << endl;
  if (nh.hasParam(flag_name)) {
    bool f = false;
    if (!nh.getParam(flag_name, f)) {
      cout << "i dont get flag with nh" << endl;
    }
    cout << "ok flag with nh is : " << f << endl;
    cout << "nh param has param " << flag_name << endl;
  } else {
    ROS_WARN("nh hasnt param");
  }

  nh.setParam("new_param", 1);
  int k;
  nh.getParam("new_param", k);
  cout << "k is : " << k << endl;
  ros::spinOnce();

  return 0;

}

