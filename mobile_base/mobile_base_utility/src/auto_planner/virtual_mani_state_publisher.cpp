#include <string>

#include "ros/ros.h"

void virtualBehavior() { ros::Duration(1.4).sleep(); }

int main(int argc, char** argv) {
  ros::init(argc, argv, "virtual_state_publisher");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  std::string arr_param_name;
  nh_private.param("arr_param_name", arr_param_name, std::string(""));

  bool is_task_point = false;
  bool arrived = false;
  while (ros::ok()) {
    ros::param::get("is_task_point", is_task_point);
    ros::param::get(arr_param_name, arrived);
    if (is_task_point && arrived) {
      virtualBehavior();
      ros::param::set("manipulator_flag", true);
      ros::param::set("is_task_point", false);
      ros::param::set(arr_param_name, false);
    }
  }

  return 0;
}