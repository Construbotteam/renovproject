#include <iostream>

#include "geometry_msgs/PoseStamped.h"
#include "ros/ros.h"
#include "std_msgs/Bool.h"

namespace mobile_base {
class MoveTrigger {
 public:
  MoveTrigger(ros::NodeHandle& nh, ros::NodeHandle& nh_private);
  ~MoveTrigger() {}
  void getTrigger(const std_msgs::Bool& msg);

 private:
  std::string get_trigger_topic_;
  std::string send_goal_topic_;

  ros::Publisher goal_pub_;
  ros::Subscriber trigger_sub_;
};  // class MoveTrigger

MoveTrigger::MoveTrigger(ros::NodeHandle& nh, ros::NodeHandle& nh_private) {
  nh_private.param("get_trigger_topic", get_trigger_topic_,
                   std::string("move_trigger"));
  nh_private.param("send_goal_topic", send_goal_topic_,
                   std::string("move_base_simple/goal"));

  goal_pub_ = nh.advertise<geometry_msgs::PoseStamped>(send_goal_topic_, 10);
  trigger_sub_ =
      nh.subscribe(get_trigger_topic_, 10, &MoveTrigger::getTrigger, this);
}

}  // namespace mobile_base

int main(int argc, char** argv) {
  ros::init(argc, argv, "move_trigger");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  mobile_base::MoveTrigger tri(nh, nh_private);
  ros::spin();

  return 0;
}