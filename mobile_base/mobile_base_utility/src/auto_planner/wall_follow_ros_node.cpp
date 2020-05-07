#include "mobile_base_utility/wall_follow_ros.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "wall_follow_ros_node");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  tf2_ros::Buffer bf(ros::Duration(0.2));
  tf2_ros::TransformListener tfl(bf);

  mobile_base::WallFollowROS follower(nh, nh_private, bf);
  ros::spin();

  return 0;
}