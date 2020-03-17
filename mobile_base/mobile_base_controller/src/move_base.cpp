#include "mobile_base_controller/MoveBase.h"

using mobile::MoveBase;


int main(int argc, char** argv) {
  ros::init(argc, argv, "move_base");
  ros::NodeHandle n;

  MoveBase move;
  ros::Subscriber path_sub = n.subscribe("global_path", 100, &MoveBase::PathCallback, &move);
  ros::Timer pose_timer = n.createTimer(ros::Duration(0.1), &MoveBase::PoseCallback, &move);
  ros::spin();

  return 0;
}