#include "mobile_base_controller/parallel_turn_controller.h"

using mobile_base::ParallelTurnController;

int main(int argc, char** argv) {
  ros::init(argc, argv, "parallel_turn_controller");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  ParallelTurnController controller(nh, nh_private);

  ros::spin();
  return 0;
}