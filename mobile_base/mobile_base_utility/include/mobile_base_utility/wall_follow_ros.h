#ifndef WALL_FOLLOW_ROS_H
#define WALL_FOLLOW_ROS_H

#include <iostream>
#include <string>
#include <vector>

#include "geometry_msgs/PoseStamped.h"
#include "mobile_base_utility/room_line_extractor.h"
#include "nav_msgs/OccupancyGrid.h"
#include "sensor_msgs/LaserScan.h"
#include "ros/ros.h"
#include "tf/tf.h"

namespace mobile_base {
class WallFollowROS {
 public:
  WallFollowROS(ros::NodeHandle& nh, ros::NodeHandle& nh_private);
  virtual ~WallFollowROS() {}
  void initParam(ros::NodeHandle& nh_private);
  void getMapCallback(const nav_msgs::OccupancyGrid& map_msg);
  void getScanCallback(const sensor_msgs::LaserScan& scan_msg);

  geometry_msgs::PoseStamped getStampedPose2D(const double& x, const double& y,
                                              const double& th);

 private:
  std::string map_topic_, goal_topic_, goal_frame_;
  // arr -> abbr. of arrival
  std::string arr_param_name, arr_namespace_;
  bool get_map_, wall_finish_;

  double capture_velo_, capture_duration_, capture_interval_;

  ros::Subscriber map_sub_,scan_sub_;
  ros::Publisher goal_pub_;
  sensor_msgs::LaserScan scan_data_;

  RoomLineExtractor room_extractor_;
};  // class WallFollowROS

}  // namespace mobile_base

#endif