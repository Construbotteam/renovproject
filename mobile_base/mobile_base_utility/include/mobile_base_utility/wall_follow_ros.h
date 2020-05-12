#ifndef WALL_FOLLOW_ROS_H
#define WALL_FOLLOW_ROS_H

#include <iostream>
#include <string>
#include <thread>
#include <vector>

#include "angles/angles.h"
#include "geometry_msgs/PoseArray.h"
#include "geometry_msgs/PoseStamped.h"
#include "mobile_base_utility/data_visualizer.h"
#include "mobile_base_utility/room_line_extractor.h"
#include "nav_msgs/OccupancyGrid.h"
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "tf/tf.h"
#include "tf2/utils.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace mobile_base {

typedef std::vector<Pose2d> Pose2dVec;
typedef std::vector<geometry_msgs::PoseStamped> PoseStampedVec;

class WallFollowROS {
 public:
  WallFollowROS(ros::NodeHandle& nh, ros::NodeHandle& nh_private,
                tf2_ros::Buffer& bf);
  virtual ~WallFollowROS();
  void initParam(ros::NodeHandle& nh_private);
  void getMapCallback(const nav_msgs::OccupancyGrid& map_msg);
  void getScanCallback(const sensor_msgs::LaserScan& scan_msg);

  geometry_msgs::PoseStamped getStampedPose2D(const double& x, const double& y,
                                              const double& th);
  bool getGlobalPose(geometry_msgs::PoseStamped& global_pose);
  bool scanTransform(std::vector<double>& scan_points);
  Pose2dVec getWayPoints(const LineParamVec& sorted_lines, const Pose2d& pose);
  void transformPoints(const PoseStampedVec& in, PoseStampedVec& out,
                       const geometry_msgs::Transform& tfm);
  void planLoop();

 private:
  std::string map_topic_, goal_topic_, scan_topic_;
  std::string map_frame_id_, base_frame_id_, scan_frame_id_;

  // arr -> abbr. of arrival
  std::string arr_param_name_, arr_namespace_;
  std::string rot_switch_param_;
  bool get_map_, wall_finish_, use_lsd_;

  double capture_velo_, capture_duration_, capture_interval_;
  double circum_radius_, task_dis_, task_interval_;
  double max_laser_range_;

  ros::Subscriber map_sub_, scan_sub_;
  ros::Publisher goal_pub_;
  sensor_msgs::LaserScan scan_data_;
  tf2_ros::Buffer& bf_;

  RoomLineExtractor room_extractor_;
  DataVisualizer visualizer_;
  Pose2d* centers_;
  int center_num_;

  std::thread* plan_thread_;
};  // class WallFollowROS

}  // namespace mobile_base

#endif
