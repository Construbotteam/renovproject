#ifndef BROKE_LINE_H
#define BROKE_LINE_H

#include <Eigen/Dense>
#include <cmath>
#include <iostream>

#include "costmap_2d/costmap_2d_ros.h"
#include "geometry_msgs/PoseStamped.h"
#include "ros/ros.h"
#include "nav_msgs/Path.h"
#include "tf/tf.h"

namespace mobile_base {
class BrokeLine {
 public:
  BrokeLine() : initialize_(false) {}
  BrokeLine(costmap_2d::Costmap2DROS* costmap, ros::NodeHandle& nh)
      : initialize_(false) {
    initialize(costmap, nh);
  }
  virtual ~BrokeLine() {}
  void initialize(costmap_2d::Costmap2DROS* costmap, ros::NodeHandle& nh);
  bool breakPath(const std::vector<geometry_msgs::PoseStamped>& path,
                 std::vector<geometry_msgs::PoseStamped>& new_path);
  double getYaw(const Eigen::Vector2d& vec);
  bool acrossObstacle(const geometry_msgs::PoseStamped& from,
                      const geometry_msgs::PoseStamped& to,
                      costmap_2d::Costmap2D* costmap);
  void setDirection(std::vector<geometry_msgs::PoseStamped>& path);

 private:
  double angle_tolerance_;
  bool initialize_;

  costmap_2d::Costmap2DROS* costmap_ros_;
  ros::Publisher line_path_pub_;
};  // class BrokeLine
}  // namespace mobile_base

#endif