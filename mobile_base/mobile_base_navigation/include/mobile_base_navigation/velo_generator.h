#ifndef VELO_GENERATOR_H
#define VELO_GENERATOR_H

#include <eigen3/Eigen/Dense>
#include <iostream>

#include "costmap_2d/costmap_2d_ros.h"
#include "geometry_msgs/Polygon.h"
#include "geometry_msgs/PoseStamped.h"
#include "ros/ros.h"
#include "tf/tf.h"

namespace mobile_base {
class VeloGenerator {
 public:
  VeloGenerator(const double& v_max, const double& v_min, const double& acc_max,
                costmap_2d::Costmap2DROS* costmap);
  virtual ~VeloGenerator() {}
  bool planGlobalVelo(const std::vector<geometry_msgs::PoseStamped>& plan,
                      std::vector<double>& velo_plan);
  bool planLocalVelo(const int& index,
                     const geometry_msgs::PoseStamped& global_pose,
                     std::vector<double>& velo_plan);

  void getMaxVelo(const double& v);
  void getMaxAcc(const double& acc);

  bool linearBrake(const std::vector<geometry_msgs::PoseStamped>& plan,
                   std::vector<double>& velo_plan);

 private:
  double v_max_, v_min_;
  double acc_max_;

  geometry_msgs::Polygon base_outline_;
  costmap_2d::Costmap2DROS* costmap_ros_;
  std::vector<geometry_msgs::PoseStamped> global_plan_;
};  // class VeloGenerator
}  // namespace mobile_base

#endif