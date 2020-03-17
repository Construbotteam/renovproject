#ifndef PATH_SMOOTHER_H
#define PATH_SMOOTHER_H

#include <fstream>
#include <string>
#include <vector>
#include "geometry_msgs/PoseStamped.h"
#include "nav_msgs/Path.h"
#include "ros/ros.h"
#include "tf/tf.h"

namespace mobile_base {

struct ControlCmd2d {
  double v;
  double w;
};  // struct ControlCmd

class PathSmoother {
 public:
  PathSmoother() : if_initialized_(false) {}
  PathSmoother(const std::string& name);
  virtual ~PathSmoother() {}
  void Initialize(const std::string& name);
  bool UpdatePath(const geometry_msgs::PoseStamped& start,
                  const nav_msgs::Path& old_path, nav_msgs::Path* new_path);

  ControlCmd2d VirtualController(const geometry_msgs::PoseStamped& cur_pose,
                                 const nav_msgs::Path& path);
  void UpdatePose(geometry_msgs::PoseStamped* pose, const ControlCmd2d& cmd);
  double FindDistance(const geometry_msgs::PoseStamped& p1,
                      const geometry_msgs::PoseStamped& p2);
  double FindRestDistance(const int& index, const nav_msgs::Path& path);

 private:
  bool if_initialized_;
  double forward_distance_;
  double max_linear_velo_;
  double max_angular_velo_;
  double dis_threshold_;
  double delta_t_sec_;
  double k_coeff_;
  int path_multi_;

  std::ofstream out_;
};  // class PathSmoother

}  // namespace mobile_base

#endif
