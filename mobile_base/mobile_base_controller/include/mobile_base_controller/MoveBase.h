#ifndef MOVEBASE_H
#define MOVEBASE_H

#include <string>
#include <vector>
#include <cmath>
#include <eigen3/Eigen/Dense>

#include "ros/ros.h"
#include "nav_msgs/Path.h"
#include "sensor_msgs/JointState.h"
#include "tf/transform_listener.h"

#include "Model.h"

namespace mobile{

struct Pose2d {
  Pose2d(double x, double y, double yaw) : x(x), y(y), yaw(yaw) {}
  Pose2d() {}
  double x;
  double y;
  double yaw;
};  // struct BasePath
typedef std::vector<Pose2d> Pose2dVec;

class MoveBase {
 public:
  MoveBase();
  ~MoveBase() {}
  void PathCallback(const nav_msgs::Path& path_msg);
  void PoseCallback(const ros::TimerEvent&);
  void ParamInit();
  void Setup();
  void Hoffman(const Pose2d& pose);
  int ClosestPoint(const Pose2d& pose);
 
 private:
  /* PARAMETER */
  std::string map_frame_id;
  std::string base_frame_id;
  double preset_vel;
  bool use_hoffman_control;

  Model vehicle;
  Pose2dVec path;
  bool get_path;
  ros::NodeHandle n_private;
  ros::Publisher cmd_pub;
  
};  // class MoveBase





}  // mobile








#endif