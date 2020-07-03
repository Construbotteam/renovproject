#ifndef DIFF_PLANNER_H
#define DIFF_PLANNER_H

#include <eigen3/Eigen/Dense>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include "angles/angles.h"
#include "costmap_2d/costmap_2d_ros.h"
#include "geometry_msgs/Twist.h"
#include "mobile_base_navigation/velo_generator.h"
#include "nav_core/base_local_planner.h"
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "tf/tf.h"
#include "tf2/utils.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace mobile_base {

int sign(const double &var);

struct PidParam {
  PidParam() : kp_(0.0), ki_(0.0), kd_(0.0) {}
  void getPid(const double &kp, const double &ki, const double &kd) {
    kp_ = kp;
    ki_ = ki;
    kd_ = kd;
  }

  double kp_;
  double ki_;
  double kd_;
};  // struct PidParam

struct ControlError {
  ControlError()
      : err_(0.0), pre_err_(0.0), pre_pre_err_(0.0), integ_err_(0.0) {}
  void reset() { err_ = pre_err_ = pre_pre_err_ = integ_err_ = 0.0; }
  void update() {
    pre_pre_err_ = pre_err_;
    pre_err_ = err_;
    integ_err_ += err_;
  }
  double err_;
  double pre_err_;
  double pre_pre_err_;
  double integ_err_;
};  // struct ControlError

enum MoveState { MoveForward, PureRotation, MoveStraight };  // enum MoveState

class DiffPlanner : public nav_core::BaseLocalPlanner {
 public:
  DiffPlanner();
  DiffPlanner(std::string name, tf2_ros::Buffer *tf,
              costmap_2d::Costmap2DROS *costmap_ros);
  virtual ~DiffPlanner();
  void initialize(std::string name, tf2_ros::Buffer *tf,
                  costmap_2d::Costmap2DROS *costmap_ros);
  void paramInit(ros::NodeHandle &nh_private);
  bool computeVelocityCommands(geometry_msgs::Twist &cmd_vel);
  bool isGoalReached();
  void getCollision(const bool &crash);
  bool setPlan(const std::vector<geometry_msgs::PoseStamped> &plan);

  double findDistance(const geometry_msgs::PoseStamped &pose1,
                      const geometry_msgs::PoseStamped &pose2);
  double findRestJourney(const uint &cur_index);
  void enableFirstRotation();
  bool getRobotPose(geometry_msgs::PoseStamped *cur_pose);

  double getMaxV();
  double getMinV();
  double getMaxThetaV();
  double getMinThetaV();

  void setMoveState(const MoveState& state) { move_state_ = state; }
  MoveState getMoveState();

  void rotateToGoal(const geometry_msgs::PoseStamped &global_pose,
                    const double &goal_yaw, geometry_msgs::Twist &cmd_vel);
  void moveToGoalPid(const geometry_msgs::PoseStamped &global_pose,
                     geometry_msgs::Twist &cmd_vel);
  void resetControlParam();
  double incrementPid(const PidParam &param, const ControlError &err);
  double positionPid(const PidParam &param, const ControlError &err);
  void brake(geometry_msgs::Twist &cmd_vel);
  void moveWithLimit(geometry_msgs::Twist &cmd_vel);
  double purePursuit(const geometry_msgs::PoseStamped &global_pose,
                     const geometry_msgs::PoseStamped &forward_pose,
                     const double &ref_v);

 private:
  std::string base_frame_id_;
  std::string map_frame_id_;

  std::string name_;
  tf2_ros::Buffer *tf_;
  geometry_msgs::Twist pre_cmd_;
  costmap_2d::Costmap2DROS *costmap_ros_;
  std::vector<geometry_msgs::PoseStamped> original_plan_;
  std::vector<double> velo_plan_;
  ros::Time start_time_;

  bool initialized_, debug_, crash_;
  bool point_reached_, goal_reached_, rotate_to_first_pose_;

  double max_v_, min_v_, max_theta_v_, min_theta_v_, pure_rotation_max_theta_v_;
  double acc_x_limit_, acc_y_limit_, acc_theta_limit_;
  double xy_goal_tolerance_, yaw_goal_tolerance_;
  double transform_tolerance_;
  double theta_coeff_;
  int forward_index_delta_;
  int control_freq_;
  std::ofstream out_;

  double forward_distance_;
  double k_coeff_;

  PidParam track_pid_, rot_pid_, v_pid_, th_pid_;
  // tracking error includes [0]x_err, [1]y_err, [2]yaw_err
  ControlError track_e_[3], rot_e_, v_err_;
  VeloGenerator *velo_generator_;

  MoveState move_state_;
};  // class DiffPlanner

}  // namespace mobile_base

#endif
