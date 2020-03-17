#include <yaml-cpp/yaml.h>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include "nav_msgs/Path.h"
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "tf/tf.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

namespace mobile_base {

struct ControlError {
  ControlError() : err(0.0), pre_err(0.0), ppre_err(0.0) {}
  double err;
  double pre_err;
  double ppre_err;
};  // struct ControlError

struct PIDParam {
  PIDParam() : kp(0.0), ki(0.0), kd(0.0) {}
  void init(double p, double i, double d) {
    kp = p;
    ki = i;
    kd = d;
  }
  double kp;
  double ki;
  double kd;
};  // struct PIDParam

class ParallelTurnController {
 public:
  ParallelTurnController(ros::NodeHandle& nh, ros::NodeHandle& nh_private);
  virtual ~ParallelTurnController() {}
  void paramInit(ros::NodeHandle& nh_private);
  void readFile(const std::string& addr);
  bool getPose(geometry_msgs::PoseStamped* cur_pose);
  void getPathCallback(const nav_msgs::Path& path);
  void motorControl(const ros::TimerEvent&);
  double findDistance(const geometry_msgs::PoseStamped& p1,
                      const geometry_msgs::PoseStamped& p2);
  size_t findClosestPoint(const geometry_msgs::PoseStamped& cur_pose,
                          const nav_msgs::Path& path);
  void moveStraight(const double& v, const double& turning_angle);
  void moveRotation(const geometry_msgs::PoseStamped& cur_pose,
                    const double& goal_yaw);
  void brake();

 private:
  double max_linear_velocity_;
  double max_angular_velocity_;
  double max_turning_speed_;
  double max_acc_;
  double control_freq_;
  double forward_dis_;
  double control_v_;
  double pre_control_v_;
  double pre_turning_angle_;
  double arrival_tolerance_;
  double yaw_tolerance_;
  double turning_tolerance_;

  double front_rear_track_;
  double left_right_track_;
  double wheel_radius_;

  double t_flag_;

  ControlError linear_journey_;
  ControlError angular_journey_;
  PIDParam linear_pid_;
  PIDParam angular_pid_;

  std::string config_file_addr_;
  std::string joint_signal_pub_topic_;
  std::string path_sub_topic_;
  std::string map_frame_id_;
  std::string base_frame_id_;

  bool if_get_path_;
  bool if_reach_point_;
  bool if_reach_goal_;

  nav_msgs::Path cur_path_;
  sensor_msgs::JointState joint_signal_;

  ros::Publisher joint_signal_pub_;
  ros::Subscriber path_sub_;
  ros::Timer control_timer_;

};  // class ParallelTurnController

}  // namespace mobile_base