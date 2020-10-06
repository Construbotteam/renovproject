#include "mobile_base_navigation/diff_planner.h"

namespace mobile_base {

int sign(const double& var) { return var > 0.0 ? 1 : -1; }

DiffPlanner::DiffPlanner()
    : initialized_(false),
      goal_reached_(false),
      point_reached_(false),
      rotate_to_first_pose_(false),
      crash_(false),
      tf_(NULL),
      costmap_ros_(NULL) {
  resetControlParam();
}

DiffPlanner::DiffPlanner(std::string name, tf2_ros::Buffer* tf,
                         costmap_2d::Costmap2DROS* costmap_ros)
    : initialized_(false),
      goal_reached_(false),
      point_reached_(false),
      rotate_to_first_pose_(false),
      crash_(false),
      tf_(NULL),
      costmap_ros_(NULL) {
  resetControlParam();
  initialize(name, tf, costmap_ros);
}

DiffPlanner::~DiffPlanner() {
  out_.close();
  if (velo_generator_) {
    delete velo_generator_;
  }
}

void DiffPlanner::initialize(std::string name, tf2_ros::Buffer* tf,
                             costmap_2d::Costmap2DROS* costmap_ros) {
  if (!initialized_) {
    ros::NodeHandle nh_private("~/" + name);
    nh_private.param("debug", debug_, false);

    tf_ = tf;
    costmap_ros_ = costmap_ros;

    out_.open("/home/glh/data/test/diff_controller_err/err_param_omega.txt",
              std::ios::app);
    out_.clear();
    start_time_ = ros::Time::now();

    paramInit(nh_private);
    velo_generator_ =
        new VeloGenerator(max_v_, min_v_, acc_x_limit_, costmap_ros);
    original_plan_.clear();
    initialized_ = true;
  } else {
    ROS_WARN("This planner with name -> '%s' has already been initialized",
             name.c_str());
  }
}

void DiffPlanner::paramInit(ros::NodeHandle& nh_private) {
  nh_private.param("base_frame_id", base_frame_id_, std::string("base_link"));
  nh_private.param("map_frame_id", map_frame_id_, std::string("map"));
  // velocity limit
  nh_private.param("max_v", max_v_, 0.5);
  nh_private.param("min_v", min_v_, 0.01);
  nh_private.param("max_theta", max_theta_v_, 0.5);
  nh_private.param("min_theta", min_theta_v_, 0.08);
  nh_private.param("pure_rotation_max_theta_v_", pure_rotation_max_theta_v_,
                   0.5);

  // acceleration limit
  nh_private.param("acc_x_limit", acc_x_limit_, 2.5);
  nh_private.param("acc_y_limit", acc_y_limit_, 2.5);
  nh_private.param("acc_theta_limit", acc_theta_limit_, 1.5);

  // arrival judgement
  nh_private.param("xy_goal_tolerance", xy_goal_tolerance_, 0.1);
  nh_private.param("yaw_goal_tolerance", yaw_goal_tolerance_, 0.05);

  // tracking paramters
  nh_private.param("forward_index_delta_", forward_index_delta_, 4);
  nh_private.param("control_freq", control_freq_, 20);
  ROS_INFO("control frequency : %d", control_freq_);

  nh_private.param("transform_tolerance", transform_tolerance_, 0.3);
  nh_private.param("theta_coeff", theta_coeff_, 0.3);

  double track_kp, track_ki, track_kd;
  nh_private.param("track_kp", track_kp, 1.0);
  nh_private.param("track_ki", track_ki, 0.0);
  nh_private.param("track_kd", track_kd, 0.0);
  track_pid_.getPid(track_kp, track_ki, track_kd);

  double th_kp, th_ki, th_kd;
  nh_private.param("x_kp", th_kp, 1.0);
  nh_private.param("x_ki", th_ki, 1.0);
  nh_private.param("x_kd", th_kd, 0.0);
  th_pid_.getPid(th_kp, th_ki, th_kd);

  double rotation_kp, rotation_ki, rotation_kd;
  nh_private.param("rotation_kp", rotation_kp, 1.0);
  nh_private.param("rotation_ki", rotation_ki, 0.0);
  nh_private.param("rotation_kd", rotation_kd, 0.0);
  rot_pid_.getPid(rotation_kp, rotation_ki, rotation_kd);

  double v_kp, v_ki, v_kd;
  nh_private.param("v_kp", v_kp, 0.2);
  nh_private.param("v_ki", v_ki, 0.1);
  nh_private.param("v_kd", v_kd, 0.0);
  v_pid_.getPid(v_kp, v_ki, v_kd);

  nh_private.param("k_coeff", k_coeff_, 1.0);
}

bool DiffPlanner::setPlan(const std::vector<geometry_msgs::PoseStamped>& plan) {
  if (!initialized_) {
    ROS_WARN("Please initialize this planner first");
    return false;
  }

  if (plan.size() < 1) {
    ROS_WARN("The size of path to this planner should not be zero");
    return false;
  }

  original_plan_.clear();
  for (size_t i = 0; i < plan.size(); i++) {
    original_plan_.push_back(plan[i]);
  }

  velo_plan_.resize(original_plan_.size());
  if (!velo_generator_->planGlobalVelo(original_plan_, velo_plan_)) {
    ROS_WARN("Failure! Global velocity plan");
  }

  goal_reached_ = false;
  return true;
}

bool DiffPlanner::isGoalReached() {
  /*  */
  return goal_reached_;
}

void DiffPlanner::getCollision(const bool& crash) {
  /*  */
  crash_ = crash;
}

bool DiffPlanner::computeVelocityCommands(geometry_msgs::Twist& cmd_vel) {
  if (!initialized_) {
    ROS_WARN("Please initialize this planner first");
    return false;
  }

  if (original_plan_.size() < 1) {
    ROS_WARN("Please set a plan whose length is non-zero first");
    return false;
  }

  geometry_msgs::PoseStamped global_pose;
  // if (!getRobotPose(&global_pose)) {
  if (!costmap_ros_->getRobotPose(global_pose)) {
    ROS_WARN("Get robot pose failure");
    return false;
  }

  if (rotate_to_first_pose_) {
    // ROS_INFO("run first rot");
    double first_rot_goal_yaw = tf::getYaw(original_plan_[0].pose.orientation);
    // double first_rot_goal_yaw =
    // tf::getYaw(original_plan_.back().pose.orientation);
    double first_rot_cur_yaw = tf::getYaw(global_pose.pose.orientation);
    double first_rot_yaw_diff = angles::shortest_angular_distance(
        first_rot_cur_yaw, first_rot_goal_yaw);

    if (fabs(first_rot_yaw_diff) < yaw_goal_tolerance_) {
      brake(cmd_vel);
      move_state_ = mobile_base::PureRotation;
      resetControlParam();
      ROS_ERROR("finish first rot");
      rotate_to_first_pose_ = false;
    } else {
      rotateToGoal(global_pose, first_rot_goal_yaw, cmd_vel);
      move_state_ = mobile_base::PureRotation;
      moveWithLimit(cmd_vel);
      pre_cmd_ = cmd_vel;
      // ROS_INFO("show me the vel : %.4f", cmd_vel.angular.z);
    }
    return true;
  }

  // judge whether the robot arrives
  double cur_yaw = tf::getYaw(global_pose.pose.orientation);
  double goal_yaw = tf::getYaw(original_plan_.back().pose.orientation);
  double yaw_diff = angles::shortest_angular_distance(cur_yaw, goal_yaw);
  double pose_goal_dis = findDistance(global_pose, original_plan_.back());
  if (fabs(yaw_diff) < yaw_goal_tolerance_ &&
      (fabs(pose_goal_dis) < xy_goal_tolerance_ || point_reached_)) {
    brake(cmd_vel);
    move_state_ = mobile_base::PureRotation;
    goal_reached_ = true;
    point_reached_ = false;
    return true;
  } else if (fabs(pose_goal_dis) < xy_goal_tolerance_ || point_reached_) {
    rotateToGoal(global_pose, goal_yaw, cmd_vel);
    moveWithLimit(cmd_vel);
    move_state_ = mobile_base::PureRotation;
    pre_cmd_ = cmd_vel;
    point_reached_ = true;
    return true;
  } else if (!point_reached_) {
    moveToGoalPid(global_pose, cmd_vel);
    moveWithLimit(cmd_vel);
    move_state_ = mobile_base::MoveForward;
    pre_cmd_ = cmd_vel;
    return true;
  }
}

void DiffPlanner::moveToGoalPid(const geometry_msgs::PoseStamped& global_pose,
                                geometry_msgs::Twist& cmd_vel) {
  // find the closest point on the path
  double min_dis = std::numeric_limits<double>::max();
  double cur_yaw = tf::getYaw(global_pose.pose.orientation);
  uint cur_index = 0;
  for (size_t i = 0; i < original_plan_.size(); i++) {
    double dis = findDistance(global_pose, original_plan_[i]);
    if (dis < min_dis) {
      min_dis = dis;
      cur_index = i;
    }
  }

  // if (!velo_generator_->planLocalVelo(cur_index, global_pose, velo_plan_)) {
  //   ROS_WARN("Failure! Local velocity plan");
  //   cmd_vel.linear.x = 0.0;
  //   cmd_vel.angular.z = 0.0;
  //   return;
  // }

  // get the index corresponding to forward point
  uint forward_index = std::min(cur_index + (uint)forward_index_delta_,
                                (uint)original_plan_.size() - 1);
  double forward_x = original_plan_[forward_index].pose.position.x;
  double forward_y = original_plan_[forward_index].pose.position.y;
  double forward_yaw =
      tf::getYaw(original_plan_[forward_index].pose.orientation);
  double forward_yaw_diff =
      angles::shortest_angular_distance(cur_yaw, forward_yaw);

  Eigen::Vector3d err_vec_world, err_vec_base;
  err_vec_world << forward_x - global_pose.pose.position.x,
      forward_y - global_pose.pose.position.y, forward_yaw_diff;
  /*<< x_err_base << "  "
  this matrix is used to transform err from world(i.e. map frame) to base.
  tfed -> abbr. of transformed
  matrix is :
  | cos(th) sin(th) 0|
  |-sin(th) cos(th) 0|
  |       0       0 1|
  */
  Eigen::Matrix3d world_to_base_err_tfed;
  world_to_base_err_tfed << cos(cur_yaw), sin(cur_yaw), 0.0, -sin(cur_yaw),
      cos(cur_yaw), 0.0, 0.0, 0.0, 1.0;
  err_vec_base = world_to_base_err_tfed * err_vec_world;

  double x_err_base, y_err_base, yaw_err_base;
  x_err_base = err_vec_base(0);
  y_err_base = err_vec_base(1);
  yaw_err_base = err_vec_base(2);

  // compute vx
  if (crash_) {
    ROS_INFO("I crash");
    v_err_.err_ = 0.0 - pre_cmd_.linear.x;
  } else {
    v_err_.err_ = velo_plan_[cur_index] - pre_cmd_.linear.x;
  }
  cmd_vel.linear.x = pre_cmd_.linear.x + incrementPid(v_pid_, v_err_);

  double turning_angle = atan2(y_err_base, x_err_base);
  turning_angle = sign(turning_angle) * std::min(fabs(turning_angle), M_PI / 2);

  cmd_vel.angular.z = turning_angle;
  /***/
  if (move_state_ == MoveForward) {
    geometry_msgs::Twist cmd_vel_mirror;
    cmd_vel_mirror.angular.z =
        angles::normalize_angle(cmd_vel.angular.z + M_PI);
    cmd_vel_mirror.linear.x = -cmd_vel.linear.x;

    double acc_angular, acc_angular_mirror;
    acc_angular_mirror = angles::shortest_angular_distance(
        pre_cmd_.angular.z, cmd_vel_mirror.angular.z);
    acc_angular = angles::shortest_angular_distance(pre_cmd_.angular.z,
                                                    cmd_vel.angular.z);
    if (fabs(acc_angular) > fabs(acc_angular_mirror)) {
      cmd_vel = cmd_vel_mirror;
    }
  }
  /***/

  // compute theta velocity
  // theta velocity = pid(y_err)
  track_e_[1].err_ = y_err_base;
  // track_e_[2].err_ = yaw_err_base;

  // update the pre_err and pre_pre_err
  for (size_t i = 0; i < 3; i++) {
    track_e_[i].update();
  }
  v_err_.update();
}

double DiffPlanner::findDistance(const geometry_msgs::PoseStamped& pose1,
                                 const geometry_msgs::PoseStamped& pose2) {
  double x1, y1, x2, y2;

  x1 = pose1.pose.position.x;
  y1 = pose1.pose.position.y;
  x2 = pose2.pose.position.x;
  y2 = pose2.pose.position.y;

  return hypot(x1 - x2, y1 - y2);
}

double DiffPlanner::findRestJourney(const uint& cur_index) {
  double rest_dis = 0;
  for (size_t i = cur_index; i < original_plan_.size() - 1; i++) {
    rest_dis += findDistance(original_plan_[i], original_plan_[i + 1]);
  }
  return rest_dis;
}

void DiffPlanner::enableFirstRotation() {
  //
  rotate_to_first_pose_ = true;
}

bool DiffPlanner::getRobotPose(geometry_msgs::PoseStamped* global_pose) {
  geometry_msgs::PoseStamped target_pose;
  tf2::toMsg(tf2::Transform::getIdentity(), target_pose.pose);
  geometry_msgs::PoseStamped robot_pose;
  tf2::toMsg(tf2::Transform::getIdentity(), robot_pose.pose);
  robot_pose.header.frame_id = base_frame_id_;
  robot_pose.header.stamp = ros::Time();

  // save time for checking tf delay later
  ros::Time cur_time = ros::Time::now();

  // get the global pose of the robot
  try {
    tf_->transform(robot_pose, target_pose, map_frame_id_);
  } catch (tf2::LookupException& ex) {
    ROS_ERROR("No Transform available Error looking up robot pose: %s",
              ex.what());
    return false;
  } catch (tf2::ConnectivityException& ex) {
    ROS_ERROR("Connectivity Error looking up robot pose: %s", ex.what());
    return false;
  } catch (tf2::ExtrapolationException& ex) {
    ROS_ERROR("Extrapolation Error looking up robot pose: %s", ex.what());
    return false;
  }

  // check global_pose timeout
  if (cur_time.toSec() - target_pose.header.stamp.toSec() >
      transform_tolerance_) {
    ROS_WARN(
        "Costmap2DROS transform timeout. Current time: %.4f, "
        "global_pose stamp: %.4f, tolerance: %.4f",
        cur_time.toSec(), target_pose.header.stamp.toSec(),
        transform_tolerance_);
    return false;
  }

  global_pose->header = target_pose.header;
  global_pose->pose = target_pose.pose;

  return true;
}

void DiffPlanner::rotateToGoal(const geometry_msgs::PoseStamped& global_pose,
                               const double& goal_yaw,
                               geometry_msgs::Twist& cmd_vel) {
  // this function only provide rotation velocity
  cmd_vel.linear.x = cmd_vel.linear.y = cmd_vel.linear.z = 0.0;
  cmd_vel.angular.x = cmd_vel.angular.y = 0.0;

  // compute angle error
  double cur_yaw = tf::getYaw(global_pose.pose.orientation);
  double yaw_diff = angles::shortest_angular_distance(cur_yaw, goal_yaw);
  rot_e_.err_ = yaw_diff;

  cmd_vel.angular.z = pre_cmd_.angular.z + incrementPid(rot_pid_, rot_e_);

  rot_e_.pre_pre_err_ = rot_e_.pre_err_;
  rot_e_.pre_err_ = rot_e_.err_;
}

void DiffPlanner::resetControlParam() {
  // reset the tracking errors and control commands to zero
  for (size_t i = 0; i < 3; i++) {
    track_e_[i].reset();
  }
  rot_e_.reset();
  v_err_.reset();

  // pre_angular_v_ = 0.0;
  pre_cmd_.linear.x = pre_cmd_.linear.y = pre_cmd_.linear.z =
      pre_cmd_.angular.x = pre_cmd_.angular.y = pre_cmd_.angular.z = 0.0;
}

double DiffPlanner::incrementPid(const PidParam& param,
                                 const ControlError& err) {
  double ctrl;

  ctrl = param.kp_ * (err.err_ - err.pre_err_) + param.ki_ * err.err_ +
         param.kd_ * (err.err_ - 2 * err.pre_err_ + err.pre_pre_err_);

  return ctrl;
}

double DiffPlanner::positionPid(const PidParam& param,
                                const ControlError& err) {
  double ctrl;

  ctrl = param.kp_ * err.err_ + param.ki_ * err.integ_err_ +
         param.kd_ * (err.err_ - err.pre_err_);

  return ctrl;
}

void DiffPlanner::brake(geometry_msgs::Twist& twist) {
  //
  twist.angular.x = twist.angular.y = twist.angular.z = 0.0;
  twist.linear.x = twist.linear.y = twist.linear.z = 0.0;
}

void DiffPlanner::moveWithLimit(geometry_msgs::Twist& cmd_vel) {
  double dt = 1.0 / control_freq_;

  geometry_msgs::Twist cmd_vel_mirror;
  cmd_vel_mirror.angular.z = angles::normalize_angle(cmd_vel.angular.z + M_PI);
  cmd_vel_mirror.linear.x = -cmd_vel.linear.x;

  double acc_linear, acc_angular, acc_angular_mirror;
  //  acc_angular_mirror = (cmd_vel_mirror.angular.z - pre_cmd_.angular.z) / dt;
  //  acc_angular = (cmd_vel.angular.z - pre_cmd_.angular.z) / dt;
  acc_angular_mirror = angles::shortest_angular_distance(
                           pre_cmd_.angular.z, cmd_vel_mirror.angular.z) /
                       dt;
  acc_angular =
      angles::shortest_angular_distance(pre_cmd_.angular.z, cmd_vel.angular.z) /
      dt;
  /*
  if (fabs(acc_angular) > fabs(acc_angular_mirror)) {
    acc_angular = acc_angular_mirror;
    cmd_vel = cmd_vel_mirror;
  }
  */

  acc_linear = (cmd_vel.linear.x - pre_cmd_.linear.x) / dt;

  /*
  Attention here!!
  The order of acc-limit and max-limit is important.
  If we place max-limit after acc-limit, when we modify the maximum velocity
  during navigation, this might cause a sudden velocity decrease
  */
  // the maximum velocity limit
  cmd_vel.linear.x = cmd_vel.linear.x > 0 ? std::min(cmd_vel.linear.x, max_v_)
                                          : std::max(cmd_vel.linear.x, -max_v_);

  if (move_state_ == PureRotation) {
    cmd_vel.angular.z =
        cmd_vel.angular.z > 0
            ? std::min(cmd_vel.angular.z, pure_rotation_max_theta_v_)
            : std::max(cmd_vel.angular.z, -pure_rotation_max_theta_v_);
  } else if (move_state_ == MoveForward) {
    /*
      cmd_vel.angular.z = cmd_vel.angular.z > 0
                              ? std::min(cmd_vel.angular.z, M_PI / 2)
                              : std::max(cmd_vel.angular.z, -M_PI / 2);
                              */
  }

  // the acceleration limit
  if (fabs(acc_linear) > acc_x_limit_) {
    cmd_vel.linear.x = cmd_vel.linear.x + sign(acc_linear) * acc_x_limit_ * dt;
  }
  if (fabs(acc_angular) > acc_theta_limit_) {
    cmd_vel.angular.z =
        cmd_vel.angular.z + sign(acc_angular) * acc_theta_limit_ * dt;
  }
}

double DiffPlanner::purePursuit(const geometry_msgs::PoseStamped& global_pose,
                                const geometry_msgs::PoseStamped& forward_pose,
                                const double& ref_v) {
  double turn_radius, theta, turn_v;
  Eigen::Vector2d car_vec, forward_vec, x_axis;

  double cur_yaw = tf::getYaw(global_pose.pose.orientation);
  x_axis << 1.0, 0.0;
  car_vec << cos(cur_yaw), sin(cur_yaw);
  forward_vec << forward_pose.pose.position.x - global_pose.pose.position.x,
      forward_pose.pose.position.y - global_pose.pose.position.y;

  theta = M_PI / 2 -
          acos(car_vec.dot(forward_vec) / car_vec.norm() / forward_vec.norm());
  turn_radius = 0.5 * forward_vec.norm() / cos(theta);
  turn_v = ref_v / turn_radius;

  double forward_vec_yaw =
      sign(forward_vec(1)) * acos(forward_vec.dot(x_axis) / forward_vec.norm());
  turn_v = sign(angles::shortest_angular_distance(cur_yaw, forward_vec_yaw)) *
           fabs(turn_v);

  return turn_v;
}

double DiffPlanner::getMaxV() { return max_v_; }

double DiffPlanner::getMinV() { return min_v_; }

double DiffPlanner::getMaxThetaV() { return max_theta_v_; }

double DiffPlanner::getMinThetaV() { return min_theta_v_; }

MoveState DiffPlanner::getMoveState() { return move_state_; }

}  // namespace mobile_base
