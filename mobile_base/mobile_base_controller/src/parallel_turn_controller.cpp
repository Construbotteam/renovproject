#include "mobile_base_controller/parallel_turn_controller.h"

using mobile_base::ParallelTurnController;

ParallelTurnController::ParallelTurnController(ros::NodeHandle& nh,
                                               ros::NodeHandle& nh_private) {
  paramInit(nh_private);
  // readFile(config_file_addr_);

  if_get_path_ = false;
  if_reach_goal_ = false;
  if_reach_point_ = false;
  control_v_ = 0.0;
  pre_control_v_ = 0.0;
  pre_turning_angle_ = 0.0;

  t_flag_ = ros::Time::now().toSec();

  joint_signal_.header.frame_id = "motor";
  joint_signal_.name.resize(8);
  joint_signal_.position.resize(8);
  joint_signal_.velocity.resize(8);
  joint_signal_.name = {"front_left_walking",  "front_right_walking",
                        "rear_left_walking",   "rear_right_walking",
                        "front_left_steering", "front_right_steering",
                        "rear_left_steering",  "rear_right_steering"};

  joint_signal_pub_ =
      nh.advertise<sensor_msgs::JointState>(joint_signal_pub_topic_, 10);
  path_sub_ = nh.subscribe(path_sub_topic_, 10,
                           &ParallelTurnController::getPathCallback, this);
  control_timer_ = nh.createTimer(ros::Duration(1.0 / control_freq_),
                                  &ParallelTurnController::motorControl, this);
}

void ParallelTurnController::paramInit(ros::NodeHandle& nh_private) {
  nh_private.param("config_file_addr", config_file_addr_, std::string(""));
  nh_private.param("joint_signal_pub_topic", joint_signal_pub_topic_,
                   std::string("joint_signal"));
  nh_private.param("path_sub_topic", path_sub_topic_, std::string("path"));
  nh_private.param("map_frame_id", map_frame_id_, std::string("map"));
  nh_private.param("base_frame_id", base_frame_id_, std::string("base_link"));
  nh_private.param("control_freq", control_freq_, 10.0);
  nh_private.param("forward_dis", forward_dis_, 0.2);
  nh_private.param("max_acc", max_acc_, 0.05);
  nh_private.param("arrival_tolerance", arrival_tolerance_, 0.05);
  nh_private.param("yaw_tolerance", yaw_tolerance_, 0.08);
  nh_private.param("turning_tolerance", turning_tolerance_, 1.5708);
  nh_private.param("front_rear_track", front_rear_track_, 0.5);
  nh_private.param("left_right_track", left_right_track_, 0.395);
  nh_private.param("wheel_radius", wheel_radius_, 0.15);
  nh_private.param("max_linear_velocity", max_linear_velocity_, 1.0);
  nh_private.param("max_angular_velocity", max_angular_velocity_, 0.05);
  nh_private.param("max_turning_speed", max_turning_speed_, 0.7);

  double lkp, lki, lkd;
  double akp, aki, akd;
  nh_private.param("linear_kp", lkp, 0.5);
  nh_private.param("linear_ki", lki, 0.0);
  nh_private.param("linear_kd", lkd, 0.0);
  linear_pid_.init(lkp, lki, lkd);
  nh_private.param("angular_kp", lkp, 0.5);
  nh_private.param("angular_kp", aki, 0.0);
  nh_private.param("angular_kp", akd, 0.0);
  angular_pid_.init(akp, aki, akd);
}

void ParallelTurnController::readFile(const std::string& addr) {
  YAML::Node config = YAML::LoadFile(addr);
  max_linear_velocity_ = config["max_linear_velocity"].as<double>();
  max_angular_velocity_ = config["max_angular_velocity"].as<double>();
}

bool ParallelTurnController::getPose(geometry_msgs::PoseStamped* cur_pose) {
  tf2_ros::Buffer bf(ros::Duration(10.0));
  tf2_ros::TransformListener tf_listener(bf);

  geometry_msgs::TransformStamped tfs;
  if (bf.canTransform(map_frame_id_, base_frame_id_, ros::Time(0),
                      ros::Duration(0.2))) {
    try {
      tfs = bf.lookupTransform(map_frame_id_, base_frame_id_, ros::Time(0),
                               ros::Duration(0.2));
    } catch (tf2::TransformException& ex) {
      ROS_WARN("tf error in parallel turn controller -> %s", ex.what());
      ros::Duration(1.0).sleep();
      return false;
    }
  } else {
    ROS_WARN("parallel turn controller error -> no transform");
    ros::Duration(0.1).sleep();
    return false;
  }

  cur_pose->header.frame_id = tfs.header.frame_id;
  cur_pose->header.stamp = tfs.header.stamp;

  cur_pose->pose.position.x = tfs.transform.translation.x;
  cur_pose->pose.position.y = tfs.transform.translation.y;
  cur_pose->pose.position.z = tfs.transform.translation.z;

  // cur_pose->pose.orientation = tfs.transform.rotation;
  cur_pose->pose.orientation.w = tfs.transform.rotation.w;
  cur_pose->pose.orientation.x = tfs.transform.rotation.x;
  cur_pose->pose.orientation.y = tfs.transform.rotation.y;
  cur_pose->pose.orientation.z = tfs.transform.rotation.z;

  return true;
}

void ParallelTurnController::getPathCallback(const nav_msgs::Path& path) {
  cur_path_ = path;
  // ROS_INFO("i get a path for parallel turn controller");
  if_get_path_ = true;
  if_reach_goal_ = false;
}

void ParallelTurnController::motorControl(const ros::TimerEvent&) {
  if (!if_get_path_) {
    ROS_WARN("please provide a path");
    ros::Duration(1.0).sleep();
    return;
  }

  if (if_reach_goal_) {
    ROS_INFO("get path and reach the goal point, no control signal output");
    ros::Duration(1.0).sleep();
    return;
  }

  geometry_msgs::PoseStamped cur_pose;
  if (!getPose(&cur_pose)) {
    ROS_WARN("unable to get current global pose");
    return;
  }

  double max_dis_diff = 1000000.0;
  size_t forward_index = cur_path_.poses.size() - 1;
  size_t cur_index = findClosestPoint(cur_pose, cur_path_);
  for (size_t i = cur_index; i < cur_path_.poses.size(); i++) {
    double point_dis = findDistance(cur_pose, cur_path_.poses[i]);
    double point_dis_diff = fabs(point_dis - forward_dis_);
    if (point_dis_diff < max_dis_diff) {
      max_dis_diff = point_dis_diff;
      forward_index = i;
    }
  }
  double cur_forward_dis =
      findDistance(cur_pose, cur_path_.poses[forward_index]);

  geometry_msgs::PoseStamped forward_point = cur_path_.poses[forward_index];
  double forward_angle =
      atan2(forward_point.pose.position.y - cur_pose.pose.position.y,
            forward_point.pose.position.x - cur_pose.pose.position.x);

  // ROS_ERROR("x: %.6f, y: %.6f, z: %.6f, w: %.6f",
  // cur_pose.pose.orientation.x,
  //           cur_pose.pose.orientation.y, cur_pose.pose.orientation.z,
  //           cur_pose.pose.orientation.w);
  double cur_yaw = tf::getYaw(cur_pose.pose.orientation);

  double angle_diff = forward_angle - cur_yaw;
  double turning_angle = angle_diff;
  if (fabs(angle_diff) > M_PI) {
    if (forward_angle < 0) {
      turning_angle = 2 * M_PI + angle_diff;
    } else {
      turning_angle = angle_diff - 2 * M_PI;
    }
  }

  linear_journey_.err = 0;
  for (size_t i = cur_index; i < cur_path_.poses.size() - 1; i++) {
    linear_journey_.err +=
        findDistance(cur_path_.poses[i], cur_path_.poses[i + 1]);
  }

  control_v_ =
      linear_pid_.kp * (linear_journey_.err - linear_journey_.pre_err) +
      linear_pid_.kd * (linear_journey_.err - 2 * linear_journey_.pre_err +
                        linear_journey_.ppre_err) +
      linear_pid_.ki * linear_journey_.err;
  linear_journey_.ppre_err = linear_journey_.pre_err;
  linear_journey_.pre_err = linear_journey_.err;

  if (fabs(control_v_) > max_linear_velocity_) {
    control_v_ = pow(-1, std::signbit(control_v_)) * max_linear_velocity_;
  }

  // double cur_turning_speed =
  //     fabs((turning_angle - pre_turning_angle_) / (1.0 / control_freq_));
  // if (cur_turning_speed > max_turning_speed_) {
  //   if (turning_angle > pre_turning_angle_) {
  //     turning_angle =
  //         pre_turning_angle_ + max_turning_speed_ * (1.0 / control_freq_);
  //   } else {
  //     turning_angle =
  //         pre_turning_angle_ - max_turning_speed_ * (1.0 / control_freq_);
  //   }
  // }
  // pre_turning_angle_ = turning_angle;

  if (fabs(turning_angle) > M_PI / 2 &&
      fabs(turning_angle - pre_turning_angle_) > turning_tolerance_) {
    turning_angle =
        turning_angle + pow(-1, std::signbit(turning_angle) + 1) * M_PI;
    control_v_ = -control_v_;
  }
  pre_turning_angle_ = turning_angle;

  int sign_tmp = 0;
  if (control_v_ > pre_control_v_) {
    sign_tmp = 1;
  } else {
    sign_tmp = -1;
  }
  double cur_acc = fabs(pre_control_v_ - control_v_) / (1.0 / control_freq_);
  if (cur_acc > max_acc_) {
    control_v_ = pre_control_v_ + sign_tmp * max_acc_ * (1.0 / control_freq_);
  }
  pre_control_v_ = control_v_;

  /********************/
  double cur_t_flag = ros::Time::now().toSec();
  std::ofstream out("/home/glh/dataTest/parallel_controller/control.txt",
                    std::ios::app);
  out << control_v_ << "  " << turning_angle << "  " << cur_t_flag - t_flag_
      << std::endl;
  out.close();

  /********************/

  double pose_goal_dis = findDistance(cur_pose, cur_path_.poses.back());
  if (pose_goal_dis > arrival_tolerance_) {
    moveStraight(control_v_, turning_angle);
    if_reach_point_ = false;
  } else {
    if (!if_reach_point_) {
      joint_signal_.velocity = {0, 0, 0, 0, 0, 0, 0, 0};
      double angle_tmp = fabs(atan(front_rear_track_ / left_right_track_));
      joint_signal_.position = {0,         0,         0,         0,
                                angle_tmp, angle_tmp, angle_tmp, angle_tmp};
      if_reach_point_ = true;
      joint_signal_pub_.publish(joint_signal_);
      ros::Duration(2.0).sleep();
    }
    double goal_yaw = tf::getYaw(cur_path_.poses.back().pose.orientation);
    moveRotation(cur_pose, goal_yaw);
  }
}

double ParallelTurnController::findDistance(
    const geometry_msgs::PoseStamped& p1,
    const geometry_msgs::PoseStamped& p2) {
  double dis;
  double x1, x2, y1, y2;

  x1 = p1.pose.position.x;
  y1 = p1.pose.position.y;

  x2 = p2.pose.position.x;
  y2 = p2.pose.position.y;

  dis = sqrt(pow(x1 - x2, 2) + pow(y1 - y2, 2));
  return dis;
}

size_t ParallelTurnController::findClosestPoint(
    const geometry_msgs::PoseStamped& cur_pose, const nav_msgs::Path& path) {
  double max_dis = 1000000.0;
  size_t index = path.poses.size() - 1;
  for (size_t i = 0; i < path.poses.size(); i++) {
    double dis_tmp = findDistance(cur_pose, path.poses[i]);
    if (dis_tmp < max_dis) {
      max_dis = dis_tmp;
      index = i;
    }
  }

  return index;
}

void ParallelTurnController::moveStraight(const double& v,
                                          const double& turning_angle) {
  joint_signal_.velocity = {v, v, v, v, 0, 0, 0, 0};
  joint_signal_.position = {
      0, 0, 0, 0, turning_angle, turning_angle, turning_angle, turning_angle};
  joint_signal_pub_.publish(joint_signal_);
}

void ParallelTurnController::moveRotation(
    const geometry_msgs::PoseStamped& cur_pose, const double& goal_yaw) {
  double cur_yaw = tf::getYaw(cur_pose.pose.orientation);
  // double yaw_diff = goal_yaw - cur_yaw;
  angular_journey_.err = goal_yaw - cur_yaw;

  std::cout << "curr yaw : " << cur_yaw << std::endl;
  // std::cout << "goal yaw : " << goal_yaw << std::endl;
  // std::cout << "yaw diff : " << yaw_diff << std::endl;
  if (fabs(angular_journey_.err) < yaw_tolerance_) {
    brake();
    if_reach_goal_ = true;
    return;
  }

  int sign_tmp = 1;
  if (fabs(angular_journey_.err) > M_PI) {
    if (goal_yaw < 0) {
      angular_journey_.err = 2 * M_PI + angular_journey_.err;
    } else {
      angular_journey_.err = angular_journey_.err - 2 * M_PI;
    }
  }
  if (angular_journey_.err >= 0) {
    sign_tmp = 1;
  } else {
    sign_tmp = -1;
  }
  double base_angular_v;

  // base_angular_v = sign_tmp * kp_ / 2 * fabs(yaw_diff);
  base_angular_v =
      angular_pid_.kp * (angular_journey_.err - angular_journey_.pre_err) +
      angular_pid_.kd * (angular_journey_.err - 2 * angular_journey_.pre_err +
                         angular_journey_.ppre_err) +
      angular_pid_.ki * angular_journey_.err;
  angular_journey_.ppre_err = angular_journey_.pre_err;
  angular_journey_.pre_err = angular_journey_.err;
  if (fabs(base_angular_v) > max_angular_velocity_) {
    base_angular_v = sign_tmp * max_angular_velocity_;
  }
  std::cout << "base angular v : " << base_angular_v << std::endl;

  double pure_rotate_angle = fabs(atan(front_rear_track_ / left_right_track_));
  double r = 0.2 * sqrt(pow(front_rear_track_, 2) + pow(left_right_track_, 2));
  double v_linear = base_angular_v * r / wheel_radius_;

  joint_signal_.velocity = {-v_linear, v_linear, -v_linear, v_linear,
                            0,         0,        0,         0};
  joint_signal_.position = {0,
                            0,
                            0,
                            0,
                            -pure_rotate_angle,
                            pure_rotate_angle,
                            pure_rotate_angle,
                            -pure_rotate_angle};
  joint_signal_pub_.publish(joint_signal_);
}

void ParallelTurnController::brake() {
  joint_signal_.velocity = {0, 0, 0, 0, 0, 0, 0, 0};
  joint_signal_.position = {0, 0, 0, 0, 0, 0, 0, 0};
  joint_signal_pub_.publish(joint_signal_);
  ros::Duration(1.0).sleep();
}