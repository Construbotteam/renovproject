#include "mobile_base_navigation/path_smoother.h"

namespace mobile_base {

PathSmoother::PathSmoother(const std::string& name) : if_initialized_(false) {
  Initialize(name);
}

void PathSmoother::Initialize(const std::string& name) {
  if (!if_initialized_) {
    ros::NodeHandle nh_private("~/" + name);

    nh_private.param("forward_distance", forward_distance_, 0.2);
    nh_private.param("max_linear_velo", max_linear_velo_, 0.2);
    nh_private.param("max_angular_velo", max_angular_velo_, 0.2);
    nh_private.param("dis_threshold", dis_threshold_, 0.02);
    nh_private.param("delta_t_sec", delta_t_sec_, 0.01);
    nh_private.param("k_coeff", k_coeff_, 0.2);
    nh_private.param("path_multi", path_multi_, 10);

    if_initialized_ = true;
  }
}

bool PathSmoother::UpdatePath(const geometry_msgs::PoseStamped& start,
                              const nav_msgs::Path& old_path,
                              nav_msgs::Path* new_path) {
  if (!if_initialized_) {
    ROS_WARN("This smoother is not initialized. Please initialize first !");
    ROS_WARN("The new path will be equal to old path");
    new_path->header = old_path.header;
    for (size_t i = 0; i < old_path.poses.size(); i++) {
      new_path->poses.push_back(old_path.poses[i]);
    }
    return true;
  }

  nav_msgs::Path pre_new_path;
  pre_new_path.header = old_path.header;
  pre_new_path.poses.clear();
  pre_new_path.poses.push_back(start);
  geometry_msgs::PoseStamped virtual_pose = start;

  while (true) {
    ControlCmd2d control_cmd = VirtualController(virtual_pose, old_path);
    UpdatePose(&virtual_pose, control_cmd);
    // std::cout << "control cmd v : " << control_cmd.v << std::endl;
    // std::cout << "control cmd w : " << control_cmd.w << std::endl;
    // ros::Duration(1.5).sleep();
    pre_new_path.poses.push_back(virtual_pose);

    if (FindDistance(virtual_pose, old_path.poses.back()) < dis_threshold_) {
      break;
    }
    if (pre_new_path.poses.size() > path_multi_ * old_path.poses.size()) {
      ROS_WARN("path smooth failure");
      return false;
    }
  }

  int kkk;
  if (!ros::param::get("kkk", kkk)) {
    kkk = 5;
  }

  new_path->header = pre_new_path.header;
  new_path->poses.clear();
  new_path->poses.push_back(start);
  int multiplier = pre_new_path.poses.size() / old_path.poses.size();
  // std::cout << "mulitplier : " << multiplier << std::endl;

  if (multiplier > 1) {
    int sparse_multiplier = multiplier / kkk;
    while (sparse_multiplier < 1) {
      sparse_multiplier++;
    }
    for (size_t i = 1; i < pre_new_path.poses.size(); i++) {
      if (i % sparse_multiplier == 0) {
        new_path->poses.push_back(pre_new_path.poses[i]);
      }
    }
  } else {
    for (size_t i = 0; i < pre_new_path.poses.size(); i++) {
      new_path->poses.push_back(pre_new_path.poses[i]);
    }
  }

  geometry_msgs::PoseStamped final_point;
  final_point.header = pre_new_path.poses[0].header;
  final_point.pose.orientation = pre_new_path.poses.back().pose.orientation;
  final_point.pose.position = old_path.poses.back().pose.position;

  new_path->poses.push_back(final_point);
  new_path->poses.push_back(old_path.poses.back());

  for (size_t i = 0; i < new_path->poses.size(); i++) {
    double dis_com = 100000000.0;
    for (size_t j = 0; j < old_path.poses.size(); j++) {
      double point_dis = FindDistance(new_path->poses[i], old_path.poses[j]);
      if (point_dis < dis_com) {
        dis_com = point_dis;
      }
    }
    // std::cout << dis_com << std::endl;
    if (dis_com > 2) {
      ROS_ERROR("PATH ERROR");
      return false;
    }
  }
  return true;
}

ControlCmd2d PathSmoother::VirtualController(
    const geometry_msgs::PoseStamped& cur_pose, const nav_msgs::Path& path) {
  int index;
  int cur_forward_dis_index;
  double dis = 100000000.0;
  double cur_forward_dis_diff = 100000000.0;
  double cur_forward_dis;
  // find the closest point on the path
  for (size_t i = 0; i < path.poses.size(); i++) {
    double dis_tmp = FindDistance(cur_pose, path.poses[i]);
    if (dis_tmp < dis) {
      dis = dis_tmp;
      index = i;
    }
  }

  for (size_t i = index; i < path.poses.size(); i++) {
    double dis_tmp = FindDistance(cur_pose, path.poses[i]);
    double dis_diff_tmp = fabs(dis_tmp - forward_distance_);
    if (dis_diff_tmp < cur_forward_dis_diff) {
      cur_forward_dis_diff = dis_diff_tmp;
      cur_forward_dis_index = i;
      cur_forward_dis = dis_tmp;
    }
  }

  // ROS_INFO("forward index = %d,  forward dis = %.7f", cur_forward_dis_index,
  //          cur_forward_dis);
  // ROS_INFO("cur pose index = %d", index);

  geometry_msgs::PoseStamped forward_pose = path.poses[cur_forward_dis_index];
  double forward_yaw =
      atan2(forward_pose.pose.position.y - cur_pose.pose.position.y,
            forward_pose.pose.position.x - cur_pose.pose.position.x);

  tf::Quaternion cur_quat(
      cur_pose.pose.orientation.x, cur_pose.pose.orientation.y,
      cur_pose.pose.orientation.z, cur_pose.pose.orientation.w);
  double cur_yaw = tf::getYaw(cur_quat);

  double yaw_diff = forward_yaw - cur_yaw;
  if (fabs(yaw_diff) > M_PI) {
    if (forward_yaw < 0) {
      yaw_diff = yaw_diff + 2 * M_PI;
    } else {
      yaw_diff = yaw_diff - 2 * M_PI;
    }
  }

  int sign;
  if (yaw_diff > 0) {
    sign = 1;
  } else {
    sign = -1;
  }

  // ROS_INFO("forward yaw = %.4f, cur yaw = %.4f, yaw diff = %.4f, sign = %d",
  //          forward_yaw, cur_yaw, yaw_diff, sign);

  double motion_radius =
      0.5 * cur_forward_dis / cos(fabs(M_PI / 2 - fabs(yaw_diff)));

  ControlCmd2d cmd;
  double rest_d = FindRestDistance(index, path);
  // ROS_INFO("motion radius = %.4f", motion_radius);
  // ROS_INFO("rest dis = %.6f, max linear velo = %.6f, max angular velo =
  // %.6f",
  //  rest_d, max_linear_velo_, max_angular_velo_);

  if (k_coeff_ * rest_d > max_linear_velo_) {
    cmd.v = max_linear_velo_;
  } else {
    cmd.v = k_coeff_ * rest_d;
  }
  if (fabs(cmd.v / motion_radius) > max_angular_velo_) {
    cmd.w = max_angular_velo_;
  } else {
    cmd.w = cmd.v / motion_radius * sign;
  }
  // cmd.v = std::min(k_coeff_ * rest_d, max_linear_velo_);
  // cmd.w = std::min(cmd.v / motion_radius * sign, max_angular_velo_);
  // std::cout << "original cmd v" << cmd.v << std::endl;
  // std::cout << "original cmd w" << cmd.w << std::endl;

  return cmd;
}

void PathSmoother::UpdatePose(geometry_msgs::PoseStamped* pose,
                              const ControlCmd2d& cmd) {
  tf::Quaternion quat(pose->pose.orientation.x, pose->pose.orientation.y,
                      pose->pose.orientation.z, pose->pose.orientation.w);
  double yaw = tf::getYaw(quat);
  pose->pose.position.x += cmd.v * cos(yaw) * delta_t_sec_;
  pose->pose.position.y += cmd.v * sin(yaw) * delta_t_sec_;

  yaw = yaw + cmd.w * delta_t_sec_;
  quat = tf::createQuaternionFromYaw(yaw);
  pose->pose.orientation.x = quat.x();
  pose->pose.orientation.y = quat.y();
  pose->pose.orientation.z = quat.z();
  pose->pose.orientation.w = quat.w();
}

double PathSmoother::FindDistance(const geometry_msgs::PoseStamped& p1,
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

double PathSmoother::FindRestDistance(const int& index,
                                      const nav_msgs::Path& path) {
  double rest_dis = 0;
  for (size_t i = index; i < path.poses.size(); i++) {
    rest_dis += FindDistance(path.poses[index], path.poses[i]);
  }
  return rest_dis;
}

}  // namespace mobile_base
