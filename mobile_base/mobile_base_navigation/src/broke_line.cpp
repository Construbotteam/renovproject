#include "mobile_base_navigation/broke_line.h"
#include <fstream>

using mobile_base::BrokeLine;

int sign(const double& num) { return num >= 0 ? 1 : -1; }

void BrokeLine::initialize(costmap_2d::Costmap2DROS* costmap,
                           ros::NodeHandle& nh) {
  costmap_ros_ = costmap;

  line_path_pub_ = nh.advertise<nav_msgs::Path>("line_path", 10);
  initialize_ = true;
}

bool BrokeLine::breakPath(const std::vector<geometry_msgs::PoseStamped>& path,
                          std::vector<geometry_msgs::PoseStamped>& new_path) {
  new_path.clear();

  new_path.push_back(path[0]);
  costmap_2d::Costmap2D* costmap = costmap_ros_->getCostmap();
  double reso = costmap->getResolution();

  size_t index = 1;
  size_t path_size = path.size();
  geometry_msgs::PoseStamped start = path[0];
  while (index < path_size) {
    if (acrossObstacle(start, path[index], costmap)) {
      new_path.push_back(path[index - 1]);
      start = path[index - 1];
    }
    index++;
  }
  new_path.push_back(path.back());

  std::vector<geometry_msgs::PoseStamped> new_path_tmp = new_path;
  new_path.clear();

  size_t path_index = 0;
  start = new_path_tmp[0];
  // new_path.push_back(start);
  while (path_index < new_path_tmp.size() - 1) {
    geometry_msgs::PoseStamped pose_tmp;
    pose_tmp.header = path[0].header;

    Eigen::Vector2d vec_tmp;
    vec_tmp << new_path_tmp[path_index + 1].pose.position.x -
                   new_path_tmp[path_index].pose.position.x,
        new_path_tmp[path_index + 1].pose.position.y -
            new_path_tmp[path_index].pose.position.y;
    double pose_yaw = getYaw(vec_tmp);

    int interp_num = std::ceil(vec_tmp.norm() / reso);
    for (size_t i = 0; i < interp_num; i++) {
      pose_tmp.pose.position.x =
          start.pose.position.x + i * reso * cos(pose_yaw);
      pose_tmp.pose.position.y =
          start.pose.position.y + i * reso * sin(pose_yaw);
      new_path.push_back(pose_tmp);
    }

    path_index++;

    start = new_path_tmp[path_index];
  }
  new_path.push_back(new_path_tmp.back());
  setDirection(new_path);

  // publish the broken line path
  nav_msgs::Path line_path;
  line_path.header.frame_id = "map";
  for (size_t i = 0; i < new_path.size(); i++) {
    geometry_msgs::PoseStamped pose_stamped_tmp;
    pose_stamped_tmp.header = new_path[i].header;
    pose_stamped_tmp.pose = new_path[i].pose;

    line_path.poses.push_back(pose_stamped_tmp);
  }

  line_path_pub_.publish(line_path);
  return true;
}

double BrokeLine::getYaw(const Eigen::Vector2d& vec) {
  Eigen::Vector2d x_axis(1.0, 0.0);

  double yaw;
  yaw = sign(vec(1)) * fabs(acos(vec.dot(x_axis) / vec.norm()));
  return yaw;
}

bool BrokeLine::acrossObstacle(const geometry_msgs::PoseStamped& from,
                               const geometry_msgs::PoseStamped& to,
                               costmap_2d::Costmap2D* costmap) {
  if (!initialize_ || costmap == NULL) {
    ROS_WARN("Please initialize first or the costmap should not be empty");
    return true;
  }

  double fx, fy, tx, ty, resolution;
  fx = from.pose.position.x;
  fy = from.pose.position.y;
  tx = to.pose.position.x;
  ty = to.pose.position.y;
  resolution = costmap->getResolution();

  double yaw = getYaw(Eigen::Vector2d(tx - fx, ty - fy));

  int incre_num = std::ceil(hypot(tx - fx, ty - fy) / resolution);
  for (size_t i = 1; i < incre_num; i++) {
    uint mx, my;
    costmap->worldToMap(fx + i * resolution * cos(yaw),
                        fy + i * resolution * sin(yaw), mx, my);
    u_char cost = costmap->getCost(mx, my);
    if (cost != costmap_2d::FREE_SPACE) {
      return true;
    }
  }

  return false;
}

void BrokeLine::setDirection(std::vector<geometry_msgs::PoseStamped>& path) {
  Eigen::Vector2d vec;
  for (size_t i = 0; i < path.size() - 1; i++) {
    vec << path[i + 1].pose.position.x - path[i].pose.position.x,
        path[i + 1].pose.position.y - path[i].pose.position.y;
    double yaw = getYaw(vec);
    path[i].pose.orientation = tf::createQuaternionMsgFromYaw(yaw);
  }
}