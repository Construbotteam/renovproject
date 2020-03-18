#include "mobile_base_navigation/velo_generator.h"

using mobile_base::VeloGenerator;

VeloGenerator::VeloGenerator(const double &v_max, const double &v_min,
                             const double &acc_max,
                             costmap_2d::Costmap2DROS *costmap_ros)
    : v_max_(v_max),
      v_min_(v_min),
      acc_max_(acc_max),
      costmap_ros_(costmap_ros) {
  base_outline_ = costmap_ros_->getRobotFootprintPolygon();
}

bool VeloGenerator::planGlobalVelo(
    const std::vector<geometry_msgs::PoseStamped> &plan,
    std::vector<double> &velo_plan) {
  // update the global plan
  global_plan_.clear();
  for (size_t i = 0; i < plan.size(); i++) {
    global_plan_.push_back(plan[i]);
  }

  return linearBrake(plan, velo_plan);
}

bool VeloGenerator::planLocalVelo(const int &index,
                                  const geometry_msgs::PoseStamped &global_pose,
                                  std::vector<double> &velo_plan) {
  if (global_plan_.size() < 4) {
    ROS_INFO(
        "Stop computing local velocity plan. The rest of path is too short, "
        "path size is : %d",
        (int)global_plan_.size());
    return false;
  }

  costmap_2d::Costmap2D *costmap_ptr = costmap_ros_->getCostmap();
  double dis_delta = costmap_ptr->getResolution();
  // std::vector<Eigen::Vector3d> points_in_polygon_base;
  std::vector<Eigen::Vector3d> points_in_polygon_map;
  // points_in_polygon_base.clear();
  points_in_polygon_map.clear();

  double dis_tmp, max_dis_tmp;
  double farest_x, farest_y;
  max_dis_tmp = std::numeric_limits<double>::min();
  for (size_t i = 0; i < base_outline_.points.size(); i++) {
    dis_tmp = sqrt(pow(base_outline_.points[i].x, 2) +
                   pow(base_outline_.points[i].y, 2));
    if (dis_tmp > max_dis_tmp) {
      max_dis_tmp = dis_tmp;
      farest_x = base_outline_.points[i].x;
      farest_y = base_outline_.points[i].y;
    }
  }
  int x_indices_num, y_indices_num;
  x_indices_num = (int)(2 * fabs(farest_x) / dis_delta) + 1;
  y_indices_num = (int)(2 * fabs(farest_y) / dis_delta) + 1;

  int collision_index;
  bool collision_on_path = false;
  size_t forward_index = index + 4;
  forward_index = std::min(forward_index, global_plan_.size());
  for (size_t k = index; k < forward_index; k++) {
    Eigen::Quaterniond global_pose_quat(
        global_plan_[k].pose.orientation.w, global_plan_[k].pose.orientation.x,
        global_plan_[k].pose.orientation.y, global_plan_[k].pose.orientation.z);
    Eigen::Matrix3d global_pose_matrix = global_pose_quat.toRotationMatrix();
    global_pose_matrix(0, 2) = global_plan_[k].pose.position.x;
    global_pose_matrix(1, 2) = global_plan_[k].pose.position.y;

    for (size_t i = 0; i < x_indices_num; i++) {
      for (size_t j = 0; j < y_indices_num; j++) {
        double x_tmp = -fabs(farest_x) + i * dis_delta;
        double y_tmp = -fabs(farest_y) + j * dis_delta;
        Eigen::Vector3d point_tmp(x_tmp, y_tmp, 0);
        points_in_polygon_map.push_back(global_pose_matrix * point_tmp);
      }
    }  // end of loop computing points locating in polygon field w.r.t. map

    int collision_count = 0;
    for (size_t i = 0; i < points_in_polygon_map.size(); i++) {
      uint mx, my;
      if (!costmap_ptr->worldToMap(points_in_polygon_map[i](0),
                                   points_in_polygon_map[i](1), mx, my)) {
        ROS_WARN("Failure! Velo generator transforms point from world to map");
        continue;
      }
      u_char cost_value = costmap_ptr->getCost(mx, my);
      if (cost_value >= costmap_2d::LETHAL_OBSTACLE) {
        collision_count++;
      }
    }

    if (collision_count >= 4) {
      collision_index = k;
      collision_on_path = true;
      break;
    }
  }  // end of loop iterating all path points to judge collision

  if (collision_on_path) {
    std::vector<geometry_msgs::PoseStamped> brake_plan;
    brake_plan.clear();
    for (size_t i = index; i < collision_index; i++) {
      brake_plan.push_back(global_plan_[i]);
    }
    std::vector<double> brake_velo;
    brake_velo.clear();
    if (!linearBrake(brake_plan, brake_velo)) {
      ROS_WARN("Failue! Plan velocities to brake");
      return false;
    }
    for (size_t i = index; i < collision_index; i++) {
      velo_plan[i] = brake_velo[i - index];
    }
  } else {
    linearBrake(global_plan_, velo_plan);
  }

  return true;
}

void VeloGenerator::getMaxVelo(const double &v) {
  /**/
  v_max_ = v;
}
void VeloGenerator::getMaxAcc(const double &acc) {
  /**/
  acc_max_ = acc;
}

bool VeloGenerator::linearBrake(
    const std::vector<geometry_msgs::PoseStamped> &plan,
    std::vector<double> &velo_plan) {
  if (plan.size() < 2) {
    ROS_WARN("Size of plan is too short for VeloGenerator, size is : %d",
             (int)plan.size());
    return false;
  }

  double brake_dis = v_max_ * v_max_ / 2.0 / acc_max_;
  double x1, y1, x2, y2;
  x1 = plan[0].pose.position.x;
  y1 = plan[0].pose.position.y;
  x2 = plan[1].pose.position.x;
  y2 = plan[1].pose.position.y;
  double path_point_dis = sqrt(pow(x1 - x2, 2) + pow(y1 - y2, 2));

  int brake_ind_num = (int)(brake_dis / path_point_dis) + 1;
  brake_ind_num = std::min((int)plan.size(), brake_ind_num);

  // some bugs here
  if (velo_plan.size() == 0) {
    velo_plan.resize(global_plan_.size());
  }

  int velo_plan_size = velo_plan.size();
  for (size_t i = 0; i < velo_plan_size; i++) {
    velo_plan[i] = v_max_;
  }
  for (size_t i = 0; i < brake_ind_num; i++) {
    velo_plan[velo_plan_size - 1 - i] = v_min_ + i * acc_max_;
    if (velo_plan[velo_plan_size - 1 - i] > v_max_) {
      velo_plan[velo_plan_size - 1 - i] = v_max_;
      break;
    }
  }

  return true;
}
