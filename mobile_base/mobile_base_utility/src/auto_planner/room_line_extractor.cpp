#include "mobile_base_utility/room_line_extractor.h"

namespace mobile_base {

RoomLineExtractor::RoomLineExtractor() : set_rooms_(false), init_pic_(false) {
  scan_cloud_.clear();
}

RoomLineExtractor::~RoomLineExtractor() {
  if (centers_) {
    delete[] centers_;
  }

  if (virtual_pic_.data_) {
    delete[] virtual_pic_.data_;
  }
}

bool RoomLineExtractor::setRooms(const int& num, Pose2d* centers) {
  room_num_ = num;
  if (room_num_ < 1) {
    set_rooms_ = false;
    return false;
  }

  centers_ = new Pose2d[num];
  centers_ = centers;

  set_rooms_ = true;

  return true;
}

void RoomLineExtractor::initVirtualPic(const int& sx, const int& sy,
                                       const double& ox, const double& oy,
                                       const double& reso) {
  virtual_pic_.size_x_ = sx;
  virtual_pic_.size_y_ = sy;
  virtual_pic_.ox_ = ox;
  virtual_pic_.oy_ = oy;
  virtual_pic_.reso_ = reso;

  int data_size = virtual_pic_.size_x_ * virtual_pic_.size_y_;
  virtual_pic_.data_ = new double[data_size];
  for (size_t i = 0; i < data_size; i++) {
    virtual_pic_.data_[i] = 0.0;
  }

  init_pic_ = true;
}

void RoomLineExtractor::updateVirtualPic(const std::vector<double> ps) {
  if (!init_pic_) {
    std::cerr << "Please initialize virtual picture first" << std::endl;
    return;
  }

  int point_num = ps.size() / 2;
  for (size_t i = 0; i < point_num; i++) {
    int ox_cell = std::floor(fabs(virtual_pic_.ox_) / virtual_pic_.reso_);
    int oy_cell = std::floor(fabs(virtual_pic_.oy_) / virtual_pic_.reso_);

    // use floor or ceil here ???
    int dx_cell = std::floor(ps[i * 2] / virtual_pic_.reso_);
    int dy_cell = std::floor(ps[i * 2 + 1] / virtual_pic_.reso_);

    int px_cell = ox_cell + dx_cell;
    int py_cell = oy_cell + dy_cell;

    virtual_pic_.data_[px_cell + py_cell * virtual_pic_.size_x_] = 100.0;
  }
}

void RoomLineExtractor::resetVirtualPic() {
  if (init_pic_) {
    int data_size = virtual_pic_.size_x_ * virtual_pic_.size_y_;
    for (size_t i = 0; i < data_size; i++) {
      virtual_pic_.data_[i] = 0.0;
    }
  }
}

LineParamVec RoomLineExtractor::computeWalls(const int& num, double* tuple,
                                             const Pose2d& pose) {
  LineParamVec line_params;

  line_params.clear();
  for (size_t i = 0; i < num; i++) {
    LineParam param;
    param.start_.x_ = tuple[i * 7 + 0];
    param.start_.y_ = tuple[i * 7 + 1];
    param.end_.x_ = tuple[i * 7 + 2];
    param.end_.y_ = tuple[i * 7 + 3];

    param.complete();
    line_params.push_back(param);
  }

  return sortLines(pose, line_params);
}

LineParamVec RoomLineExtractor::sortLines(const Pose2d& pose,
                                          const LineParamVec& param_vec) {
  bool searched[param_vec.size()];
  for (size_t i = 0; i < param_vec.size(); i++) {
    searched[i] = false;
  }

  LineParamVec sorted_vec;
  sorted_vec.clear();
  // find the closest line w.r.t. the pose input
  double dis_min = std::numeric_limits<double>::max();
  int closest_line_index = 0;
  for (size_t i = 0; i < param_vec.size(); i++) {
    double dis_tmp = point2LineDistance(pose.x_, pose.y_, param_vec[i]);
    if (dis_tmp < dis_min) {
      dis_min = dis_tmp;
      closest_line_index = i;
    }
  }
  sorted_vec.push_back(param_vec[closest_line_index]);
  searched[closest_line_index] = true;

  double dis_start = hypot(pose.x_ - param_vec[closest_line_index].start_.x_,
                           pose.y_ - param_vec[closest_line_index].start_.y_);
  double dis_end = hypot(pose.x_ - param_vec[closest_line_index].end_.x_,
                         pose.y_ - param_vec[closest_line_index].end_.y_);
  if (dis_start > dis_end) {
    sorted_vec[0].reverse();
  }

  int cur_index = closest_line_index;
  while (sorted_vec.size() < param_vec.size()) {
    double point_dis_min = std::numeric_limits<double>::max();
    int index_tmp;
    bool reverse_param = false;
    for (size_t i = 0; i < param_vec.size(); i++) {
      if (!searched[i] && (i != cur_index)) {
        double point_dis_start =
            hypot(sorted_vec.back().end_.x_ - param_vec[i].start_.x_,
                  sorted_vec.back().end_.y_ - param_vec[i].start_.y_);
        double point_dis_end =
            hypot(sorted_vec.back().end_.x_ - param_vec[i].end_.x_,
                  sorted_vec.back().end_.y_ - param_vec[i].end_.y_);

        if (point_dis_start < point_dis_end &&
            point_dis_start < point_dis_min) {
          reverse_param = false;
          point_dis_min = point_dis_start;
          index_tmp = i;
        }

        if (point_dis_start > point_dis_end && point_dis_end < point_dis_min) {
          reverse_param = true;
          point_dis_min = point_dis_end;
          index_tmp = i;
        }
      }  // endif of one single iteration to look for a closest line endpoint
    }

    searched[index_tmp] = true;
    sorted_vec.push_back(param_vec[index_tmp]);
    if (reverse_param) {
      sorted_vec.back().reverse();
    }
  }

  return sorted_vec;
}

double RoomLineExtractor::point2LineDistance(const double& x, const double& y,
                                             const LineParam& param) {
  Eigen::Vector3d v1(x - param.start_.x_, y - param.start_.y_, 0);
  Eigen::Vector3d v2(param.end_.x_ - param.start_.x_,
                     param.end_.y_ - param.start_.y_, 0);

  // area of parallelogram equals to base times height
  double dis = (v1.cross(v2)).norm() / v2.norm();

  return dis;
}

void RoomLineExtractor::setExtractorParam(
    const int& step_size, const double& disThreshold, const int& countThreshold,
    const double& min_wall_size, const int& min_fit_points_num,
    const double& min_angle_dis, const double& min_neighbour_dis) {
  if (step_size > 0) {
    step_size_ = step_size;
  } else {
    std::cerr << "Step size should be greater than 1" << std::endl;
    step_size_ = 1;
  }

  disThreshold_ = disThreshold;
  countThreshold_ = std::min(countThreshold, step_size_);
  min_wall_size_ = min_wall_size;
  min_fit_points_num_ = min_fit_points_num;

  min_angle_dis_ = min_angle_dis;
  min_neighbour_dis_ = min_neighbour_dis;
}

void RoomLineExtractor::updateScanCloud(const std::vector<double>& ps) {
  for (size_t i = 0; i < ps.size() / 2; i++) {
    Pose2d pose;
    pose.x_ = ps[2 * i + 0];
    pose.y_ = ps[2 * i + 1];
    scan_cloud_.push_back(pose);
  }
}

void RoomLineExtractor::resetScanCloud() { scan_cloud_.clear(); }

void RoomLineExtractor::filterScanCloud() {
  Pose2dVec filter_cloud;
  filter_cloud.clear();

  int id = 1;
  Pose2d cur_point = scan_cloud_[0];
  while (id < scan_cloud_.size()) {
    double neighbour_dis = hypot(cur_point.x_ - scan_cloud_[id].x_,
                                 cur_point.y_ - scan_cloud_[id].y_);

    double cur_ang = atan2(cur_point.y_, cur_point.x_);
    double next_ang = atan2(scan_cloud_[id].y_, scan_cloud_[id].x_);
    double neighbour_angle_dis = shortestAngleDistance(cur_ang, next_ang);

    if (neighbour_angle_dis > min_angle_dis_ &&
        neighbour_dis > min_neighbour_dis_) {
      filter_cloud.push_back(scan_cloud_[id]);
      cur_point = scan_cloud_[id];
    }
    id++;
  }

  scan_cloud_.clear();
  scan_cloud_ = Pose2dVec(filter_cloud.begin(), filter_cloud.end());
}

LineParam RoomLineExtractor::lineFit(const Pose2dVec& points) {
  int points_size = points.size();
  Eigen::VectorXd b_vec(points_size);
  Eigen::MatrixXd mat = Eigen::MatrixXd::Ones(points_size, 2);
  for (size_t i = 0; i < points_size; i++) {
    b_vec(i) = points[i].y_;
    mat(i, 0) = points[i].x_;
  }

  double k, b;
  Eigen::Vector2d analytic =
      (mat.transpose() * mat).inverse() * mat.transpose() * b_vec;
  k = analytic(0);
  b = analytic(1);

  LineParam fit_param;
  fit_param.start_.x_ = points[0].x_;
  fit_param.start_.y_ = k * points[0].x_ + b;
  fit_param.end_.x_ = points.back().x_;
  fit_param.end_.y_ = k * points.back().x_ + b;

  return fit_param;
}

LineParamVec RoomLineExtractor::extract() {
  LineParamVec walls;
  walls.clear();
  // sort the cloud
  for (size_t i = 0; i < scan_cloud_.size(); i++) {
    scan_cloud_[i].th_ = atan2(scan_cloud_[i].y_, scan_cloud_[i].x_);
  }
  std::sort(scan_cloud_.begin(), scan_cloud_.end(),
            RoomLineExtractor::cloudCompare);

  filterScanCloud();
  // compute lines
  Pose2dVec fit_points;
  fit_points.push_back(scan_cloud_[0]);
  fit_points.push_back(scan_cloud_[1]);
  int index = 1;
  while (index < scan_cloud_.size()) {
    LineParam fit_param = lineFit(fit_points);

    // count numbers of points that exceed the distance threshold
    int ex_count = 0;
    int rest_points_count = scan_cloud_.size() - 1 - index;
    for (size_t i = 0; i < std::min(step_size_, rest_points_count); i++) {
      double dis_tmp =
          point2LineDistance(scan_cloud_[index + i + 1].x_,
                             scan_cloud_[index + i + 1].y_, fit_param);
      if (dis_tmp > disThreshold_) {
        ex_count++;
      } else {
        fit_points.push_back(scan_cloud_[index + i + 1]);
      }
    }

    if (step_size_ >= rest_points_count) {
      fit_param = lineFit(fit_points);
      if (isWallValid(fit_param, fit_points)) {
        fit_param.complete();
        walls.push_back(fit_param);
      }
      break;
    } else if (ex_count > countThreshold_) {
      fit_param = lineFit(fit_points);
      if (isWallValid(fit_param, fit_points)) {
        fit_param.complete();
        walls.push_back(fit_param);
      }

      index = index + 1 + (step_size_ - ex_count);
      fit_points.clear();
      fit_points.push_back(scan_cloud_[index]);
      fit_points.push_back(scan_cloud_[index + 1]);

      index++;
    } else {
      index += step_size_;
    }
  }
  // connect the first and last lines if neccessary
  /* TO DO */
  // return lines
  return walls;
}

}  // namespace mobile_base
