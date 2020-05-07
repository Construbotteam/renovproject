#include "mobile_base_utility/room_line_extractor.h"

namespace mobile_base {

RoomLineExtractor::RoomLineExtractor() : set_rooms_(false), init_pic_(false) {}

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
    double x1, y1, x2, y2;
    x1 = param.start_.x_ = tuple[i * 7 + 0];
    y1 = param.start_.y_ = tuple[i * 7 + 1];
    x2 = param.end_.x_ = tuple[i * 7 + 2];
    y2 = param.end_.y_ = tuple[i * 7 + 3];

    param.e_start_.x_ = x1 + 1.0;
    param.e_end_.x_ = x2 + 1.0;
    if (fabs(param.end_.y_ - param.start_.y_) < 1e-5) {
      param.e_start_.x_ = x1;
      param.e_start_.y_ = y1 + 1.0;
      param.e_end_.x_ = x2;
      param.e_end_.y_ = y2 + 1.0;
    } else {
      param.e_start_.y_ = (x1 - x2) / (y2 - y1) + y1;
      param.e_end_.y_ = (x1 - x2) / (y2 - y1) + y2;
    }
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

  std::vector<double> dve;

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
  Eigen::Vector2d v1(x - param.start_.x_, y - param.start_.y_);
  Eigen::Vector2d v2(param.end_.x_ - param.start_.x_,
                     param.end_.y_ - param.start_.y_);

  double right_angle_edge = v1.dot(v2) / v2.norm();
  double dis = sqrt(pow(v1.norm(), 2) - pow(right_angle_edge, 2));

  return dis;
}

}  // namespace mobile_base
