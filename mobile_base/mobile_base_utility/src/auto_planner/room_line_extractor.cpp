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

void RoomLineExtractor::updateVirtualPic(double* ps, const int& ps_size) {
  if (!init_pic_) {
    std::cerr << "Please initialize virtual picture first" << std::endl;
    return;
  }

  int point_num = ps_size / 2;
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
}

LineParamVec RoomLineExtractor::sortLines(const Pose2d& pose,
                                          const LineParamVec& param_vec) {}

}  // namespace mobile_base
