#ifndef ROOM_LINE_EXTRACTOR
#define ROOM_LINE_EXTRACTOR

#include <algorithm>
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "ros/ros.h"

namespace mobile_base {

struct Pose2d {
  Pose2d(const double& x, const double& y, const double& th)
      : x_(x), y_(y), th_(th) {}
  Pose2d() : x_(0.0), y_(0.0), th_(0.0) {}
  double x_;
  double y_;
  double th_;

  //   typedef std::shared_ptr<Pose2d> Ptr;
};  // struct Pose2d
typedef std::vector<Pose2d> Pose2dVec;

// This structure contains fours parts of a 2d straight line
/* The two endpoints of this line
 * Another end point of a line which is perpendicular to this line at start
 * Another end point of a line which is perpendicular to this line at end
 */
struct LineParam {
  void reverse() {
    Pose2d tmp = start_;
    start_ = end_;
    end_ = tmp;

    tmp = e_start_;
    e_start_ = e_end_;
    e_end_ = tmp;
  }
  void complete() {
    double x1, y1, x2, y2;
    x1 = start_.x_;
    y1 = start_.y_;
    x2 = end_.x_;
    y2 = end_.y_;
    e_start_.x_ = x1 + 1.0;
    e_end_.x_ = x2 + 1.0;
    if (fabs(end_.y_ - start_.y_) < 1e-5) {
      e_start_.x_ = x1;
      e_start_.y_ = y1 + 1.0;
      e_end_.x_ = x2;
      e_end_.y_ = y2 + 1.0;
    } else {
      e_start_.y_ = (x1 - x2) / (y2 - y1) + y1;
      e_end_.y_ = (x1 - x2) / (y2 - y1) + y2;
    }
  }
  Pose2d start_, end_;
  Pose2d e_start_, e_end_;
};  // struct LineParam
typedef std::vector<LineParam> LineParamVec;

struct VirtualPic {
  int size_x_, size_y_;
  double ox_, oy_;
  double reso_;
  double* data_;
};  // struct VirtualPic

class RoomLineExtractor {
 public:
  RoomLineExtractor();
  virtual ~RoomLineExtractor();
  bool setRooms(const int& num, Pose2d* centers);
  void initVirtualPic(const int& sx, const int& sy, const double& ox,
                      const double& oy, const double& reso);
  void updateVirtualPic(const std::vector<double> ps);
  void resetVirtualPic();
  LineParamVec computeWalls(const int& num, double* tuple, const Pose2d& pose);
  LineParamVec sortLines(const Pose2d& pose, const LineParamVec& param_vec);
  double point2LineDistance(const double& x, const double& y,
                            const LineParam& param);
  void updateScanCloud(const std::vector<double>& ps);
  void resetScanCloud();
  void filterScanCloud();
  LineParamVec extract();
  LineParam lineFit(const Pose2dVec& points);
  inline bool isWallValid(const LineParam& param, const Pose2dVec& poses) {
    double wall_size =
        hypot(param.start_.x_ - param.end_.x_, param.start_.y_ - param.end_.y_);

    bool valid =
        (wall_size > min_wall_size_ && poses.size() > min_fit_points_num_);
    if (!valid) {
      printf(
          "This wall is invalid since wall-size : %.3f (min : %.3f) and "
          "points-num : %d (min : %d",
          wall_size, min_wall_size_, poses.size(), min_fit_points_num_);
    }

    return valid;
  }

  static inline bool cloudCompare(const Pose2d& p1, const Pose2d& p2) {
    return p1.th_ < p2.th_;
  }
  inline int sign(const double& num) { return num >= 0 ? 1 : -1; }
  inline double shortestAngleDistance(const double& from, const double& to) {
    double angle_dis = to - from;

    if (fabs(angle_dis) > M_PI) {
      angle_dis = -sign(angle_dis) * (2 * M_PI - fabs(angle_dis));
    }

    return angle_dis;
  }

  void setExtractorParam(const int& step_size, const double& disThreshold,
                         const int& countThreshold, const double& min_wall_size,
                         const int& min_fit_points_num,
                         const double& min_angle_dis,
                         const double& min_neighbour_dis);
  VirtualPic getVirtualPic() const { return virtual_pic_; }
  int getRoomQuantity() const { return room_num_; }
  Pose2dVec getScanCloud() const { return scan_cloud_; }

 private:
  int room_num_, step_size_, countThreshold_;
  bool set_rooms_;
  bool init_pic_;
  double disThreshold_, min_wall_size_;
  double min_angle_dis_, min_neighbour_dis_;
  int min_fit_points_num_;
  Pose2dVec scan_cloud_;

  VirtualPic virtual_pic_;
  Pose2d* centers_;

};  // class LineExtractor

}  // namespace mobile_base

#endif