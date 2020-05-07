#ifndef ROOM_LINE_EXTRACTOR
#define ROOM_LINE_EXTRACTOR

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

  VirtualPic getVirtualPic() const { return virtual_pic_; }
  int getRoomQuantity() const { return room_num_; }

 private:
  int room_num_;
  bool set_rooms_;
  bool init_pic_;

  VirtualPic virtual_pic_;
  Pose2d* centers_;

};  // class LineExtractor

}  // namespace mobile_base

#endif