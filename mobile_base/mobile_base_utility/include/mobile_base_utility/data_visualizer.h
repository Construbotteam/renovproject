#ifndef DATA_VISUALIZER_H
#define DATA_VISUALIZER_H

#include "geometry_msgs/PoseArray.h"
#include "mobile_base_utility/room_line_extractor.h"
#include "nav_msgs/OccupancyGrid.h"
#include "nav_msgs/Path.h"
#include "ros/ros.h"
#include "tf/tf.h"
#include "visualization_msgs/Marker.h"

namespace mobile_base {

class DataVisualizer {
 public:
  DataVisualizer(ros::NodeHandle& nh);
  DataVisualizer() {}
  virtual ~DataVisualizer() {}
  void getNodeHandle(ros::NodeHandle& nh);
  void showWalls(const LineParamVec& lines);
  void showVirtualPic(const VirtualPic& pic);
  void showWayPoints(const Pose2dVec& poses);

 private:
  ros::Publisher wall_pub_, pic_pub_, way_points_pub_;
};  // class DataVisualizer

}  // namespace mobile_base

#endif