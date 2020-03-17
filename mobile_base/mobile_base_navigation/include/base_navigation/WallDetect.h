#ifndef WALLDETECT_H
#define WALLDETECT_H

#include "ros/ros.h"
#include "nav_msgs/OccupancyGrid.h"
#include "LSD.h"

namespace mobile {

class WallDetect {
 private:
  /* PARAMETER */
  int update_dif;

  line::image_double pixel_map;
  int map_size_now;
  int map_size;
  

  ros::NodeHandle n_private;

 public:
  WallDetect();
  virtual ~WallDetect() {}
  void ParamInit();
  void GetMapCallback(const nav_msgs::OccupancyGrid& map_msg);
};

}  // namespace mobile







#endif