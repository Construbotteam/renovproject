#include "mobile_base_navigation/WallDetect.h"

using mobile::WallDetect;

WallDetect::WallDetect() {
  map_size_now = 0;
  n_private = ros::NodeHandle("detect");
}

void WallDetect::ParamInit() {
  if (!n_private.getParam("update_dif", update_dif)) {
    update_dif =  20;
  }
}


void WallDetect::GetMapCallback(const nav_msgs::OccupancyGrid& map_msg) {
  
  int map_size = 0;
  for (size_t i = 0; i < map_msg.data.size(); i++) {
    if (100 == map_msg.data[i]) {
      map_size++;
    }
  }

  int size_dif = abs(map_size_now - map_size);
  if (size_dif < update_dif) {
    ROS_INFO("map difference is too small to update"); 
  } else {
    map_size_now = map_size;

    line::image_double pixel_map = new line::image_double_s;
    pixel_map->xsize = map_msg.info.width;
    pixel_map->ysize = map_msg.info.height;

    //  transform occupancy grid map to pixel map
    pixel_map->data = new double[pixel_map->xsize * pixel_map->ysize];
    for (size_t i = 0; i < map_msg.data.size(); i++) {
      if (0 == map_msg.data[i]) {
        pixel_map->data[i] = (double)map_msg.data[i];
      } else if (100 == map_msg.data[i]) {
        pixel_map->data[i] = (double)map_msg.data[i];
      } else {
        pixel_map->data[i] = 100.0;
      }
    }

    line::ntuple_list list = new line::ntuple_list_s;
    list = line::lsd(pixel_map);

    delete[] pixel_map->data;
    delete pixel_map;
    delete list;
  }

  
}