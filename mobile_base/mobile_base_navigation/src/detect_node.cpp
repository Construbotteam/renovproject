#include "mobile_base_navigation/WallDetect.h"
#include "mobile_base_navigation/LSD.h"
#include "nav_msgs/OccupancyGrid.h"
#include <string>
#include <fstream>

void testCallback(const nav_msgs::OccupancyGrid& data) {
  line::image_double pixel_map = new line::image_double_s;
  pixel_map->xsize = data.info.width;
  pixel_map->ysize = data.info.height;

  pixel_map->data = new double[pixel_map->xsize * pixel_map->ysize];
  for (size_t i = 0; i < data.data.size(); i++) {
    if (0 == data.data[i]) {
      pixel_map->data[i] = (double)data.data[i];
    } else if (100 == data.data[i]) {
      pixel_map->data[i] = (double)data.data[i];
    } else {
      pixel_map->data[i] = 100.0;
    }
  }
  
  std::cout << "========" << std::endl;
  line::ntuple_list list = new line::ntuple_list_s;
  list = line::lsd(pixel_map);

  std::ofstream out;
  out.open("/home/glh/Desktop/line.txt");
  for (size_t i = 0; i < list->size; i++) {
    for (size_t j = 0; j < list->dim; j++) {
      out << list->values[j + i * list->dim] << "  ";
    }
    out << std::endl;
  }
  out.close();
  delete [] pixel_map->data;
  delete pixel_map;
  delete list;
  // std::cout << sizeof(list->values)/sizeof(double) << std::endl;
  std::cout << "========" << std::endl;

}

int main(int argc, char** argv) {
  ros::init(argc, argv, "detect_node");
  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("map", 100, testCallback);
  ros::spin();

  return 0;
}