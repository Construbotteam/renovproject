#include <yaml-cpp/yaml.h>
#include <fstream>
#include "map_server/image_loader.h"
#include "nav_msgs/GetMap.h"
#include "ros/ros.h"

void loadMap(const std::string& map_addr, ros::NodeHandle& nh) {
  YAML::Node map_config = YAML::LoadFile(map_addr);

  const char* fname =
      "/home/glh/renov_ws/src/renov_robot/mobile_base_utility/map/510.pgm";
  double res = map_config["resolution"].as<double>();
  double negate = map_config["negate"].as<double>();
  double occ_th = 100.0;
  double free_th = 0.0;
  double* origin = new double[3];
  origin[0] = map_config["origin"][0].as<double>();
  origin[1] = map_config["origin"][1].as<double>();
  origin[2] = map_config["origin"][2].as<double>();

  ROS_INFO(
      "fname : %s, res : %.5f, negate : %.2f, occ_th : %.2f, free_th : %.2f, "
      "origin : [%.4f, %.4f, %.4f]",
      fname, res, negate, occ_th, free_th, origin[0], origin[1], origin[2]);

  nav_msgs::GetMap::Response response;

  try {
    map_server::loadMapFromFile(&response, fname, res, negate, occ_th, free_th,
                                origin, RAW);
  } catch (const std::runtime_error& e) {
    ROS_ERROR("%s", e.what());
    return;
  }
  delete[] origin;

  response.map.info.map_load_time = ros::Time::now();
  response.map.header.frame_id = "map";
  response.map.header.stamp = ros::Time::now();
  std::cout << response.map.info.width << std::endl;
  std::cout << response.map.info.height << std::endl;

  std::ofstream out("/home/glh/Desktop/bim_map.txt");
  for (size_t i = 0; i < response.map.data.size(); i++) {
    out << (int)response.map.data[i] << std::endl;
  }
  out.close();

  // ros::Publisher map_pub = nh.advertise<nav_msgs::OccupancyGrid>("map", 10);
  // ros::Publisher map_meta_pub =
  //     nh.advertise<nav_msgs::MapMetaData>("map_metadata", 10);

  // ros::Rate r(10);
  // while (ros::ok()) {
  //   map_pub.publish(response.map);
  //   map_meta_pub.publish(response.map.info);
  //   ros::spinOnce();
  //   r.sleep();
  // }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "bim_map2scan");
  ros::NodeHandle nh;

  std::string map_addr =
      "/home/glh/renov_ws/src/renov_robot/mobile_base_utility/map/510.yaml";
  loadMap(map_addr, nh);
  return 0;
}