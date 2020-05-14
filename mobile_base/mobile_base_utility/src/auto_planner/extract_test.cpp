#include <fstream>
#include <iostream>
#include <string>

#include "mobile_base_utility/data_visualizer.h"
#include "mobile_base_utility/room_line_extractor.h"
#include "mobile_base_utility/wall_follow_ros.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

using mobile_base::LineParam;
using mobile_base::LineParamVec;

bool loadFile(const std::string& addr, std::vector<double>& ps) {
  std::ifstream in;
  in.open(addr);

  ps.clear();
  if (in.is_open()) {
    while (!in.eof()) {
      double x, y, z;
      in >> x >> y;
      ps.push_back(x);
      ps.push_back(y);
    }
  } else {
    std::cerr << "Open file failure" << std::endl;
    std::cerr << "File address is : " << addr << std::endl;
    return false;
  }

  in.close();
  return true;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "extract_test");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  mobile_base::DataVisualizer visualizer;
  visualizer.getNodeHandle(nh);

  mobile_base::RoomLineExtractor extractor;

  tf2_ros::Buffer bf(ros::Duration(0.1));
  tf2_ros::TransformListener tfl(bf);
  mobile_base::WallFollowROS follow(nh, nh_private, bf);

  int step_size, cTh, min_fit_points_num;
  double dTh, min_wall_size, min_ang, min_neig;
  nh_private.param("step_size", step_size, 5);
  nh_private.param("countThreshold", cTh, 2);
  nh_private.param("disThreshold", dTh, 0.05);
  nh_private.param("min_wall_size", min_wall_size, 0.6);
  nh_private.param("min_fit_points_num", min_fit_points_num, 3);
  nh_private.param("min_angle_dis", min_ang, 0.1);
  nh_private.param("min_neighbour_dis", min_neig, 0.05);
  extractor.setExtractorParam(step_size, dTh, cTh, min_wall_size,
                              min_fit_points_num, min_ang, min_neig);

  std::string addr;
  nh_private.param("file_address", addr, std::string(""));

  std::vector<double> ps;
  loadFile(addr, ps);
  extractor.updateScanCloud(ps);

  LineParamVec lines = extractor.extract();
  mobile_base::Pose2dVec poses =
      follow.getWayPoints(lines, mobile_base::Pose2d(0, 0, 0));

  ros::Rate r(2);
  while (ros::ok()) {
    visualizer.showScanCloud(extractor.getScanCloud());
    visualizer.showWalls(lines);
    mobile_base::Pose2dVec vec_tmp;
    for (size_t i = 0; i < poses.size(); i++) {
      vec_tmp.push_back(poses[i]);
      visualizer.showWayPoints(vec_tmp);
      ros::Duration(0.5).sleep();
    }

    ros::spinOnce();
    r.sleep();
  }

  return 0;
}