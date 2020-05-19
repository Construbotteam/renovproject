#include "mobile_base_utility/data_visualizer.h"

namespace mobile_base {

DataVisualizer::DataVisualizer(ros::NodeHandle& nh) {
  //   wall_pub_ = nh.advertise<nav_msgs::Path>("walls", 10);
  wall_pub_ = nh.advertise<visualization_msgs::Marker>("walls", 10);
  pic_pub_ = nh.advertise<nav_msgs::OccupancyGrid>("virtual_pic", 10);
  way_points_pub_ = nh.advertise<geometry_msgs::PoseArray>("way_points", 10);
  scan_cloud_pub_ =
      nh.advertise<visualization_msgs::Marker>("scan_clouds_markers", 10);
}

void DataVisualizer::getNodeHandle(ros::NodeHandle& nh) {
  wall_pub_ = nh.advertise<visualization_msgs::Marker>("walls", 10);
  pic_pub_ = nh.advertise<nav_msgs::OccupancyGrid>("virtual_pic", 10);
  way_points_pub_ = nh.advertise<geometry_msgs::PoseArray>("way_points", 10);
  scan_cloud_pub_ =
      nh.advertise<visualization_msgs::Marker>("scan_clouds_markers", 10);
}

void DataVisualizer::showVirtualPic(const VirtualPic& pic) {
  nav_msgs::OccupancyGrid map;
  map.header.frame_id = "map";
  map.header.stamp = ros::Time::now();

  map.info.width = pic.size_x_;
  map.info.height = pic.size_y_;
  map.info.origin.position.x = pic.ox_;
  map.info.origin.position.y = pic.oy_;
  map.info.resolution = pic.reso_;

  int pix_size = pic.size_x_ * pic.size_y_;
  for (size_t i = 0; i < pix_size; i++) {
    map.data.push_back(pic.data_[i]);
  }

  pic_pub_.publish(map);
  ROS_INFO("I publish virtual picture");
}

void DataVisualizer::showWalls(const LineParamVec& lines) {
  visualization_msgs::Marker marker;
  marker.header.frame_id = "map";
  marker.header.stamp = ros::Time::now();

  marker.ns = "walls";
  marker.id = 0;
  marker.lifetime = ros::Duration();

  marker.type = visualization_msgs::Marker::LINE_LIST;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.orientation.w = 1.0;

  marker.scale.x = 0.05;
  marker.color.a = 1.0;
  marker.color.r = 1.0;

  for (size_t i = 0; i < lines.size(); i++) {
    geometry_msgs::Point p1, p2;
    p1.x = lines[i].start_.x_;
    p1.y = lines[i].start_.y_;
    p2.x = lines[i].end_.x_;
    p2.y = lines[i].end_.y_;

    marker.points.push_back(p1);
    marker.points.push_back(p2);
  }

  wall_pub_.publish(marker);
}

void DataVisualizer::showWayPoints(const Pose2dVec& poses) {
  geometry_msgs::PoseArray pose_array;
  pose_array.header.frame_id = "map";
  pose_array.header.stamp = ros::Time::now();

  for (size_t i = 0; i < poses.size(); i++) {
    geometry_msgs::Pose pose;
    pose.position.x = poses[i].x_;
    pose.position.y = poses[i].y_;
    pose.position.z = 0.0;

    pose.orientation = tf::createQuaternionMsgFromYaw(poses[i].th_);
    pose_array.poses.push_back(pose);
  }

  way_points_pub_.publish(pose_array);
}

void DataVisualizer::showScanCloud(const Pose2dVec& points) {
  visualization_msgs::Marker marker;
  marker.header.frame_id = "map";
  marker.header.stamp = ros::Time::now();

  marker.ns = "cloud";
  marker.id = 0;
  marker.lifetime = ros::Duration();

  marker.type = visualization_msgs::Marker::POINTS;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.orientation.w = 1.0;

  marker.scale.x = marker.scale.y = 0.05;
  marker.color.a = 1.0;
  marker.color.b = 1.0;

  for (size_t i = 0; i < points.size(); i++) {
    geometry_msgs::Point p;
    p.x = points[i].x_;
    p.y = points[i].y_;

    marker.points.push_back(p);
  }

  scan_cloud_pub_.publish(marker);
}

}  // namespace mobile_base
