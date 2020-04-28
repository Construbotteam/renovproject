#include "mobile_base_utility/wall_follow_ros.h"

#include "mobile_base_utility/lsd.h"

namespace mobile_base {

WallFollowROS::WallFollowROS(ros::NodeHandle& nh, ros::NodeHandle& nh_private,
                             tf2_ros::Buffer& bf)
    : get_map_(false), wall_finish_(true), bf_(bf) {
  initParam(nh_private);

  map_sub_ = nh.subscribe(map_topic_, 10, &WallFollowROS::getMapCallback, this);
  goal_pub_ = nh.advertise<geometry_msgs::PoseStamped>(goal_topic_, 10);

  int center_num = 1;
  Pose2d centers[center_num];
  if (!room_extractor_.setRooms(center_num, centers)) {
    ROS_WARN("Set rooms to extractor failure");
  }

  int center_index = 0;
  while (ros::ok()) {
    if (!get_map_) {
      continue;
    }
    // send centers of walls sequentially
    if (wall_finish_) {
      geometry_msgs::PoseStamped goal = getStampedPose2D(
          centers[center_index].x_, centers[center_index].y_, 0.0);
      goal_pub_.publish(goal);

      // one goal point at a time and wait till arrival
      ros::Duration(0.1).sleep();
      while (true) {
        bool if_arrive = false;
        ros::param::get(arr_namespace_ + "/" + arr_param_name, if_arrive);
        if (if_arrive) {
          break;
        }
        ros::Duration(0.5).sleep();
      }  // end of while-loop to get the arrival signal
      wall_finish_ = false;
    }  // end of sending wall-center as goal point and the base has arrived

    geometry_msgs::PoseStamped global_pose;
    if (!getGlobalPose(&global_pose)) {
      ROS_WARN("Get initial pose failure");
      continue;
    }
    double pre_yaw = tf2::getYaw(global_pose.pose.orientation);

    // a trigger should be set here
    while (true) {
      ros::param::set(rot_switch_param_, true);
      if (!getGlobalPose(&global_pose)) {
        ROS_WARN("Get pose failure while obtaining scan point cloud");
        continue;
      }

      std::vector<double> scan_points;
      scanTransform(scan_points);
      room_extractor_.updateVirtualPic(scan_points);

      double cur_yaw = tf2::getYaw(global_pose.pose.orientation);
      if (cur_yaw - pre_yaw > capture_interval_) {
        scanTransform(scan_points);
        room_extractor_.updateVirtualPic(scan_points);
        pre_yaw = cur_yaw;
      }

      bool rot_switch;
      ros::param::get(rot_switch_param_, rot_switch);
      if (!rot_switch) {
        break;
      }
    }

    VirtualPic virtual_pic = room_extractor_.getVirtualPic();
    int line_num;
    double* lines = lsd(&line_num, virtual_pic.data_, virtual_pic.size_x_,
                        virtual_pic.size_y_);

    if (!getGlobalPose(&global_pose)) {
      ROS_WARN("Get global pose of robot failure!!!");
      continue;
    }

    Pose2d pose2d(global_pose.pose.position.x, global_pose.pose.position.y,
                  tf::getYaw(global_pose.pose.orientation));
    LineParamVec sequentail_walls =
        room_extractor_.computeWalls(line_num, lines, pose2d);

    ros::spinOnce();
  }
}

WallFollowROS::~WallFollowROS() {}

void WallFollowROS::initParam(ros::NodeHandle& nh_private) {
  nh_private.param("map_topic", map_topic_, std::string("map"));
  nh_private.param("map_frame_id", map_frame_id_, std::string("map"));
  nh_private.param("base_frame_id", base_frame_id_, std::string("base_link"));
  nh_private.param("scan_frame_id", scan_frame_id_, std::string("laser_link"));
  nh_private.param("arr_param_name", arr_param_name, std::string(""));
  nh_private.param("arr_namespace", arr_namespace_, std::string(""));

  nh_private.param("capture_velo", capture_velo_, 0.1);
  nh_private.param("capture_interval", capture_interval_, 0.1);
  capture_duration_ = 2 * M_PI / capture_velo_;
}

void WallFollowROS::getMapCallback(const nav_msgs::OccupancyGrid& map_msg) {
  if (!get_map_) {
    int size_x = map_msg.info.width;
    int size_y = map_msg.info.height;
    double ox = map_msg.info.origin.position.x;
    double oy = map_msg.info.origin.position.y;
    double reso = map_msg.info.resolution;

    room_extractor_.initVirtualPic(size_x, size_y, ox, oy, reso);
    get_map_ = true;
  }
}

void WallFollowROS::getScanCallback(const sensor_msgs::LaserScan& scan_msg) {
  scan_data_ = scan_msg;
}

geometry_msgs::PoseStamped WallFollowROS::getStampedPose2D(const double& x,
                                                           const double& y,
                                                           const double& th) {
  geometry_msgs::PoseStamped ps;
  ps.header.frame_id = map_frame_id_;
  ps.header.stamp = ros::Time::now();

  ps.pose.position.x = x;
  ps.pose.position.y = y;
  ps.pose.orientation = tf::createQuaternionMsgFromYaw(th);

  return ps;
}

bool WallFollowROS::getGlobalPose(geometry_msgs::PoseStamped* global_pose) {
  tf2::toMsg(tf2::Transform::getIdentity(), global_pose->pose);
  geometry_msgs::PoseStamped robot_pose;
  tf2::toMsg(tf2::Transform::getIdentity(), robot_pose.pose);
  robot_pose.header.frame_id = base_frame_id_;
  robot_pose.header.stamp = ros::Time();

  // save time for checking tf delay later
  ros::Time cur_time = ros::Time::now();

  // get the global pose of the robot
  try {
    bf_.transform(robot_pose, global_pose, map_frame_id_);
  } catch (const tf2::LookupException& ex) {
    ROS_ERROR("No Transform available Error looking up robot pose: %s\n",
              ex.what());
    return false;
  }

  // check global_pose timeout
  if (cur_time.toSec() - global_pose->header.stamp.toSec() > 0.1) {
    ROS_WARN(
        "Costmap2DROS transform timeout. Current time: %.4f, global_pose "
        "stamp: %.4f, tolerance: %.4f",
        cur_time.toSec(), global_pose->header.stamp.toSec(), 0.1);
    return false;
  }

  return true;
}

bool WallFollowROS::scanTransform(std::vector<double>& scan_points) {
  geometry_msgs::PoseArray raw_points, transformed_points;
  raw_points.header.frame_id = scan_frame_id_;
  raw_points.header.stamp = scan_data_.header.stamp;
  raw_points.poses.clear();

  double scan_angle = scan_data_.angle_min;
  int index = 0;
  while (scan_angle <= scan_data_.angle_max) {
    geometry_msgs::Pose pose;
    tf2::toMsg(tf2::Transform::getIdentity(), pose);
    transformed_points.poses.push_back(pose);

    pose.position.x = scan_data_.ranges[index] * cos(scan_angle);
    pose.position.y = scan_data_.ranges[index] * sin(scan_angle);
    raw_points.poses.push_back(pose);

    scan_angle += scan_data_.angle_increment;
    index++;
  }

  try {
    bf_.transform(raw_points, transformed_points, map_frame_id_);
  } catch (const tf2::LookupException& ex) {
    ROS_ERROR("No Transform available between laser and map: %s\n", ex.what());
    return false;
  }

  scan_points.clear();
  for (size_t i = 0; i < transformed_points.poses.size(); i++) {
    scan_points.push_back(transformed_points.poses[i].position.x);
    scan_points.push_back(transformed_points.poses[i].position.y);
  }

  return true;
}

}  //  namespace mobile_base