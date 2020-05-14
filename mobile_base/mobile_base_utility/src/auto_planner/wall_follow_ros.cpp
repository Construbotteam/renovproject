#include "mobile_base_utility/wall_follow_ros.h"

#include "mobile_base_utility/lsd.h"

namespace mobile_base {

void test(const int& a, const int& b) {}

WallFollowROS::WallFollowROS(ros::NodeHandle& nh, ros::NodeHandle& nh_private,
                             tf2_ros::Buffer& bf)
    : get_map_(false), wall_finish_(true), use_lsd_(false), bf_(bf) {
  initParam(nh_private);

  map_sub_ = nh.subscribe(map_topic_, 10, &WallFollowROS::getMapCallback, this);
  scan_sub_ =
      nh.subscribe(scan_topic_, 10, &WallFollowROS::getScanCallback, this);
  goal_pub_ = nh.advertise<geometry_msgs::PoseStamped>(goal_topic_, 10);
  visualizer_.getNodeHandle(nh);

  // int center_num = 1;
  // Pose2d centers[center_num];
  center_num_ = 1;
  centers_ = new Pose2d[center_num_];
  centers_[0].x_ = 0.0;
  centers_[0].y_ = 0.0;
  centers_[0].th_ = 0.0;
  if (!room_extractor_.setRooms(center_num_, centers_)) {
    ROS_WARN("Set rooms to extractor failure");
  }

  plan_thread_ = new std::thread(&WallFollowROS::planLoop, this);
}

void WallFollowROS::planLoop() {
  // remember to increase by 1 after each circle
  int center_index = 0;
  while (ros::ok()) {
    if (!get_map_) {
      ROS_WARN("Please a offer a map by topic first");
      ros::Duration(1.0).sleep();
      continue;
    }

    if (wall_finish_) {
      // send centers of walls sequentially
      geometry_msgs::PoseStamped goal = getStampedPose2D(
          centers_[center_index].x_, centers_[center_index].y_, 0.0);
      goal_pub_.publish(goal);

      // one goal point at a time and wait till arrival
      ros::Duration(0.1).sleep();
      while (true) {
        bool if_arrive = false;
        ros::param::get(arr_param_name_, if_arrive);
        if (if_arrive) {
          break;
        }
        ros::Duration(0.5).sleep();
      }  // end of while-loop to get the arrival signal
      wall_finish_ = false;
      ROS_INFO("I arrive the center position");

      // combine different frames of scans to get a virtual 2D pixel picture
      geometry_msgs::PoseStamped global_pose;
      if (!getGlobalPose(global_pose)) {
        ROS_WARN("Get initial pose failure");
        continue;
      }
      double pre_yaw = tf2::getYaw(global_pose.pose.orientation);

      // reset and update for the first time
      std::vector<double> scan_points;
      scanTransform(scan_points);
      if (use_lsd_) {
        room_extractor_.resetVirtualPic();
        room_extractor_.updateVirtualPic(scan_points);
      } else {
        room_extractor_.resetScanCloud();
        room_extractor_.updateScanCloud(scan_points);
      }
      ROS_INFO("I update the first frame of point cloud");

      ros::param::set(rot_switch_param_, true);
      while (true) {
        if (!getGlobalPose(global_pose)) {
          ROS_WARN("Get pose failure while obtaining scan point cloud");
          continue;
        }

        double cur_yaw = tf2::getYaw(global_pose.pose.orientation);
        double angle_diff =
            fabs(angles::shortest_angular_distance(pre_yaw, cur_yaw));
        if (angle_diff > capture_interval_) {
          scanTransform(scan_points);
          if (use_lsd_) {
            room_extractor_.updateVirtualPic(scan_points);
          } else {
            room_extractor_.updateScanCloud(scan_points);
          }
          pre_yaw = cur_yaw;
        }

        bool rot_switch = true;
        ros::param::get(rot_switch_param_, rot_switch);
        if (!rot_switch) {
          break;
        }
      }
      ROS_INFO("I get all the frames of point cloud");

      if (!getGlobalPose(global_pose)) {
        ROS_WARN("Get global pose of robot failure!!!");
        continue;
      }
      Pose2d pose2d(global_pose.pose.position.x, global_pose.pose.position.y,
                    tf2::getYaw(global_pose.pose.orientation));

      LineParamVec sequential_walls;
      if (use_lsd_) {
        // apply lsd method to extract line features in virtual picture
        VirtualPic virtual_pic = room_extractor_.getVirtualPic();
        visualizer_.showVirtualPic(virtual_pic);

        int line_num;
        double* lines = lsd(&line_num, virtual_pic.data_, virtual_pic.size_x_,
                            virtual_pic.size_y_);

        sequential_walls =
            room_extractor_.computeWalls(line_num, lines, pose2d);
      } else {
        sequential_walls = room_extractor_.extract();
        visualizer_.showScanCloud(room_extractor_.getScanCloud());
      }
      visualizer_.showWalls(sequential_walls);
      ROS_INFO("I finish computing %d walls", sequential_walls.size());

      // get way points
      Pose2dVec way_points = getWayPoints(sequential_walls, pose2d);
      visualizer_.showWayPoints(way_points);

    }  // end of extracting walls into way points

    ros::spinOnce();
  }
}

WallFollowROS::~WallFollowROS() {
  if (centers_) {
    delete[] centers_;
  }
  if (plan_thread_) {
    delete plan_thread_;
  }
}

void WallFollowROS::initParam(ros::NodeHandle& nh_private) {
  nh_private.param("map_topic", map_topic_, std::string("map"));
  nh_private.param("scan_topic", scan_topic_, std::string("scan"));
  nh_private.param("goal_topic", goal_topic_, std::string("GoalPoint"));
  nh_private.param("map_frame_id", map_frame_id_, std::string("map"));
  nh_private.param("base_frame_id", base_frame_id_, std::string("base_link"));
  nh_private.param("scan_frame_id", scan_frame_id_, std::string("laser_link"));
  nh_private.param("arr_param_name", arr_param_name_, std::string(""));
  nh_private.param("arr_namespace", arr_namespace_, std::string(""));
  nh_private.param("rot_switch_param", rot_switch_param_, std::string(""));

  nh_private.param("capture_velo", capture_velo_, 0.1);
  nh_private.param("capture_interval", capture_interval_, 0.1);
  capture_duration_ = 2 * M_PI / capture_velo_;

  nh_private.param("circum_radius", circum_radius_, 0.5);
  nh_private.param("task_dis", task_dis_, 0.5);
  nh_private.param("task_interval", task_interval_, 0.5);
  nh_private.param("max_laser_range", max_laser_range_, 10.0);

  nh_private.param("use_lsd", use_lsd_, false);

  int step_size, countThreshold, min_fit_points_num;
  double disThreshold, min_wall_size;
  double min_angle_dis, min_neighbour_dis;
  nh_private.param("step_size", step_size, 5);
  nh_private.param("countThreshold", countThreshold, 2);
  nh_private.param("disThreshold", disThreshold, 0.05);
  nh_private.param("min_wall_size", min_wall_size, 0.6);
  nh_private.param("min_fit_points_num", min_fit_points_num, 3);
  nh_private.param("min_angle_dis", min_angle_dis, 0.1);
  nh_private.param("min_neighbour_dis", min_neighbour_dis, 0.05);
  room_extractor_.setExtractorParam(step_size, disThreshold, countThreshold,
                                    min_wall_size, min_fit_points_num,
                                    min_angle_dis, min_neighbour_dis);
}

void WallFollowROS::getMapCallback(const nav_msgs::OccupancyGrid& map_msg) {
  if (!get_map_) {
    int size_x = map_msg.info.width;
    int size_y = map_msg.info.height;
    double ox = map_msg.info.origin.position.x;
    double oy = map_msg.info.origin.position.y;
    double reso = map_msg.info.resolution;

    ROS_INFO(
        "\n Map info \n size_x : %d \n size_y : %d \n origin_x : %.3f \n "
        "origin_y : %.3f \n reso : %.3f \n",
        size_x, size_y, ox, oy, reso);

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

bool WallFollowROS::getGlobalPose(geometry_msgs::PoseStamped& global_pose) {
  tf2::toMsg(tf2::Transform::getIdentity(), global_pose.pose);
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
  if (cur_time.toSec() - global_pose.header.stamp.toSec() > 0.1) {
    ROS_WARN(
        "Costmap2DROS transform timeout. Current time: %.4f, global_pose "
        "stamp: %.4f, tolerance: %.4f",
        cur_time.toSec(), global_pose.header.stamp.toSec(), 0.1);
    return false;
  }

  return true;
}

bool WallFollowROS::scanTransform(std::vector<double>& scan_points) {
  std::vector<geometry_msgs::PoseStamped> raw_points, transformed_points;
  raw_points.clear();

  double scan_angle = scan_data_.angle_min;
  int index = 0;
  while (scan_angle <= scan_data_.angle_max) {
    double range = scan_data_.ranges[index];
    if (fabs(range) > max_laser_range_ || std::isnan(range)) {
      index++;
      continue;
    }

    geometry_msgs::PoseStamped pose;
    tf2::toMsg(tf2::Transform::getIdentity(), pose.pose);

    pose.header.frame_id = scan_frame_id_;
    pose.header.stamp = ros::Time();
    pose.pose.position.x = scan_data_.ranges[index] * cos(scan_angle);
    pose.pose.position.y = scan_data_.ranges[index] * sin(scan_angle);
    raw_points.push_back(pose);

    scan_angle += scan_data_.angle_increment;
    index++;
  }

  geometry_msgs::TransformStamped tfs;
  try {
    tfs = bf_.lookupTransform(map_frame_id_, scan_frame_id_, ros::Time(0),
                              ros::Duration(0.1));
  } catch (const tf2::LookupException& ex) {
    ROS_ERROR("No Transform available between laser and map: %s\n", ex.what());
    ros::Duration(0.1).sleep();
    return false;
  }
  transformPoints(raw_points, transformed_points, tfs.transform);

  scan_points.clear();
  for (size_t i = 0; i < transformed_points.size(); i++) {
    scan_points.push_back(transformed_points[i].pose.position.x);
    scan_points.push_back(transformed_points[i].pose.position.y);
  }

  return true;
}

Pose2dVec WallFollowROS::getWayPoints(const LineParamVec& sorted_lines,
                                      const Pose2d& pose) {
  Pose2dVec way_points;
  way_points.clear();

  Eigen::Vector2d pose_vec(pose.x_, pose.y_);
  for (size_t i = 0; i < sorted_lines.size(); i++) {
    Eigen::Vector2d start_vec(sorted_lines[i].start_.x_,
                              sorted_lines[i].start_.y_);
    Eigen::Vector2d end_vec(sorted_lines[i].end_.x_, sorted_lines[i].end_.y_);
    Eigen::Vector2d direc_vec = end_vec - start_vec;
    double wall_length = direc_vec.norm();

    double coef =
        (pose_vec - start_vec).dot(direc_vec) / pow(direc_vec.norm(), 2);

    Eigen::Vector2d ortho_point_vec = start_vec + coef * direc_vec;
    Eigen::Vector2d ortho_vec = pose_vec - ortho_point_vec;

    direc_vec.normalize();
    ortho_vec.normalize();
    ortho_vec = ortho_vec * task_dis_;

    Eigen::Vector2d new_start = start_vec + ortho_vec;
    Eigen::Vector2d new_end = end_vec + ortho_vec;

    bool get_first_task_pose = false;
    double yaw = atan2(ortho_vec(0), ortho_vec(1));
    Eigen::Vector2d task_pose;
    while (true) {
      if (!get_first_task_pose) {
        task_pose = new_start + task_interval_ * direc_vec;
        way_points.push_back(Pose2d(task_pose(0), task_pose(1), yaw));

        get_first_task_pose = true;
        continue;
      }
      task_pose += task_interval_ * direc_vec;

      if ((task_pose - new_start).norm() > wall_length) {
        task_pose = new_end - circum_radius_ * direc_vec;
        way_points.push_back(Pose2d(task_pose(0), task_pose(1), yaw));
        break;
      } else {
        way_points.push_back(Pose2d(task_pose(0), task_pose(1), yaw));
      }
    }
  }

  return way_points;
}

void WallFollowROS::transformPoints(const PoseStampedVec& in,
                                    PoseStampedVec& out,
                                    const geometry_msgs::Transform& tfm) {
  Eigen::Quaterniond quat(tfm.rotation.w, tfm.rotation.x, tfm.rotation.y,
                          tfm.rotation.z);
  Eigen::Matrix3d rot_m = quat.toRotationMatrix();

  out.clear();
  for (size_t i = 0; i < in.size(); i++) {
    geometry_msgs::PoseStamped pose;
    pose.pose.position.x = in[i].pose.position.x * rot_m(0, 0) +
                           in[i].pose.position.y * rot_m(0, 1) +
                           tfm.translation.x;
    pose.pose.position.y = in[i].pose.position.x * rot_m(1, 0) +
                           in[i].pose.position.y * rot_m(1, 1) +
                           tfm.translation.y;

    out.push_back(pose);
  }
}

}  //  namespace mobile_base
