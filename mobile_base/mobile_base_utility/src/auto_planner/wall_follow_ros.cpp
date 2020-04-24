#include "mobile_base_utility/wall_follow_ros.h"

#include "mobile_base_utility/lsd.h"

namespace mobile_base {

WallFollowROS::WallFollowROS(ros::NodeHandle& nh, ros::NodeHandle& nh_private)
    : get_map_(false), wall_finish_(true) {
  initParam(nh_private);

  map_sub_ = nh.subscribe(map_topic_, 10, &WallFollowROS::getMapCallback, this);

  int center_num = 1;
  // remember to delete this pointer !!!!!!
  Pose2d* centers = new Pose2d[center_num];
  if (!room_extractor_.setRooms(center_num, centers)) {
    ROS_WARN("Set rooms to extractor failure");
  }

  int center_index = 0;
  while (ros::ok()) {
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
      }  // end of while-loop to get the arrival signal
      wall_finish_ = false;
    }  // end of sending wall-center as goal point and the base has arrived

    while (true) {
      /* get multi-frame of scan */
    }

    VirtualPic virtual_pic = room_extractor_.getVirtualPic();
    int line_num;
    double* lines = lsd(&line_num, virtual_pic.data_, virtual_pic.size_x_,
                        virtual_pic.size_y_);

    LineParamVec walls = room_extractor_.computeWalls(line_num, lines);
  }
}

WallFollowROS::~WallFollowROS() {}

void WallFollowROS::initParam(ros::NodeHandle& nh_private) {
  nh_private.param("map_topic", map_topic_, std::string("map"));
  nh_private.param("goal_frame", goal_frame_, std::string("map"));
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
  ps.header.frame_id = goal_frame_;
  ps.header.stamp = ros::Time::now();

  ps.pose.position.x = x;
  ps.pose.position.y = y;
  ps.pose.orientation = tf::createQuaternionMsgFromYaw(th);

  return ps;
}

}  //  namespace mobile_base