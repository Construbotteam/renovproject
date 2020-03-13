#include <yaml-cpp/yaml.h>
#include <string>
#include <vector>
#include "nav_msgs/Path.h"
#include "ros/ros.h"
#include "tf/tf.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

class StraightLinePath {
 public:
  StraightLinePath(ros::NodeHandle& nh, ros::NodeHandle& nh_private);
  virtual ~StraightLinePath() {}
  void ParamInit(ros::NodeHandle& nh_private);
  void ReadFile();
  bool GetPose(geometry_msgs::PoseStamped* pose);
  bool GeneratePath(const geometry_msgs::PoseStamped& start_pose,
                    const geometry_msgs::PoseStamped& goal_pose,
                    nav_msgs::Path* path);
  void PublishPath(const ros::TimerEvent&);

 private:
  std::string file_addr_;
  std::string map_frame_id_;
  std::string base_frame_id_;
  std::string path_topic_;
  // std::string manipulator_status_name_;

  std::vector<double> x_coordinate_list_;
  std::vector<double> y_coordinate_list_;
  std::vector<double> yaw_list_;
  std::vector<int> turning_point_num_list_;
  std::vector<bool> flag_list_;

  int points_num_;
  int path_num_;
  double dis_interval_;
  // bool if_manipulator_finish_;
  // size_t cur_index_;
  double plan_frequency_;
  int path_index_;
  int skip_count_;

  ros::Publisher path_pub_;
  ros::Timer path_pub_timer_;
};

StraightLinePath::StraightLinePath(ros::NodeHandle& nh,
                                   ros::NodeHandle& nh_private) {
  ParamInit(nh_private);
  ReadFile();
  // if_manipulator_finish_ = false;
  // cur_index_ = 0;
  path_index_ = -1;
  skip_count_ = 0;

  path_pub_ = nh.advertise<nav_msgs::Path>(path_topic_, 10);
  path_pub_timer_ = nh.createTimer(ros::Duration(1.0 / plan_frequency_),
                                   &StraightLinePath::PublishPath, this);
}

void StraightLinePath::ParamInit(ros::NodeHandle& nh_private) {
  nh_private.param("file_addr", file_addr_, std::string(""));
  nh_private.param("map_frame_id", map_frame_id_, std::string("map"));
  nh_private.param("base_frame_id", base_frame_id_, std::string("base_link"));
  nh_private.param("path_topic", path_topic_, std::string("straight_path"));
  // nh_private.param("manipulator_status_name", manipulator_status_name_,
  //                  std::string("if_manipulator_finish"));
  nh_private.param("plan_frequency", plan_frequency_, 10.0);
}

void StraightLinePath::ReadFile() {
  YAML::Node file = YAML::LoadFile(file_addr_);

  points_num_ = file["points_num"].as<int>();
  dis_interval_ = file["dis_interval"].as<double>();
  ROS_INFO("get goal points quantity : %d", points_num_);

  x_coordinate_list_.clear();
  y_coordinate_list_.clear();
  yaw_list_.clear();
  flag_list_.clear();

  path_num_ = 0;
  for (size_t i = 0; i < points_num_; i++) {
    double x_tmp = file["x_coordinate_list"][i].as<double>();
    double y_tmp = file["y_coordinate_list"][i].as<double>();
    double yaw_tmp = file["yaw_list"][i].as<double>();
    bool flag_tmp = file["flag_list"][i].as<bool>();

    x_coordinate_list_.push_back(x_tmp);
    y_coordinate_list_.push_back(y_tmp);
    yaw_list_.push_back(yaw_tmp);
    flag_list_.push_back(flag_tmp);

    // std::cout << "=========" << std::endl;
    // std::cout << "x is " << x_tmp << std::endl;
    // std::cout << "y is " << y_tmp << std::endl;
    // std::cout << "yaw tmp is " << yaw_tmp << std::endl;
    // std::cout << "=========" << std::endl;

    if (flag_tmp) {
      path_num_++;
      // int turning_point_num = 0;
      // for (size_t j = 0; j < i - 1; j++) {
      //   if (!flag_list_[j]) {
      //     turning_point_num++;
      //   }
      // }
      // turning_point_num_list_.push_back(turning_point_num);
    }
  }
  ROS_INFO("path num is : %d", path_num_);

  // for (size_t i = 0; i < points_num_; i++) {
  //   int turning_point_num = 0;
  //   for (size_t j = 0; j < i; j++) {
  //     if (!yaw_list_[j]) {
  //       turning_point_num++;
  //     }
  //   }
  //   if (i - 1 >= 0&&!flag_list_) {
  //   }
  //   turning_point_num_list_.push_back(turning_point_num);
  // }
}

bool StraightLinePath::GetPose(geometry_msgs::PoseStamped* pose) {
  tf2_ros::Buffer buffer(ros::Duration(10.0));
  tf2_ros::TransformListener tfl(buffer);

  geometry_msgs::TransformStamped tf_stamped;
  if (buffer.canTransform(map_frame_id_, base_frame_id_, ros::Time(0),
                          ros::Duration(0.2))) {
    try {
      tf_stamped = buffer.lookupTransform(map_frame_id_, base_frame_id_,
                                          ros::Time(0), ros::Duration(0.2));

    } catch (tf2::TransformException& ex) {
      ROS_WARN("error in straight line path publisher -> %s", ex.what());
      ros::Duration(1.0).sleep();
      return false;
    }
  } else {
    ROS_WARN("no transform -> read tf failure");
    return false;
  }

  pose->header.frame_id = map_frame_id_;
  pose->header.stamp = tf_stamped.header.stamp;

  pose->pose.position.x = tf_stamped.transform.translation.x;
  pose->pose.position.y = tf_stamped.transform.translation.y;
  pose->pose.position.z = tf_stamped.transform.translation.z;

  pose->pose.orientation.x = tf_stamped.transform.rotation.x;
  pose->pose.orientation.y = tf_stamped.transform.rotation.y;
  pose->pose.orientation.z = tf_stamped.transform.rotation.z;
  pose->pose.orientation.w = tf_stamped.transform.rotation.w;

  return true;
}

bool StraightLinePath::GeneratePath(
    const geometry_msgs::PoseStamped& start_pose,
    const geometry_msgs::PoseStamped& goal_pose, nav_msgs::Path* path) {
  double x_start, y_start, x_goal, y_goal;
  x_start = start_pose.pose.position.x;
  y_start = start_pose.pose.position.y;
  x_goal = goal_pose.pose.position.x;
  y_goal = goal_pose.pose.position.y;

  path->poses.push_back(start_pose);

  double segment_angle = atan2(y_goal - y_start, x_goal - x_start);
  double path_point_dis = 0;
  double max_dis = sqrt(pow(y_goal - y_start, 2) + pow(x_goal - x_start, 2));
  if (max_dis < dis_interval_) {
    ROS_WARN(
        "the current pose and goal pose are too close, distance is %.4fm which "
        "is closer than "
        ": %.4fm",
        max_dis, dis_interval_);
    ROS_WARN(
        "current pose : x = %.4f, y = %.4f; goal pose : x = %.4f, y = %.4f",
        x_start, y_start, x_goal, y_goal);
    return false;
  }

  tf::Quaternion path_quat = tf::createQuaternionFromYaw(segment_angle);

  path->header.frame_id = map_frame_id_;
  path->header.stamp = ros::Time::now();

  while (path_point_dis + dis_interval_ < max_dis) {
    path_point_dis += dis_interval_;
    geometry_msgs::PoseStamped pose_tmp;
    pose_tmp.header.frame_id = map_frame_id_;
    pose_tmp.pose.position.x =
        path->poses.back().pose.position.x + dis_interval_ * cos(segment_angle);
    pose_tmp.pose.position.y =
        path->poses.back().pose.position.y + dis_interval_ * sin(segment_angle);
    // std::cout << "pose tmp is : " << pose_tmp.pose.position.x << "  "
    //           << pose_tmp.pose.position.y << std::endl;
    pose_tmp.pose.orientation.x = path_quat.x();
    pose_tmp.pose.orientation.y = path_quat.y();
    pose_tmp.pose.orientation.z = path_quat.z();
    pose_tmp.pose.orientation.w = path_quat.w();

    path->poses.push_back(pose_tmp);
    // std::cout << "++++++++++++" << std::endl;
  }

  path->poses.push_back(goal_pose);
  return true;
}

void StraightLinePath::PublishPath(const ros::TimerEvent&) {
  // if (!ros::param::get(manipulator_status_name_, if_manipulator_finish_)) {
  //   if_manipulator_finish_ = false;
  // }

  if (!ros::param::get("path_index", path_index_)) {
    ROS_WARN("cannot receive path index!!");
  }
  if (path_index_ > path_num_) {
    ROS_WARN(
        "path index exceeds the limit, the index will set as the final segment "
        "of path");
    path_index_ = path_num_;
  }

  path_index_ = path_index_ - 1;
  if (path_index_ >= 0) {
    nav_msgs::Path path;
    geometry_msgs::PoseStamped cur_pose;
    if (!GetPose(&cur_pose)) {
      ROS_WARN("get robot pose failure");
      return;
    }
    if (path_index_ + skip_count_ > path_num_ - 1) {
      path.header.frame_id = map_frame_id_;
      path.header.stamp = ros::Time::now();
      path.poses.clear();
      return;
    }

    ROS_INFO("to goal point index: ");
    while (true) {
      geometry_msgs::PoseStamped goal_pose;
      goal_pose.header.frame_id = map_frame_id_;
      goal_pose.header.stamp = ros::Time::now();

      std::cout << "the index i use is : " << path_index_ + skip_count_
                << std::endl;

      goal_pose.pose.position.x = x_coordinate_list_[path_index_ + skip_count_];
      goal_pose.pose.position.y = y_coordinate_list_[path_index_ + skip_count_];

      tf::Quaternion goal_quat =
          tf::createQuaternionFromYaw(yaw_list_[path_index_ + skip_count_]);
      goal_pose.pose.orientation.x = goal_quat.x();
      goal_pose.pose.orientation.y = goal_quat.y();
      goal_pose.pose.orientation.z = goal_quat.z();
      goal_pose.pose.orientation.w = goal_quat.w();

      // ROS_INFO("cur pose : x: %.4f, y: %.4f; goal pose : x: %.4f, y: %.4f",
      //          cur_pose.pose.position.x, cur_pose.pose.position.y,
      //          goal_pose.pose.position.x, goal_pose.pose.position.y);
      if (!GeneratePath(cur_pose, goal_pose, &path)) {
        ROS_WARN("get path failure");
      } else {
        ROS_INFO("get path success");
      }

      // ROS_INFO("%d", (int)path_index_);
      ROS_INFO("set a goal");
      if (flag_list_[path_index_ + skip_count_]) {
        break;
      }
      skip_count_++;
      cur_pose = goal_pose;

      // if (path_index_ < path_num_ - 1) {
      //   cur_index_++;
      // } else {
      //   ROS_WARN("do not set the last goal point to false");
      //   return;
      // }

    }  // end of getting path loop

    int pre_index = path_index_;
    while (true) {
      path_pub_.publish(path);
      ros::Duration(0.2).sleep();
      ros::param::get("path_index", path_index_);
      // std::cout << "path index " << path_index_ << std::endl;
      // std::cout << "pre index " << pre_index << std::endl;

      if (path_index_ - 1 != pre_index) {
        // std::cout << "index modified" << std::endl;
        break;
      }
    }
    // ros::param::set(manipulator_status_name_, 0);
    // if (cur_index_ < points_num_ - 1) {
    //   cur_index_++;
    // } else {
    //   return;
    // }

  }  // end of path publish operation
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "straight_line_path");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  StraightLinePath straight_line_path(nh, nh_private);
  ros::spin();

  return 0;
}