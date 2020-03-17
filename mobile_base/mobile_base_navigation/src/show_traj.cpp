#include <string>
#include "nav_msgs/Path.h"
#include "ros/ros.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

class ShowTraj {
 public:
  ShowTraj(tf2_ros::Buffer& buffer, ros::NodeHandle& nh);
  ~ShowTraj() {}
  void TrajectoryPublish();
  double FindDistance(double x1, double y1, double x2, double y2);

 private:
  tf2_ros::Buffer& bf_;
  nav_msgs::Path car_traj_;
  ros::Publisher traj_pub_;
};

ShowTraj::ShowTraj(tf2_ros::Buffer& buffer, ros::NodeHandle& nh) : bf_(buffer) {
  car_traj_.header.frame_id = "map";
  car_traj_.header.stamp = ros::Time::now();
  car_traj_.poses.clear();

  traj_pub_ = nh.advertise<nav_msgs::Path>("base_trajectory", 10);
}

void ShowTraj::TrajectoryPublish() {
  geometry_msgs::TransformStamped tfs;

  int count = 0;
  while (true) {
    if (bf_.canTransform("map", "base_link", ros::Time(0),
                         ros::Duration(0.2))) {
      try {
        tfs = bf_.lookupTransform("map", "base_link", ros::Time(0),
                                  ros::Duration(0.2));
      } catch (tf2::TransformException& ex) {
        ROS_WARN("tf error of map & base_link -> %s", ex.what());
        ros::Duration(1.0).sleep();
        return;
      }
      break;
    } else {
      count++;
      if (count > 10) {
        ROS_WARN("pub trajectory -> no transform, read failure");
        exit(0);
      }
      ROS_WARN("pub trajectory -> no transform, read again : %d", count);
      ros::Duration(0.2).sleep();
      continue;
    }
  }

  geometry_msgs::PoseStamped cur_pose;
  cur_pose.header.frame_id = "map";
  cur_pose.pose.position.x = tfs.transform.translation.x;
  cur_pose.pose.position.y = tfs.transform.translation.y;
  cur_pose.pose.position.z = tfs.transform.translation.z;

  cur_pose.pose.orientation = tfs.transform.rotation;

  if (car_traj_.poses.size() < 1) {
    car_traj_.poses.push_back(cur_pose);
    car_traj_.header.stamp = ros::Time::now();
    traj_pub_.publish(car_traj_);
    return;
  }

  double dis = FindDistance(cur_pose.pose.position.x, cur_pose.pose.position.y,
                            car_traj_.poses.back().pose.position.x,
                            car_traj_.poses.back().pose.position.y);
  if (dis > 0.05) {
    car_traj_.poses.push_back(cur_pose);
  }
  traj_pub_.publish(car_traj_);
}

double ShowTraj::FindDistance(double x1, double y1, double x2, double y2) {
  double dis;
  dis = sqrt(pow(x1 - x2, 2) + pow(y1 - y2, 2));
  return dis;
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "show_trajectory");
  ros::NodeHandle nh;

  tf2_ros::Buffer buffer(ros::Duration(10));
  tf2_ros::TransformListener tfl(buffer);

  ShowTraj show_traj(buffer, nh);

  ros::Rate r(10);
  while (ros::ok()) {
    show_traj.TrajectoryPublish();
    r.sleep();
    ros::spinOnce();
  }
  return 0;
}