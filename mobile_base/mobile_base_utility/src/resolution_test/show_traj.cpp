#include <fstream>
#include <string>
#include "nav_msgs/Path.h"
#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

class ShowTraj {
 public:
  ShowTraj(tf2_ros::Buffer& buffer, ros::NodeHandle& nh);
  ~ShowTraj() {}
  void TrajectoryPublish(const ros::TimerEvent&);
  double FindDistance(double x1, double y1, double x2, double y2);
  void WriteFileCallback(const std_msgs::Bool& flag);

 private:
  tf2_ros::Buffer& bf_;
  nav_msgs::Path car_traj_;
  ros::Publisher traj_pub_;
  ros::Subscriber write_file_sub_;
  ros::Timer traj_timer_;
};

ShowTraj::ShowTraj(tf2_ros::Buffer& buffer, ros::NodeHandle& nh) : bf_(buffer) {
  car_traj_.header.frame_id = "map";
  car_traj_.header.stamp = ros::Time::now();
  car_traj_.poses.clear();

  traj_pub_ = nh.advertise<nav_msgs::Path>("base_trajectory", 10);
  traj_timer_ = nh.createTimer(ros::Duration(1.0 / 10),
                               &ShowTraj::TrajectoryPublish, this);
  write_file_sub_ =
      nh.subscribe("write_file", 10, &ShowTraj::WriteFileCallback, this);
}

void ShowTraj::TrajectoryPublish(const ros::TimerEvent&) {
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

  double trigger_dis = 0.05;
  if (ros::param::has("trigger_dis")) {
    ros::param::get("trigger_dis", trigger_dis);
  } else {
    ros::param::set("trigger_dis", 0.05);
  }

  if (dis > trigger_dis) {
    car_traj_.poses.push_back(cur_pose);
  }
  traj_pub_.publish(car_traj_);
}

double ShowTraj::FindDistance(double x1, double y1, double x2, double y2) {
  double dis;
  dis = sqrt(pow(x1 - x2, 2) + pow(y1 - y2, 2));
  return dis;
}

void ShowTraj::WriteFileCallback(const std_msgs::Bool& flag) {
  if (flag.data) {
    std::string file_addr = "/home/glh/Desktop/path.txt";
    std::ofstream out(file_addr);
    if (out.is_open()) {
      for (size_t i = 0; i < car_traj_.poses.size(); i++) {
        out << car_traj_.poses[i].pose.position.x << "  "
            << car_traj_.poses[i].pose.position.y << std::endl;
      }
    } else {
      std::cout << "open file : " << file_addr << " failure" << std::endl;
      return;
    }
    out.close();
    std::cout << "data written to : " << file_addr << " success" << std::endl;
    exit(0);
  }
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "show_trajectory");
  ros::NodeHandle nh;

  tf2_ros::Buffer buffer(ros::Duration(10));
  tf2_ros::TransformListener tfl(buffer);

  ShowTraj show_traj(buffer, nh);

  ros::spin();
  return 0;
}