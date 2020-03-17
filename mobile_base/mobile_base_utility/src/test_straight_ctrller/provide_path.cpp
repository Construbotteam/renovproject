#include "nav_msgs/Path.h"
#include "ros/ros.h"
#include "tf/tf.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

double findDis(const double& x1, const double& y1, const double& x2,
               const double& y2);
bool getPose(geometry_msgs::PoseStamped* pose);
bool getPath(const double& goal_x, const double& goal_y, nav_msgs::Path* path);

int main(int argc, char** argv) {
  ros::init(argc, argv, "provide_path");
  ros::NodeHandle nh;

  tf2_ros::Buffer bf(ros::Duration(10.0));
  tf2_ros::TransformListener tfl(bf);

  ros::Publisher path_pub_ = nh.advertise<nav_msgs::Path>("path", 10);

  double goal_x, goal_y;
  ros::Rate r(10);
  while (ros::ok()) {
    bool if_pub;
    std::cout << "intput pub, '0' for stop and quit : ";
    std::cin >> if_pub;
    if (!if_pub) {
      exit(0);
    }
    std::cout << "input goal x : ";
    std::cin >> goal_x;
    std::cout << "input goal y : ";
    std::cin >> goal_y;

    nav_msgs::Path path;

    if (!getPath(goal_x, goal_y, &path)) {
      ROS_WARN("no path output");
      continue;
    } else {
      path_pub_.publish(path);
    }
    ros::spinOnce();
    r.sleep();
  }
}

bool getPath(const double& goal_x, const double& goal_y, nav_msgs::Path* path) {
  geometry_msgs::PoseStamped pose, goal;
  if (!getPose(&pose)) {
    ROS_WARN("get pose failue");
    return false;
  }

  double x1, x2, y1, y2;
  x1 = pose.pose.position.x;
  y1 = pose.pose.position.y;
  x2 = goal_x;
  y2 = goal_y;
  if (findDis(x1, y1, x2, y2) < 0.05) {
    ROS_WARN("too close ");
    return false;
  }

  goal = pose;
  goal.pose.position.x = x2;
  goal.pose.position.y = y2;
  goal.pose.orientation = tf::createQuaternionMsgFromYaw(M_PI / 2);

  path->header.frame_id = "map";
  path->header.stamp = ros::Time::now();

  path->poses.clear();
  path->poses.push_back(pose);

  double angle = atan2(y2 - y1, x2 - x1);
  double dis_increment = 0.05;
  int max_point_num = sqrt(pow(x1 - x2, 2) + pow(y1 - y2, 2)) / dis_increment;
  while (true) {
    double x_tmp =
        path->poses.back().pose.position.x + cos(angle) * dis_increment;
    double y_tmp =
        path->poses.back().pose.position.y + sin(angle) * dis_increment;

    geometry_msgs::PoseStamped path_point;
    path_point.header = pose.header;
    path_point.pose.position.x = x_tmp;
    path_point.pose.position.y = y_tmp;

    path->poses.push_back(path_point);

    double rest_dis = sqrt(pow(x2 - x_tmp, 2) + pow(y2 - y_tmp, 2));
    if (rest_dis < dis_increment) {
      path->poses.push_back(goal);
      break;
    }
    if (path->poses.size() > max_point_num * 2) {
      ROS_WARN("path point num exceeds");
      return false;
    }
  }
  return true;
}

bool getPose(geometry_msgs::PoseStamped* cur_pose) {
  tf2_ros::Buffer bf(ros::Duration(10.0));
  tf2_ros::TransformListener tf_listener(bf);

  geometry_msgs::TransformStamped tfs;
  if (bf.canTransform("map", "base_link", ros::Time(0), ros::Duration(0.2))) {
    try {
      tfs = bf.lookupTransform("map", "base_link", ros::Time(0),
                               ros::Duration(0.2));
    } catch (tf2::TransformException& ex) {
      ROS_WARN("tf error in parallel turn controller -> %s", ex.what());
      ros::Duration(1.0).sleep();
      return false;
    }
  } else {
    ROS_WARN("parallel turn controller error -> no transform");
    ros::Duration(0.1).sleep();
    return false;
  }

  cur_pose->header.frame_id = tfs.header.frame_id;
  cur_pose->header.stamp = tfs.header.stamp;

  cur_pose->pose.position.x = tfs.transform.translation.x;
  cur_pose->pose.position.y = tfs.transform.translation.y;
  cur_pose->pose.position.z = tfs.transform.translation.z;

  cur_pose->pose.orientation = tfs.transform.rotation;

  return true;
}
double findDis(const double& x1, const double& y1, const double& x2,
               const double& y2) {
  double dif1, dif2, dis;
  dif1 = pow(x1 - x2, 2);
  dif2 = pow(y1 - y2, 2);
  dis = sqrt(dif1 + dif2);
  return dis;
}