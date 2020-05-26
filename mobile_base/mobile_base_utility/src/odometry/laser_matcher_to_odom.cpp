#include "geometry_msgs/Pose2D.h"
#include "nav_msgs/Odometry.h"
#include "ros/ros.h"
#include "tf/tf.h"
#include "tf2_ros/transform_broadcaster.h"

class MatcherOdom {
 public:
  MatcherOdom(ros::NodeHandle& nh);
  ~MatcherOdom() {}
  void getPose2DCallback(const geometry_msgs::Pose2D& pose);

 private:
  tf2_ros::TransformBroadcaster tfb_;
  ros::Publisher odom_pub_;
  ros::Subscriber pose2d_sub_;
};

MatcherOdom::MatcherOdom(ros::NodeHandle& nh) {
  odom_pub_ = nh.advertise<nav_msgs::Odometry>("odom", 10);
  pose2d_sub_ =
      nh.subscribe("pose2D", 10, &MatcherOdom::getPose2DCallback, this);
}

void MatcherOdom::getPose2DCallback(const geometry_msgs::Pose2D& pose) {
  nav_msgs::Odometry odom_data;
  odom_data.header.frame_id = "odom";
  odom_data.child_frame_id = "base_link";
  odom_data.header.stamp = ros::Time::now();
  odom_data.pose.pose.position.x = pose.x;
  odom_data.pose.pose.position.y = pose.y;
  odom_data.pose.pose.orientation = tf::createQuaternionMsgFromYaw(pose.theta);

  geometry_msgs::TransformStamped tfs;
  tfs.child_frame_id = odom_data.child_frame_id;
  tfs.header = odom_data.header;
  tfs.transform.translation.x = pose.x;
  tfs.transform.translation.y = pose.y;
  tfs.transform.rotation = odom_data.pose.pose.orientation;

  odom_pub_.publish(odom_data);
  tfb_.sendTransform(tfs);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "laser_matcher_to_odom");
  ros::NodeHandle nh;

  MatcherOdom matcher_odom(nh);
  ros::spin();
  return 0;
}