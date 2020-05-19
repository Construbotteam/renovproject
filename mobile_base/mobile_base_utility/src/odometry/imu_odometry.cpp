#include <iostream>

#include "nav_msgs/Odometry.h"
#include "ros/ros.h"
#include "sensor_msgs/Imu.h"
#include "tf/tf.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_broadcaster.h"

class ImuOdom {
 public:
  ImuOdom(ros::NodeHandle& nh, ros::NodeHandle& nh_private)
      : get_first_imu_msg_(false) {
    initParam(nh_private);
    odom_data_.child_frame_id = base_frame_;
    odom_data_.header.frame_id = odom_frame_;
    odom_data_.pose.pose.position.x = odom_data_.pose.pose.position.y = 0.0;
    odom_data_.pose.pose.orientation = tf::createQuaternionMsgFromYaw(0.0);

    imu_sub_ = nh.subscribe(imu_topic_, 10, &ImuOdom::imuCallback, this);
    if (pub_odom_) {
      odom_pub_ = nh.advertise<nav_msgs::Odometry>(odom_topic_, 10);
    }
  }
  virtual ~ImuOdom() {}
  void initParam(ros::NodeHandle& nh_private);
  void imuCallback(const sensor_msgs::Imu& imu_msg);

 private:
  std::string imu_topic_, odom_topic_;
  std::string odom_frame_, base_frame_;
  bool get_first_imu_msg_, pub_odom_;

  tf2_ros::TransformBroadcaster tfb_;
  ros::Publisher odom_pub_;
  ros::Subscriber imu_sub_;
  ros::Time stamp_;

  nav_msgs::Odometry odom_data_;
};  // class ImuOdom

void ImuOdom::initParam(ros::NodeHandle& nh_private) {
  nh_private.param("imu_topic", imu_topic_, std::string("imu"));
  nh_private.param("odom_topic", odom_topic_, std::string("odom"));
  nh_private.param("odom_frame", odom_frame_, std::string("odom"));
  nh_private.param("base_frame", base_frame_, std::string("base_link"));

  nh_private.param("pub_odom", pub_odom_, true);
}

void ImuOdom::imuCallback(const sensor_msgs::Imu& imu_msg) {
  if (!get_first_imu_msg_) {
    stamp_ = imu_msg.header.stamp;
    get_first_imu_msg_ = true;
    return;
  }

  double dt = (imu_msg.header.stamp - stamp_).toSec();
  stamp_ = imu_msg.header.stamp;
  odom_data_.header.stamp = imu_msg.header.stamp;

  odom_data_.twist.twist.linear.x += imu_msg.linear_acceleration.x * dt;
  odom_data_.twist.twist.linear.y += imu_msg.linear_acceleration.y * dt;

  odom_data_.pose.pose.position.x += odom_data_.twist.twist.linear.x * dt;
  odom_data_.pose.pose.position.y += odom_data_.twist.twist.linear.y * dt;
  odom_data_.pose.pose.orientation = imu_msg.orientation;

  if (pub_odom_) {
    odom_pub_.publish(odom_data_);
  }

  geometry_msgs::TransformStamped tfs;
  tfs.child_frame_id = base_frame_;
  tfs.header.frame_id = odom_frame_;
  tfs.header.stamp = imu_msg.header.stamp;
  tfs.transform.rotation = odom_data_.pose.pose.orientation;
  tfs.transform.translation.x = odom_data_.pose.pose.position.x;
  tfs.transform.translation.y = odom_data_.pose.pose.position.y;

  tfb_.sendTransform(tfs);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "imu_odom");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private;

  ImuOdom odom_publisher(nh, nh_private);

  ros::spin();
  return 0;
}
