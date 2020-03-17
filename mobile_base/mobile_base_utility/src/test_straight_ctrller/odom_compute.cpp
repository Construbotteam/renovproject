#include <fstream>
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "tf/tf.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/transform_broadcaster.h"

struct Pose2d {
  Pose2d() : x(0), y(0), yaw(0) {}
  double x;
  double y;
  double yaw;
};

class OdomCompute {
 public:
  OdomCompute(ros::NodeHandle& nh, const double& freq);
  virtual ~OdomCompute() {}
  void getJointStateCallback(const sensor_msgs::JointState& js);
  void publishOdom(const ros::TimerEvent&);

 private:
  double v_;
  double direc_;
  double direc_vec_[8];
  double pre_stamp_;

  Pose2d pose_;

  tf2_ros::TransformBroadcaster tfb_;
  ros::Subscriber js_sub_;
  ros::Timer odom_timer_;
};

OdomCompute::OdomCompute(ros::NodeHandle& nh, const double& freq) {
  v_ = 0;
  direc_ = 0;
  pre_stamp_ = ros::Time::now().toSec();
  js_sub_ = nh.subscribe("joint_signal", 10,
                         &OdomCompute::getJointStateCallback, this);
  odom_timer_ = nh.createTimer(ros::Duration(1.0 / freq),
                               &OdomCompute::publishOdom, this);
}

void OdomCompute::getJointStateCallback(const sensor_msgs::JointState& js) {
  v_ = js.velocity[0];
  direc_ = js.position.back();
  for (size_t i = 0; i < js.position.size(); i++) {
    direc_vec_[i] = js.position[i];
  }
}

void OdomCompute::publishOdom(const ros::TimerEvent&) {
  double cur_stamp = ros::Time::now().toSec();
  double delta_t = cur_stamp - pre_stamp_;
  pre_stamp_ = cur_stamp;

  bool turning_motion = false;
  if (std::signbit(direc_vec_[4]) != std::signbit(direc_vec_[5]) &&
      std::signbit(direc_vec_[6]) != std::signbit(direc_vec_[7])) {
    turning_motion = true;
  }
  if (!turning_motion) {
  pose_.x += v_ * cos(direc_ + pose_.yaw) * delta_t;
  pose_.y += v_ * sin(direc_ + pose_.yaw) * delta_t;
  } else {
    double radius = sqrt(0.5 * 0.5 / 4 + 0.395 * 0.395 / 4);
    double angular_v = -v_ / radius;
    pose_.yaw += angular_v * delta_t;
  }

  geometry_msgs::TransformStamped tfs;
  tfs.child_frame_id = "base_link";
  tfs.header.frame_id = "map";
  tfs.transform.translation.x = pose_.x;
  tfs.transform.translation.y = pose_.y;
  tfs.transform.translation.z = 0.0;

  tf::Quaternion q = tf::createQuaternionFromYaw(pose_.yaw);
  tfs.transform.rotation.w = q.w();
  tfs.transform.rotation.x = q.x();
  tfs.transform.rotation.y = q.y();
  tfs.transform.rotation.z = q.z();

  tfb_.sendTransform(tfs);
}

int main(int argc, char** argv) {
  ros::init(argc, argv, "odom_compute");
  ros::NodeHandle nh;

  double freq;
  std::cout << "please enter the frequency : ";
  std::cin >> freq;

  OdomCompute oc(nh, freq);
  ros::spin();

  return 0;
}
