#include <eigen3/Eigen/Core>

#include "nav_msgs/Odometry.h"
#include "pcl/common/io.h"
#include "pcl/console/time.h"
#include "pcl/registration/icp.h"
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
#include "tf/tf.h"

typedef pcl::PointXYZ Point3d;
typedef pcl::PointCloud<Point3d> PointCloud3d;

namespace mobile_base {

class IcpOdom {
 public:
  IcpOdom(ros::NodeHandle& nh, ros::NodeHandle& nh_private)
      : get_first_cloud_(false) {
    initParam(nh_private);
    pre_cloud_ = boost::make_shared<PointCloud3d>();
    odom_data_.header.frame_id = odom_frame_;
    odom_data_.child_frame_id = base_frame_;

    odom_trans_ = Eigen::Matrix4f::Identity(4, 4);

    odom_pub_ = nh.advertise<nav_msgs::Odometry>(odom_topic_, 10);
    scan_sub_ = nh.subscribe(scan_topic_, 10, &IcpOdom::getScanCallback, this);
  }
  ~IcpOdom() {}
  void initParam(ros::NodeHandle& nh_private);
  void getScanCallback(const sensor_msgs::LaserScan& scan);

 private:
  std::string odom_frame_, base_frame_;
  std::string odom_topic_, scan_topic_;
  bool get_first_cloud_;
  Eigen::Matrix4f odom_trans_;
  PointCloud3d::Ptr pre_cloud_;

  nav_msgs::Odometry odom_data_;
  ros::Publisher odom_pub_;
  ros::Subscriber scan_sub_;
};  // class IcpOdom

void IcpOdom::initParam(ros::NodeHandle& nh_private) {
  nh_private.param("odom_frame", odom_frame_, std::string("odom"));
  nh_private.param("base_frame", base_frame_, std::string("base_link"));
  nh_private.param("odom_topic", odom_topic_, std::string("icp_odom"));
  nh_private.param("scan_topic", scan_topic_, std::string("scan"));
}

void IcpOdom::getScanCallback(const sensor_msgs::LaserScan& scan) {
  PointCloud3d::Ptr cur_cloud(new PointCloud3d);

  int id = 0;
  for (double angle = scan.angle_min; angle <= scan.angle_max;
       angle += scan.angle_increment) {
    Point3d point;
    point.x = scan.ranges[id] * cos(angle);
    point.y = scan.ranges[id] * sin(angle);
    point.z = 0;

    cur_cloud->points.push_back(point);
    id++;
  }

  if (!get_first_cloud_) {
    *pre_cloud_ = *cur_cloud;
    get_first_cloud_ = true;
    return;
  }

  pcl::IterativeClosestPoint<Point3d, Point3d> icp;
  icp.setMaximumIterations(4);
  icp.setEuclideanFitnessEpsilon(1e-4);
  icp.setTransformationEpsilon(1e-10);

  icp.setInputSource(cur_cloud);
  icp.setInputTarget(pre_cloud_);

  PointCloud3d::Ptr final_cloud(new PointCloud3d);
  icp.align(*final_cloud);
  Eigen::Matrix4f mat = icp.getFinalTransformation();
  std::cout << "==================" << std::endl;
  std::cout << mat << std::endl;
  odom_trans_ = odom_trans_ * mat;

  Eigen::Matrix3f rot = odom_trans_.block(0, 0, 3, 3);
  Eigen::Vector3f euler_angles = rot.eulerAngles(0, 1, 2);
  odom_data_.pose.pose.position.x = odom_trans_(0, 3);
  odom_data_.pose.pose.position.y = odom_trans_(1, 3);

  odom_data_.pose.pose.orientation =
      tf::createQuaternionMsgFromYaw(euler_angles(2));
  odom_pub_.publish(odom_data_);

  *pre_cloud_ = *cur_cloud;
}

}  // namespace mobile_base

int main(int argc, char** argv) {
  ros::init(argc, argv, "icp_odom");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  mobile_base::IcpOdom icp_odom(nh, nh_private);

  ros::spin();

  return 0;
}