#include "ros/ros.h"
#include "nav_msgs/Path.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"


int main(int argc, char** argv) {
  ros::init(argc, argv, "pure_rot_traj");
  ros::NodeHandle nh;

  ros::Publisher pub = nh.advertise<nav_msgs::Path>("rot_traj", 10);
  nav_msgs::Path traj;
  traj.header.frame_id = "map";

  std::string base_frame_id_ = "base_link";
  std::string map_frame_id_ = "map";

  tf2_ros::Buffer tf_buffer(ros::Duration(10));
  tf2_ros::TransformListener tl(tf_buffer);
  geometry_msgs::TransformStamped tfs;

  int tmp_count = 0;
  ros::Rate r(10);
  while (ros::ok()) {

    if (tf_buffer.canTransform(map_frame_id_, base_frame_id_, ros::Time(0),
                               ros::Duration(0.2))) {
      try {
        tfs = tf_buffer.lookupTransform(map_frame_id_, base_frame_id_,
                                        ros::Time(0), ros::Duration(0.2));
      } catch (tf2::TransformException& ex) {
        ROS_WARN("error in mobile planner -> %s", ex.what());
        ros::Duration(1.0).sleep();
        continue;
      }
      tmp_count = 0;
    } else {
      tmp_count++;
      ROS_WARN("mobile planner error -> no transform, read again : %d",
               tmp_count);
      if (tmp_count > 10) {
        ROS_WARN("mobile planner error -> no transform, read failure");
        exit(0);
      }
      ros::Duration(0.2).sleep();
      continue;
    }

    geometry_msgs::PoseStamped start;
    start.header.frame_id = "map";
    start.pose.position.x = tfs.transform.translation.x;
    start.pose.position.y = tfs.transform.translation.y;
    start.pose.position.z = tfs.transform.translation.z;

    start.pose.orientation.x = tfs.transform.rotation.x;
    start.pose.orientation.y = tfs.transform.rotation.y;
    start.pose.orientation.z = tfs.transform.rotation.z;
    start.pose.orientation.w = tfs.transform.rotation.w;

    traj.poses.push_back(start);
    pub.publish(traj);

    ros::spinOnce();
    r.sleep();
  }

  return 0;

}
