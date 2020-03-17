#include <fstream>
#include <iostream>
#include <string>

#include "ros/ros.h"
#include "tf/tf.h"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"

int main(int argc, char** argv) {
  ros::init(argc, argv, "way_point_note");
  ros::NodeHandle nh;

  std::string file_name;
  std::cout << "please enter the file name : ";
  std::cin >> file_name;
  std::string file_addr = "/home/glh/Desktop/" + file_name;

  std::ofstream out(file_addr, std::ios::app);
  std::cout << "data will be written to : " << file_addr << std::endl;

  if (out.is_open()) {
    tf2_ros::Buffer bf(ros::Duration(10.0));
    tf2_ros::TransformListener tfl(bf);

    geometry_msgs::TransformStamped tfstamped;

    while (true) {
      int k;
      std::cin >> k;
      if (0 == k) {
        break;
      }

      if (bf.canTransform("map", "base_link", ros::Time(0),
                          ros::Duration(0.2))) {
        try {
          tfstamped = bf.lookupTransform("map", "base_link", ros::Time(0),
                                         ros::Duration(0.1));
        } catch (tf2::TransformException& ex) {
          ROS_WARN("%s", ex.what());
          ros::Duration(1.0);
          continue;
        }
      } else {
        ROS_WARN("no transform");
        continue;
      }

      tf::Quaternion q(
          tfstamped.transform.rotation.x, tfstamped.transform.rotation.y,
          tfstamped.transform.rotation.z, tfstamped.transform.rotation.w);
      double yaw = tf::getYaw(q);

      out << tfstamped.transform.translation.x << "  "
          << tfstamped.transform.translation.y << "  " << yaw << std::endl;
    }
    out.close();
  }
  return 0;
}