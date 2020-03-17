#include "ros/ros.h"
#include "std_msgs/Bool.h"



int main(int argc, char** argv) {
  ros::init(argc, argv, "stop_motor");
  ros::NodeHandle nh;

  ros::Publisher pub = nh.advertise<std_msgs::Bool>("stop_driver", 100);
  ros::Rate r(10);

  int n = 0;
  while (ros::ok()) {
    if (n < 4) {
      std_msgs::Bool stop_cmd;
      stop_cmd.data = true;
      pub.publish(stop_cmd);
    }
    n++;

    ros::spinOnce();
    r.sleep();
  }
  // std_msgs::Bool stop_cmd;
  // stop_cmd.data = true;
  // pub.publish(stop_cmd);
  // ros::spinOnce();
  return 0;
}
