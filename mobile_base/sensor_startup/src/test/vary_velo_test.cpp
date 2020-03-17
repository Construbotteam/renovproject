#include "ros/ros.h"
#include "geometry_msgs/Twist.h"


int main(int argc, char** argv) {
  ros::init(argc, argv, "vary_velo_test");
  ros::NodeHandle nh;

  ros::Publisher pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 10);
  ros::Rate r(10);

  geometry_msgs::Twist msg;
  double st = ros::Time::now().toSec();
  while (ros::ok()) {
    double cur_t = ros::Time::now().toSec(); 
    msg.linear.x = 4 * sin(cur_t - st);
    pub.publish(msg);

    ros::spinOnce();
    r.sleep();
  }
  return 0;

}
