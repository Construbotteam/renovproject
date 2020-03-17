#ifndef TELEOP_H
#define TELEOP_H

#include <termio.h>
#include <iostream>
#include <cmath>

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"

namespace teleop {

enum Mode {
  FORWARD,
  ROTATE
};

class Teleop {
 public:
  Teleop();
  ~Teleop();
  void GetKey();
  void PubVelocity(int key_value);
 private:
  double v;  // uint : m/s
  double v_max;
  // angular velocity
  double av;  // uint : rad/s 
  double av_max;
  Mode mode;

  ros::NodeHandle n_private;
  ros::NodeHandle nh;
  ros::Publisher velo_pub;
};  // class Teleop

}  // namespace teleop

#endif