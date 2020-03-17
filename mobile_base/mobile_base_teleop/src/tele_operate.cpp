#include <stdio.h>
#include <termio.h>
#include <string>
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"

namespace mobile_base {

enum TeleopStatus { FORWARD, STRAIGHT, ROTATE };

class Teleop {
 public:
  Teleop(ros::NodeHandle& nh, ros::NodeHandle& nh_private);
  virtual ~Teleop() {}
  void paramInit(ros::NodeHandle& nh_private);
  int scanKeyboard();
  void teleop(const ros::TimerEvent&);
  void forwardMove(const double& v);
  void straightMove(const double& v);
  void rotateMove(const double& v);
  void showInstruction();

 private:
  std::string teleop_signal_topic_;
  double max_angular_v_;
  double max_linear_v_;
  double linear_v_increment_;
  double angular_v_increment_;

  TeleopStatus status_;
  double linear_v_;
  double angular_v_;

  sensor_msgs::JointState teleop_signal_;
  ros::Publisher teleop_signal_pub_;
  ros::Timer teleop_timer_;
};

Teleop::Teleop(ros::NodeHandle& nh, ros::NodeHandle& nh_private) {
  paramInit(nh_private);
  status_ = FORWARD;
  linear_v_ = angular_v_ = 0.0;

  teleop_signal_.header.frame_id = "motor";
  teleop_signal_.name.resize(8);
  teleop_signal_.position.resize(8);
  teleop_signal_.velocity.resize(8);
  teleop_signal_.name = {"front_left_walking",  "front_right_walking",
                         "rear_left_walking",   "rear_right_walking",
                         "front_left_steering", "front_right_steering",
                         "rear_left_steering",  "rear_right_steering"};

  teleop_signal_pub_ =
      nh.advertise<sensor_msgs::JointState>(teleop_signal_topic_, 10);
  teleop_timer_ = nh.createTimer(ros::Duration(1.0 / 5), &Teleop::teleop, this);
}

void Teleop::paramInit(ros::NodeHandle& nh_private) {
  nh_private.param("teleop_signal_topic", teleop_signal_topic_,
                   std::string("cmd_base_joint"));
  nh_private.param("max_linear_v", max_linear_v_, 0.2);
  nh_private.param("max_angular_v", max_angular_v_, 0.2);
  nh_private.param("linear_v_increment", linear_v_increment_, 0.05);
  nh_private.param("angular_v_increment", angular_v_increment_, 0.08);
}

void Teleop::teleop(const ros::TimerEvent&) {
  char key = scanKeyboard();

  switch (key) {
    case 'f': {
      status_ = FORWARD;
      linear_v_ = 0.0;
      ROS_INFO("set status FORWARD");
      forwardMove(linear_v_);
      break;
    }
    case 'r': {
      status_ = ROTATE;
      angular_v_ = 0.0;
      ROS_INFO("set status ROTATE");
      rotateMove(angular_v_);
      break;
    }
    case 'c': {
      status_ = STRAIGHT;
      linear_v_ = 0.0;
      ROS_INFO("set status STRAIGHT");
      straightMove(linear_v_);
      break;
    }
    case 'w': {
      if (status_ == FORWARD) {
        linear_v_ += linear_v_increment_;
        if (fabs(linear_v_) > max_linear_v_) {
          linear_v_ = max_linear_v_;
        }
        forwardMove(linear_v_);
      }
      break;
    }
    case 'x': {
      if (status_ == FORWARD) {
        linear_v_ -= linear_v_increment_;
        if (fabs(linear_v_) > max_linear_v_) {
          linear_v_ = -max_linear_v_;
        }
        forwardMove(linear_v_);
      }
      break;
    }
    case 'a': {
      if (status_ == ROTATE) {
        angular_v_ += angular_v_increment_;
        if (fabs(angular_v_) > max_angular_v_) {
          angular_v_ = max_angular_v_;
        }
        rotateMove(angular_v_);
      }
      break;
    }
    case 'd': {
      if (status_ == ROTATE) {
        angular_v_ -= angular_v_increment_;
        if (fabs(angular_v_) > max_angular_v_) {
          angular_v_ = -max_angular_v_;
        }
        rotateMove(angular_v_);
      }
      break;
    }
    case 'W': {
      if (status_ == STRAIGHT) {
        linear_v_ += linear_v_increment_;
        if (fabs(linear_v_) > max_linear_v_) {
          linear_v_ = max_linear_v_;
        }
        straightMove(linear_v_);
      }
      break;
    }
    case 'X': {
      if (status_ == STRAIGHT) {
        linear_v_ -= linear_v_increment_;
        if (fabs(linear_v_) > max_linear_v_) {
          linear_v_ = -max_linear_v_;
        }
        straightMove(linear_v_);
      }
      break;
    }

    case 's': {
      status_ = FORWARD;
      linear_v_ = 0.0;
      forwardMove(linear_v_);
      break;
    }
    case 'S': {
      status_ = FORWARD;
      linear_v_ = 0.0;
      forwardMove(linear_v_);
      break;
    }
    case '/': {
      ROS_INFO("quit tele controller");
      exit(0);
    }
    default: {
      showInstruction();
      break;
    }
  }
}

void Teleop::forwardMove(const double& v) {
  teleop_signal_.header.stamp = ros::Time::now();
  teleop_signal_.position = {0, 0, 0, 0, 0, 0, 0, 0};
  teleop_signal_.velocity = {v, v, v, v, 0, 0, 0, 0};
  std::cout << "current state : ";
  std::cout << std::setprecision(3) << "linear -> " << v << ", rotate -> 0.00"
            << std::endl;

  teleop_signal_pub_.publish(teleop_signal_);
}

void Teleop::straightMove(const double& v) {
  teleop_signal_.header.stamp = ros::Time::now();

  double a = M_PI / 2;
  teleop_signal_.position = {0, 0, 0, 0, a, a, a, a};
  teleop_signal_.velocity = {v, v, v, v, 0, 0, 0, 0};
  std::cout << "current state : ";
  std::cout << std::setprecision(3) << "straight -> " << v << ", rotate -> 0.00"
            << std::endl;

  teleop_signal_pub_.publish(teleop_signal_);
}

void Teleop::rotateMove(const double& v) {
  double wheel_dis_len = 0.5;
  double wheel_dis_wid = 0.395;
  double hypotenuse = sqrt(pow(wheel_dis_len, 2) + pow(wheel_dis_wid, 2));
  double r = 0.5 * hypotenuse;

  double steer_angle, v_wheel;
  steer_angle = fabs(atan(wheel_dis_len / wheel_dis_wid));
  v_wheel = v * r / (double)0.15;

  teleop_signal_.position = {
      0, 0, 0, 0, -steer_angle, steer_angle, steer_angle, -steer_angle};
  teleop_signal_.velocity = {-v_wheel, v_wheel, -v_wheel, v_wheel, 0, 0, 0, 0};
  std::cout << "current state : ";
  std::cout << std::setprecision(3) << "straight -> 0.00"
            << ", rotate -> " << v << std::endl;

  teleop_signal_pub_.publish(teleop_signal_);
}

int Teleop::scanKeyboard() {
  int in;
  struct termios new_settings;
  struct termios stored_settings;
  tcgetattr(0, &stored_settings);
  new_settings = stored_settings;
  new_settings.c_lflag &= (~ICANON);
  new_settings.c_cc[VTIME] = 0;
  tcgetattr(0, &stored_settings);
  new_settings.c_cc[VMIN] = 1;
  tcsetattr(0, TCSANOW, &new_settings);

  in = getchar();

  tcsetattr(0, TCSANOW, &stored_settings);
  return in;
}

void Teleop::showInstruction() {
  std::cout << "-------------------------------" << std::endl;
  std::cout << "*******" << std::endl;

  std::cout << "'f' -> forward movement mode" << std::endl;
  std::cout << "'w' -> forward, 'x' -> backward, 's' or 'S' -> brake"
            << std::endl;

  std::cout << "*******" << std::endl;

  std::cout << "'r' -> pure rotation movement mode" << std::endl;
  std::cout << "'a' -> anti-clockwise, 'd' -> clockwise, 's' or 'S' -> brake"
            << std::endl;

  std::cout << "*******" << std::endl;

  std::cout << "'c' -> straight movement mode" << std::endl;
  std::cout << "'W' -> forward, 'X' -> backward, 's' or 'S' -> brake"
            << std::endl;

  std::cout << "*******" << std::endl;
  std::cout << "-------------------------------" << std::endl;
}
}  // namespace mobile_base

int main(int argc, char** argv) {
  ros::init(argc, argv, "tele_operate");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  mobile_base::Teleop tele(nh, nh_private);

  ros::spin();
  return 0;
}
