#include "mobile_base_teleop/Teleop.h"

using teleop::Teleop;

Teleop::Teleop() {
  v = 0.0;
  av = 0.0;
  mode = FORWARD;

  n_private = ros::NodeHandle("tele_operate");
  if (!n_private.getParam("v_max", v_max)) {
    v_max = 0.3;
  }
  if (!n_private.getParam("av_max", av_max)) {
    av_max = 0.174;
  }
  velo_pub = nh.advertise<geometry_msgs::Twist>("cmd_vel", 100);
}

Teleop::~Teleop() {
}

void Teleop::GetKey() {
  int key_value;
  struct termios new_settings;
  struct termios stored_settings;
  tcgetattr(0, &stored_settings);
  new_settings = stored_settings;
  new_settings.c_lflag &= (~ICANON);
  new_settings.c_cc[VTIME] = 0;
  tcgetattr(0, &stored_settings);
  new_settings.c_cc[VMIN] = 1;
  tcsetattr(0, TCSANOW, &new_settings);

  key_value = getchar();
  PubVelocity(key_value);
  
  tcsetattr(0, TCSANOW, &stored_settings);
}

void Teleop::PubVelocity(int key_value) {
  
  if (0.0 == v && 0.0 == av) {
    switch (key_value) {
      case 'q': {
        mode = FORWARD;
        std::cout << "  --  FORWARD MODE" << std::endl;
        break;
      }
      case 'e': {
        mode = ROTATE;
        std::cout << "  --  ROTATE MODE" << std::endl;
        break;
      }
      default: break;
    }
  }

  if (FORWARD == mode) {
    switch (key_value) {
      case 'w': {
        v += 0.01;
        if (v >= 0) {
          std::cout << "  --  moving forward at linear speed : " << v << "m/s"
                    << std::endl;
        } else {
          std::cout << "  --  moving backward at linear speed : " << v << "m/s"
                    << std::endl;
        }
        break;
      }
      case 'x': {
        v -= 0.01;
        if (v >= 0) {
          std::cout << "  --  moving forward at linear speed : " << v << "m/s"
                    << std::endl;
        } else {
          std::cout << "  --  moving backward at linear speed : " << v << "m/s"
                    << std::endl;
        }
        break;
      }
      case 's': {
        v = 0.0;
        std::cout << "  --  brake !!!!" << std::endl;
        break;
      }
      default: {
        std::cout << "  --  incorrect commands at " << mode << " mode" 
                  << std::endl;
        break;
      }
    }

    if (fabs(v) > v_max) {
      v = pow(-1, std::signbit(v)+2) * v_max;
    }
    geometry_msgs::Twist twist;
    twist.linear.x = v;
    velo_pub.publish(twist);
    
  }  // mechanism of forward mode
  
  if (ROTATE == mode) {
    switch (key_value) {
      case 'a': {
        av += 0.017;
        if (av >= 0) {
          std::cout << "  --  rotating anti-clockwise at angular speed : " << av
                    << " rad/s" << std::endl;
        } else {
          std::cout << "  --  rotating clockwise at angular speed : " << av
                    << " rad/s " << std::endl;
        }
        break;
      }
      case 'd': {
        av -= 0.017;
        if (av >= 0) {
          std::cout << "  --  rotating anti-clockwise at angular speed : " << av
                    << " rad/s" << std::endl;
        } else {
          std::cout << "  --  rotating clockwise at angular speed : " << av
                    << " rad/s" << std::endl;
        }
        break;
      }
      case 's': {
        av = 0.0;
        std::cout << "  --  brake !!!!" << std::endl;
        break;
      }
      default: {
        std::cout << "  --  incorrect commands at " << mode << " mode" 
                  << std::endl;
        break;
      }
    }
    
    if (fabs(av) > av_max) {
      av = pow(-1, std::signbit(av) + 2) * av_max;
    }
    geometry_msgs::Twist twist;
    twist.angular.z = av;
    velo_pub.publish(twist);
  }  // mechanism of rotate mode
  if ('p' == key_value) {
    geometry_msgs::Twist twist;
    twist.linear.x = twist.linear.y = twist.linear.z = 0.0;
    twist.angular.x = twist.angular.y = twist.angular.z = 0.0;
    velo_pub.publish(twist);
    std::cout << "  --  force stop before exit" << std::endl;
    exit(1);
  }
}