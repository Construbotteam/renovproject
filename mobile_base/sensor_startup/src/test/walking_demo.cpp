#include <string>
#include <unistd.h>

#include "yaml-cpp/yaml.h"

#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "sensor_msgs/JointState.h"

#define PI 3.141592653

struct Motion {
  double v;
  double t;
};

struct DemoSetting {
  Motion forward_1;
  Motion lateral;
  std::vector<Motion> forward_2;
  int size_of_2;
};



class Demo {
  public:
   Demo();
   Demo(std::string& addr, ros::NodeHandle nh);
   void Forward(const double& v);
   void ForwardStart(const double& v_cur, const double& v_goal);
   void ForwardBrake(const double& v_cur);
   void Rotary(const double& a);
   void RotaryStart(const double& a_cur, const double& a_goal);
   void RotaryBrake(const double& a_cur);
   void Lateral(const double& v);
   void LateralStart(const double& v_cur, const double& v_goal);
   void LateralBrake(const double& v_cur);
   void WalkingDemo();
   void ReadFile(const std::string& address);
   void CooperateCallback(const std_msgs::Bool& data);

  private:
   int t;
   int T;
   int index;
   DemoSetting demo_set;
   bool flag;
   ros::Publisher state_pub;
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "walking_demo");
  ros::NodeHandle nh;

  std::cout << "ready to start !! " << std::endl;
  sleep(1);

  // std::string address =
  //   "/home/renov_robot/catkin_ws/src/renov_robot/sensor_startup/data/demo_config.yaml";
  std::string address =
    "/home/glh/renov_ws/src/renov_robot/sensor_startup/data/demo_config.yaml";
  Demo demo(address, nh);
  ros::Subscriber sub = 
    nh.subscribe("signal", 100, &Demo::CooperateCallback, &demo);
  ros::spin();

  return 0;
}
Demo::Demo(std::string& addr, ros::NodeHandle nh) {
  int index = 0;
  ReadFile(addr);
  state_pub = nh.advertise<sensor_msgs::JointState>("cmd_base_joint", 100);
}

void Demo::CooperateCallback(const std_msgs::Bool& data) {
  if (0 == index) {
    ForwardStart(0, demo_set.forward_1.v);
    Forward(0);
    usleep(demo_set.forward_1.t);
    ForwardBrake(demo_set.forward_1.v);
  }
  if (1 == index) {
    Lateral(0);
    usleep(500000);
    LateralStart(0, demo_set.lateral.v);
    Lateral(demo_set.lateral.v);
    usleep(demo_set.lateral.t);
    LateralBrake(demo_set.lateral.v);
    Forward(0);
    usleep(500000);
  }
  if (index >= 1) {
    if (index > demo_set.size_of_2) {
      ROS_WARN("wrong index!!");
      return ;
    }
    ForwardStart(0, demo_set.forward_2[index - 1].v);    
    Forward(demo_set.forward_2[index - 1].v);
    usleep(demo_set.forward_2[index - 1].t);
    ForwardBrake(demo_set.forward_2[index - 1].v);
    Forward(0);
  }
  index++;
}



void Demo::ReadFile(const std::string& address) {
  YAML::Node param = YAML::LoadFile(address);

  T = param["T"].as<int>();
  t = param["t"].as<int>();
  demo_set.forward_1.v = param["forward_1"]["v"].as<double>();
  demo_set.forward_1.t = param["forward_1"]["t"].as<double>();
  demo_set.lateral.v = param["lateral"]["v"].as<double>();
  demo_set.lateral.t = param["lateral"]["t"].as<double>();

  demo_set.size_of_2 = param["forward_2"]["size"].as<int>();
  demo_set.forward_2.resize(demo_set.size_of_2);
  for (size_t i = 0; i < demo_set.size_of_2; i++) {
    demo_set.forward_2[i].v = param["forward_2"]["v"][i].as<double>();
    demo_set.forward_2[i].t = param["forward_2"]["t"][i].as<double>();
    std::cout << "read the data" << std::endl;
  }
  std::cout << "read file finish" << std::endl;
}

void Demo::Forward(const double& v) {

  if (v*22.5 > 1000) {
    ROS_WARN("velocity is too fast!!");
    return ;
  }

  sensor_msgs::JointState js;
  js.header.frame_id = "motor";
  js.header.stamp = ros::Time::now();
  js.name.resize(8);
  js.name = {"front_left_walking",   "front_right_walking",
             "rear_left_walking",    "rear_right_walking",
             "front_right_steering", "front_left_steering",
             "rear_right_steering",  "rear_left_steering"};
  js.velocity.resize(8);
  js.velocity = {v, v, v, v, 0, 0, 0, 0};

  js.position.resize(8);
  js.position = {0, 0, 0, 0, 0, 0, 0, 0};

  state_pub.publish(js);
}

void Demo::ForwardStart(const double& v_cur, const double& v_goal) {
  double acc = 1.0;
  double vel = v_cur;

  if (vel < v_goal) {
    while (vel < v_goal) {
      Forward(vel);
      usleep(t);
      vel += acc;
    }
  } else {
    while (vel > v_goal) {
      Forward(vel);
      usleep(t);
      vel -= acc;
    }
  }
  Forward(v_goal);
  usleep(T);
}

void Demo::ForwardBrake(const double& v_cur) {
  double acc = 1.0;
  double vel = v_cur;

  if (vel > 0) {
    while (vel > 0) {
      Forward(vel);
      usleep(t);
      vel -= acc;
    }
  } else {
    while (vel < 0) {
      Forward(vel);
      usleep(t);
      vel += acc;
    }
  }
  Forward(0);
  usleep(T);
}


void Demo::Rotary(const double& a) {

  sensor_msgs::JointState js;
  js.header.frame_id = "motor";
  js.header.stamp = ros::Time::now();
  js.name.resize(8);
  js.name = {"front_left_walking",   "front_right_walking",
             "rear_left_walking",    "rear_right_walking",
             "front_right_steering", "front_left_steering",
             "rear_right_steering",  "rear_left_steering"};

  double wheel_dis_len, wheel_dis_wid;
  wheel_dis_len = 0.5;
  wheel_dis_wid = 0.395;
  double hypotenuse = sqrt(pow(wheel_dis_len, 2) + pow(wheel_dis_wid, 2));
  double r = 0.5 * hypotenuse;

  float alpha = fabs(atan(wheel_dis_len / wheel_dis_wid));
  float v_linear = a * r/(0.15/2);
  float n = v_linear * 60 / (2 * M_PI);
  js.velocity.resize(8);
  js.position.resize(8);

  js.position = {0, 0, 0, 0, -alpha, alpha, alpha, -alpha};
  js.velocity = {-n, n, -n, n, 0, 0, 0, 0};

  state_pub.publish(js);
}

void Demo::RotaryStart(const double& a_cur, const double& a_goal) {
  double acc = 0.04;
  double ang = a_cur;

  if (ang < a_goal) {
    while (ang < a_goal) {
      Rotary(ang);
      ang += acc;
      usleep(t);
    }
  } else {
    while (ang > a_goal) {
      Rotary(ang);
      ang -= acc;
      usleep(t);
    }
  }
  Rotary(a_goal);
  usleep(T);
}

void Demo::RotaryBrake(const double& a_cur) {
  double acc = 0.04;
  double ang = a_cur;

  if (ang > 0) {
    while (ang > 0) {
      Rotary(ang);
      ang -= acc;
      usleep(t);
    }
  } else {
    while (ang < 0) {
      Rotary(ang);
      ang += acc;
      usleep(t);
    }
  }
  Rotary(0);
  usleep(T);
}

void Demo::Lateral(const double& v) {
  
  if (v*22.5 > 1000) {
    ROS_WARN("too fast!!");
    return ;
  }
  sensor_msgs::JointState js;
  js.header.frame_id = "motor";
  js.header.stamp = ros::Time::now();
  js.name.resize(8);
  js.name = {"front_left_walking",   "front_right_walking",
             "rear_left_walking",    "rear_right_walking",
             "front_right_steering", "front_left_steering",
             "rear_right_steering",  "rear_left_steering"};

  js.velocity.resize(8);
  js.position.resize(8);

  double angle = -0.5 * M_PI;
  js.position = {0, 0, 0, 0, angle, angle, angle, angle};
  js.velocity = {v, v, v, v, 0, 0, 0, 0};

  state_pub.publish(js);  
}

void Demo::LateralStart(const double& v_cur, const double& v_goal) {
  
  double acc = 1.0;
  double vel = v_cur;

  if (vel < v_goal) {
    while (vel < v_goal) {
      Lateral(vel);
      usleep(t);
      vel += acc;
    }
  } else {
    while (vel > v_goal) {
      Lateral(vel);
      usleep(t);
      vel -= acc;
    }
  }
  Lateral(v_goal);
  usleep(T);
}

void Demo::LateralBrake(const double& v_cur) {
  double acc = 1.0;
  double vel = v_cur;

  if (vel > 0) {
    while (vel > 0) {
      Lateral(vel);
      usleep(t);
      vel -= acc;
    }
  } else {
    while (vel < 0) {
      Lateral(vel);
      usleep(t);
      vel += acc;
    }
  }
  Lateral(0);
  usleep(T);
}

void Demo::WalkingDemo() {

  int t_tmp = 3000000;
  double angular = 0.5424;

  ForwardStart(0, 20);

  Forward(20);
  usleep(t_tmp);

  ForwardBrake(20);

  // ============================
  Rotary(0);
  usleep(1000000);

  RotaryStart(0, angular);

  Rotary(angular);
  usleep(t_tmp);

  RotaryBrake(angular);

  // ===========================
  RotaryStart(0, -angular);

  Rotary(-angular);
  usleep(t_tmp);

  RotaryBrake(-angular);

  // ==========================
  Forward(0);
  usleep(1000000);

  ForwardStart(0, -20);

  Forward(-20);
  usleep(t_tmp);

  ForwardBrake(-20);

  // =========================
  Lateral(0);
  usleep(1000000);
  
  LateralStart(0, 20);

  Lateral(20);
  usleep(t_tmp);

  LateralBrake(20);

  // =========================
  Lateral(0);
  usleep(1000000);
  
  LateralStart(0, -20);
  
  Lateral(-20);
  usleep(t_tmp);

  LateralBrake(-20);
    
  Forward(0);
  usleep(1000000);
  

}
