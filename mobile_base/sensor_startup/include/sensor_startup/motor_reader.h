#ifndef MOTOR_READER_H
#define MOTOR_READER_H

#include <iostream>
#include <cmath>
#include <string>
#include <vector>
#include <sstream>
#include <unistd.h>
#include <boost/thread.hpp>

#include "yaml-cpp/yaml.h"

#include "ros/ros.h"
#include "std_msgs/Bool.h"
#include "std_msgs/Int64MultiArray.h"
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "tf/transform_listener.h"
#include "tf/transform_broadcaster.h"
#include "serial/serial.h"

#include "CanAssist.h"
#include "controlcan.h"

namespace mobile_base {

struct IdConfig {
  int walk_chn1;
  int walk_chn2;
  int steer_chn1;
  int steer_chn2;
};

enum MotionMode {
  FORWARD,
  ROTATE
};

class MotorReader {
 public:
  MotorReader();
  virtual ~MotorReader();
  void ParamInit();
  void ReadFile(const std::string& address);
  void Setup();
  bool DriverInit();
  bool SetMode();
  bool EnableMotor();
  void ModeCommand(const int& id_0, const int& id_1, 
                   const int& len, const uint8_t& mode);
  void DataInitial(BYTE* data, uint8_t* cmd, const uint& len);

  VCI_CAN_OBJ* GetVciObject(const int& obj_num);
  void IdCheck();
  bool SendCommand(PVCI_CAN_OBJ obj, const uint& len);

  void ControlCallback(const sensor_msgs::JointState& joint_state);
  void DataTransform(BYTE* data, uint8_t* cmd, const uint& len,
                     const uint8_t& index, const int& velo);

  void TeleopCallback(const geometry_msgs::Twist& twist);
  void FeedbackCallback();
  void StopCallback(const std_msgs::Bool& stop);
  void ControlMotor(const std::vector<float>& raw_state);
   
  void StopMotor();
  void FeedbackReq();
  std::vector<int> CommandTransform(const std::vector<float>& raw_state);
  void PrintTest(BYTE* data, const int& len, const std::string& str);

  int  FourByteHex2Int(uint8_t* data);
  bool ReadEncoder(int* encod_data);
  void Homing();

  void PublishOdometry(const sensor_msgs::JointState& joint_state);
  void OdomCallback(const nav_msgs::Odometry& odom_msg);
  double GetVariance(const std::vector<double>& data_vec);
  double ComputeMean(const std::vector<double>& data_vec);
  void Loop();

  void GetHomeCallback(const std_msgs::Int64MultiArray& home_state);

 protected:
  // COBID of multiple motor drivers
  int cob_id[4];
  // pre-defined commands as data arrays
  Command cmd;

  // this CAN device has two can channels
  // device type of CAN-Analyst, refer to controlcan.h
  int device_type;
  int device_index;
  int can_index;
  // id_num means the quantity of motor-drivers we use
  int id_num;
  
  // double cur_time;
  // double pre_time;
  ros::Time cur_time;
  ros::Time pre_time;

 private:

  /* LAUNCH PARAMETERS */
  // port connected with CAN-USB converter
  std::string port;
  // topic used to publish joint states
  std::string state_topic;
  // topic used to publish raw odometry
  std::string raw_odom_topic;
  // address of yaml file containing motor driver configurations
  std::string file_address;
  // delay time used to pause between two commands
  int delay_time;
  // time to determine how long shoud it wait if buffer is empty
  int wait_time;
  // the publish period of motor states
  double state_pub_period;


  /* CONFIG PARAMETERS */
  YAML::Node param;
  

  // working mode of motors:
  // 0 -> position servo
  // 1 -> velocity servo
  int steering_mode;
  int walking_mode;

 protected:
  // lines of encoders
  uint encoder_s;
  uint encoder_w;
  uint abs_encoder;
  int freq_multiplier;
 private:
  // reduction ratio of steering or walking motors
  double reduc_ratio_s;
  double reduc_ratio_w;
  // max value of velocity when used in position mode
  uint max_velocity;
  // command sign of each motor
  int motor_sign[8];
 protected:
  // homing position
  int home[4];
 private:
  // homing value of absolute encoder
  int abs_home[4];
  // error limit to judge whether steering motor finishes homing
  int homing_error_limit;
  double variance_limit;
  // PID parameters in homing process
  double home_kp;
  double home_ki;
  double home_kd;

  
  /* GLOBAL VARIABLES */
 protected:
  bool if_initial;
  bool if_home_finish;
  bool if_get_initial_ekf_odom;
  double preset_steer_angle;
  double base_rotate_radius;
  double wheel_radius;
 private:

  ros::NodeHandle nh;
  ros::NodeHandle n_private;
  ros::Publisher state_pub;
  ros::Publisher raw_odom_pub;

  ros::Subscriber control_sub;
  ros::Subscriber teleop_sub;
  ros::Subscriber stop_sub;
  ros::Subscriber odom_sub;
  ros::Subscriber home_sub;

  // nav_msgs::Odometry raw_odom;
  nav_msgs::Odometry filtered_odom;
  /* THREADS */ 
  boost::thread* state_pub_thread;


};  // class MotorReader

}  // namespace mobile_base

#endif
