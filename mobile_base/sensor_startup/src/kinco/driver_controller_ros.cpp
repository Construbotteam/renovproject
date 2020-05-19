#include "sensor_startup/kinco/driver_controller_ros.h"

namespace mobile_base {

DriverControllerROS::DriverControllerROS(ros::NodeHandle nh,
                                         ros::NodeHandle nh_private)
    : DriverController() {
  ParamInit(nh_private);
  DebugData(if_debug_);

  GetBaseAddress(base_file_address_);
  ActivateCAN(base_file_address_ + can_config_address_);

  ReadDriverFile(driver_config_address_);
  if_initial_ = DriverInit();

  joint_state_pub_ =
      nh.advertise<sensor_msgs::JointState>(joint_state_pub_topic_, 100);

  control_signal_sub_ = nh.subscribe(
      "cmd_vel_base", 10, &DriverControllerROS::GetControlSignalCallback, this);

  stop_signal_sub_ = nh.subscribe(
      "stop_driver", 10, &DriverControllerROS::DriverStopCallback, this);

  fb_thread_ = new std::thread(&DriverControllerROS::FeedbackLoop, this);
}

DriverControllerROS::~DriverControllerROS() {
  if (fb_thread_) {
    delete fb_thread_;
  }
}

void DriverControllerROS::ParamInit(ros::NodeHandle nh_private) {
  nh_private.param("joint_state_pub_topic", joint_state_pub_topic_,
                   std::string("motor_joint_state"));
  nh_private.param("if_debug", if_debug_, false);
  nh_private.param("can_config_address", can_config_address_,
                   std::string("/config/kinco/can_config.yaml"));
  nh_private.param("driver_config_address", driver_config_address_,
                   std::string("/config/kinco/driver_config.yaml"));
  nh_private.param(
      "base_file_address", base_file_address_,
      std::string("/home/renov_robot/renov_ws/src/renov_robot/sensor_startup"));
}

void DriverControllerROS::GetControlSignalCallback(
    const sensor_msgs::JointState& js_msg) {
  if (!if_initial_) {
    ROS_WARN("The driver controller is waiting for initialization");
    return;
  }
  if (js_msg.name.size() != id_num_) {
    ROS_WARN("Incorrect number of joint states");
    return;
  }

  std::vector<double> control_singal;
  for (size_t i = 0; i < walk_id_num_; i++) {
    control_singal.push_back(js_msg.velocity[i]);
  }
  for (size_t i = 0; i < steer_id_num_; i++) {
    control_singal.push_back(js_msg.position[i + walk_id_num_]);
  }

  ControlMotor(control_singal);
}

void DriverControllerROS::DriverStopCallback(
    const std_msgs::Bool& stop_signal) {
  DriverStop();
  DriverDisenable();
}

void DriverControllerROS::FeedbackLoop() {
  double* walk_fb;
  double* steer_fb;
  while (ros::ok()) {
    GetFeedback(walk_fb, steer_fb);
    ros::Duration(0.05).sleep();
  }
}

}  // namespace mobile_base
