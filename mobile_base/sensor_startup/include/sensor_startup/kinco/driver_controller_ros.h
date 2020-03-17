#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "std_msgs/Bool.h"

#include "driver_controller.h"

namespace mobile_base {

class DriverControllerROS : public DriverController {
 public:
  DriverControllerROS(ros::NodeHandle nh, ros::NodeHandle nh_private);
  virtual ~DriverControllerROS();
  void ParamInit(ros::NodeHandle nh_private);
  void GetControlSignalCallback(const sensor_msgs::JointState& js_msg);
  void DriverStopCallback(const std_msgs::Bool& stop_signal);

 private:
  bool if_debug_;
  bool if_initial_;
  std::string can_config_address_;
  std::string driver_config_address_;
  std::string base_file_address_;
  std::string joint_state_pub_topic_;

  ros::Subscriber control_signal_sub_;
  ros::Subscriber stop_signal_sub_;
  ros::Publisher joint_state_pub_;

};  // class DriverControllerROS

}  // namespace mobile_base
