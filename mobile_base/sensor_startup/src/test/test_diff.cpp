#include <string>

#include "geometry_msgs/Twist.h"
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "sensor_startup/kinco/can_application.h"

namespace mobile_base {

union IntHexTF {
  int num_;
  uint8_t byte_[4];
};  // union IntHexTF

class DiffDriverReader {
 public:
  DiffDriverReader(ros::NodeHandle& nh, ros::NodeHandle& nh_private);
  virtual ~DiffDriverReader();
  void initParam(ros::NodeHandle& nh_private);
  void startPDO();
  void driverStart();
  void feedbackRequest();
  void getFeedback();
  void getTwistCallback(const geometry_msgs::Twist& twist);

  void disable();
  inline double velocityTransform(int& velo) {
    return (double)(velo * right_motor_sign_ * 1875.0) /
           (double)(encoder_w_ * freq_multiplier_ * 512) / (double)reduc_ratio_;
  }
  inline double positionTransform(int& posi) {
    return (double)(posi * right_motor_sign_ * 2 * M_PI * wheel_radius_) /
           (double)(reduc_ratio_ * freq_multiplier_ * encoder_w_);
  }

 private:
  std::string can_config_addr_;
  std::string js_topic_, cmd_vel_topic_;
  CanApplication can_app_;
  double wheel_track_, wheel_radius_;
  double reduc_ratio_;
  int encoder_w_, freq_multiplier_;
  int left_motor_sign_, right_motor_sign_;

  ros::Publisher js_pub_;
  ros::Subscriber cmd_vel_sub_;
  sensor_msgs::JointState js_msg_;
};  // class DiffDriverReader

DiffDriverReader::DiffDriverReader(ros::NodeHandle& nh,
                                   ros::NodeHandle& nh_private) {
  initParam(nh_private);
  can_app_.ActivateCAN(can_config_addr_);

  startPDO();
  driverStart();

  js_msg_.header.frame_id = "wheel";
  js_msg_.name.resize(2);
  js_msg_.name = {"left_wheel", "right_wheel"};
  js_msg_.position.resize(2);
  js_msg_.velocity.resize(2);

  js_pub_ = nh.advertise<sensor_msgs::JointState>(js_topic_, 10);
  cmd_vel_sub_ = nh.subscribe(cmd_vel_topic_, 10,
                              &DiffDriverReader::getTwistCallback, this);
}

DiffDriverReader::~DiffDriverReader() {
  disable();
  ros::Duration(0.5).sleep();
  can_app_.CloseCAN();
}

void DiffDriverReader::initParam(ros::NodeHandle& nh_private) {
  nh_private.param("can_config_addr", can_config_addr_, std::string(""));
  nh_private.param("js_topic", js_topic_, std::string("base_joint"));
  nh_private.param("cmd_vel_topic", cmd_vel_topic_, std::string("cmd_vel"));
  nh_private.param("wheel_radius", wheel_radius_, 0.09);
  nh_private.param("wheel_track", wheel_track_, 0.5);
  nh_private.param("encoder_w", encoder_w_, 2500);
  nh_private.param("reduc_ratio", reduc_ratio_, 20.0);
  nh_private.param("freq_multiplier", freq_multiplier_, 4);
  nh_private.param("left_motor_sign", left_motor_sign_, 1);
  nh_private.param("right_motor_sign", right_motor_sign_, 1);
}

void DiffDriverReader::startPDO() {
  PVCI_CAN_OBJ pdo_start_obj = can_app_.GetVciObject(1, 0x00);
  pdo_start_obj->DataLen = 2;
  pdo_start_obj->Data[0] = 0x01;
  pdo_start_obj->Data[1] = 0x00;

  can_app_.SendCommand(pdo_start_obj, 1);
  delete[] pdo_start_obj;
}

void DiffDriverReader::driverStart() {
  PVCI_CAN_OBJ walk_cmd_obj = can_app_.GetVciObject(2, 0x00);
  walk_cmd_obj[0].ID = 0x201;
  walk_cmd_obj[1].ID = 0x202;

  walk_cmd_obj[0].Data[0] = 0x0f;
  walk_cmd_obj[0].Data[1] = 0x00;
  walk_cmd_obj[0].Data[2] = 0x03;

  walk_cmd_obj[1].Data[0] = 0x0f;
  walk_cmd_obj[1].Data[1] = 0x00;
  walk_cmd_obj[1].Data[2] = 0x03;

  walk_cmd_obj[0].DataLen = 3;
  walk_cmd_obj[1].DataLen = 3;

  can_app_.SendCommand(walk_cmd_obj, 2);
  delete[] walk_cmd_obj;
}

void DiffDriverReader::feedbackRequest() {
  PVCI_CAN_OBJ req_obj = can_app_.GetVciObject(1, 0x80);
  req_obj->ID = 0x80;
  req_obj->DataLen = 0;

  can_app_.SendCommand(req_obj, 1, true);
  delete[] req_obj;
}

void DiffDriverReader::getFeedback() {
  feedbackRequest();
  js_msg_.header.stamp = ros::Time::now();
  ros::Duration(0.01).sleep();

  int read_buffer_len = 100;
  PVCI_CAN_OBJ data_obj = new VCI_CAN_OBJ[read_buffer_len];
  can_app_.GetData(data_obj, read_buffer_len);

  IntHexTF tfmer_v, tfmer_p;
  double left_velo, right_velo, left_posi, right_posi;
  int count = 0;
  for (size_t i = 0; i < read_buffer_len; i++) {
    switch (data_obj[i].ID) {
      case 0x281: {
        for (size_t j = 0; j < 4; j++) {
          tfmer_v.byte_[j] = data_obj[i].Data[j];
          tfmer_p.byte_[j] = data_obj[i].Data[j + 4];
        }
        int int_velo = tfmer_v.num_;
        int int_posi = tfmer_p.num_;
        right_velo = velocityTransform(int_velo);
        right_posi = positionTransform(int_posi);

        count++;
        break;
      }
      case 0x282: {
        for (size_t j = 0; j < 4; j++) {
          tfmer_v.byte_[j] = data_obj[i].Data[j];
          tfmer_p.byte_[j] = data_obj[i].Data[j + 4];
        }
        int int_velo = tfmer_v.num_;
        int int_posi = tfmer_p.num_;
        left_velo = velocityTransform(int_velo);
        left_posi = positionTransform(int_posi);

        count++;
        break;
      }
      default: {
        break;
      }
    }
  }

  if (count != 2) {
    ROS_WARN("Data count is : %d rather than 2", count);
    return;
  }

  js_msg_.position = {left_posi, right_posi};
  js_msg_.velocity = {left_velo, right_velo};
  js_pub_.publish(js_msg_);

  delete[] data_obj;
}

void DiffDriverReader::getTwistCallback(const geometry_msgs::Twist& twist) {
  double vx = twist.linear.x;
  double az = twist.angular.z;

  double vr = vx + 0.5 * wheel_track_ * az;
  double vl = vx - 0.5 * wheel_track_ * az;

  // transform m/s to r/min
  double rr = vr / (2 * M_PI * wheel_radius_) * 60;
  double rl = vl / (2 * M_PI * wheel_radius_) * 60;

  int rr_signal = (int)(rr * 512 * encoder_w_ / 1875.0);
  int rl_signal = (int)(rl * 512 * encoder_w_ / 1875.0);

  // tfmer : abbr. of transformer
  IntHexTF tfmer_rr, tfmer_rl;
  tfmer_rr.num_ = rr_signal;
  tfmer_rl.num_ = rl_signal;

  PVCI_CAN_OBJ velo_obj = can_app_.GetVciObject(2, 0x00);
  velo_obj[0].ID = 0x201;
  velo_obj[1].ID = 0x202;

  velo_obj[0].Data[0] = 0x0f;
  velo_obj[1].Data[1] = 0x00;
  velo_obj[2].Data[2] = 0x03;

  for (size_t i = 3; i < 7; i++) {
    velo_obj[0].Data[i] = tfmer_rr.byte_[i - 3];
    velo_obj[1].Data[i] = tfmer_rl.byte_[i - 3];
  }
  velo_obj[0].DataLen = velo_obj[1].DataLen = 7;

  can_app_.SendCommand(velo_obj, 2);

  delete[] velo_obj;
}

void DiffDriverReader::disable() {
  PVCI_CAN_OBJ obj = can_app_.GetVciObject(2, 0x00);
  obj[0].ID = 0x201;
  obj[1].ID = 0x202;

  obj[0].DataLen = 3;
  obj[1].DataLen = 3;

  obj[0].Data[0] = 0x06;
  obj[0].Data[1] = 0x00;
  obj[0].Data[2] = 0x03;

  obj[1].Data[0] = 0x06;
  obj[1].Data[1] = 0x00;
  obj[1].Data[2] = 0x03;

  can_app_.SendCommand(obj, 2);
  delete[] obj;
}

}  // namespace mobile_base

int main(int argc, char** argv) {
  ros::init(argc, argv, "test_diff");
  ros::NodeHandle nh;
  ros::NodeHandle nh_private("~");

  mobile_base::DiffDriverReader reader(nh, nh_private);
  ros::spin();

  return 0;
}