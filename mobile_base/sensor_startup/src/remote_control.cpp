#include <cmath>
#include "sensor_startup/motor_reader.h"
#include "tf/transform_listener.h"
#include "tf/transform_broadcaster.h"



namespace mobile_base {

struct Pose2d {
  Pose2d() {}
  Pose2d(double x, double y, double z, double roll, double pitch, double yaw)
    : x(x), y(y), z(z), roll(roll), pitch(pitch), yaw(yaw) {} 
  double x;
  double y;
  double z;

  double roll;
  double pitch;
  double yaw;
};

class Tele : public MotorReader {
  public:
    Tele() : MotorReader() { 
      // SetTeleopMode();
      ReadTeleopFile(); 
      veh_pose = Pose2d(0, 0, 0, 0, 0, 0);
      fb_thread = NULL;
    }
    virtual ~Tele() { delete fb_thread; }
    // void SetTeleopMode();
    void ReadTeleopFile();
    void SendTf(const std::vector<float>& raw_state, const double& dt);
    void TeleControl(const geometry_msgs::Twist::ConstPtr& twist);
    void TeleStop(const std_msgs::Bool& data);
    void TeleFeedback(const double& period);
    void Loop();

  private:
    double wheel_dis_len;
    double wheel_dis_wid;
    // double pre_time;
    std::vector<double> cur_state;
    Pose2d veh_pose;  // pose of vehicle
    boost::thread* fb_thread;

    ros::NodeHandle nh;
    ros::Subscriber stop_sub;
    ros::Subscriber tele_sub;
    


};

// void Tele::SetTeleopMode() {
//   int len;

//   len = sizeof(cmd.SET_MODE_VELOCITY) / sizeof(cmd.SET_MODE_VELOCITY[0]);
//   ModeCommand(cob_id[0], cob_id[1], len, VELOCITY_MODE);

//   len = sizeof(cmd.SET_MODE_POSITION) / sizeof(cmd.SET_MODE_POSITION[0]);
//   ModeCommand(cob_id[2], cob_id[3], len, POSITION_MODE);
// }

void Tele::ReadTeleopFile() {
  YAML::Node config 
    = YAML::LoadFile("/home/renov_robot/catkin_ws/src/renov_robot/sensor_startup/data/teleop_config.yaml");
  wheel_dis_len = config["wheel_dis_len"].as<double>();
  wheel_dis_wid = config["wheel_dis_wid"].as<double>(); 

} 


void Tele::TeleControl(const geometry_msgs::Twist::ConstPtr& twist) {

  double vx = twist->linear.x;
  double az = twist->angular.z;
  
  if (vx != 0 && az == 0) {
    std::vector<float> velocity;
    for (size_t i = 0; i < 4; i++) {
      velocity.push_back((float)vx);
    }
    for (size_t i = 0; i < 4; i++) {
      velocity.push_back(0);
    }
    ControlMotor(velocity);
  } else if (vx == 0 && az != 0) {
    double hypotenuse = sqrt(pow(wheel_dis_len, 2) + pow(wheel_dis_wid, 2));
    double r = 0.5 * hypotenuse; 

    float alpha = fabs(atan(wheel_dis_len / wheel_dis_wid));
    std::vector<float> velocity;
    float v_linear = az * r / (float)0.15;
    velocity.resize(8);
    velocity = {-v_linear, v_linear, -v_linear, v_linear,
                   -alpha,    alpha,     alpha,   -alpha};
    ControlMotor(velocity);
  } else if (0 == vx && 0 == az) {
    std::vector<float> velocity;
    velocity.resize(8);
    velocity = {0, 0, 0, 0, 0, 0, 0, 0};
    ControlMotor(velocity);
  }
}

void Tele::TeleStop(const std_msgs::Bool& stop) {
  StopMotor();
}

void Tele::TeleFeedback(const double& period) {

  ros::Rate r(1.0 / period);
  while (ros::ok()) {
    if (!if_initial) {
      ROS_WARN("feedback failure caused by initialization failure");
      continue;
    }
    FeedbackReq();
    // cur_time = ros::Time::now().toSec();

    uint data_num;
    data_num = VCI_GetReceiveNum(device_type, device_index, can_index);
    if (0 == data_num) {
      ROS_WARN("no data in buffer of motor feedback!!");
      continue;
    } else if (-1 == data_num) {
      ROS_WARN("get data num of motor feedback failure!");
      continue;
    }

    PVCI_CAN_OBJ rec_obj = new VCI_CAN_OBJ[data_num];
    uint rec_num = VCI_Receive(device_type, device_index, can_index, rec_obj,
                               data_num, 10);
    if (-1 == rec_num) {
      delete[] rec_obj;
      ROS_WARN("CAN reading of motor feedback failure!");
      continue;
    }

    sensor_msgs::JointState state;
    state.header.frame_id = "motor";
    state.header.stamp = ros::Time::now();
    state.name.resize(8);
    state.name = {"front_left_walking",  "front_right_walking",
                  "rear_left_walking",   "rear_right_walking",
                  "front_left_steering", "front_right_steering",
                  "rear_left_steering",  "rear_right_steering"};
    state.position.resize(8);
    state.velocity.resize(8);
    state.velocity = {10000.0, 10000.0, 10000.0, 10000.0,
                      10000.0, 10000.0, 10000.0, 10000.0};
    for (size_t i = 0; i < data_num; i++) {
      if (LEFT_MOTOR == rec_obj[i].Data[2]) {
        if (POSITION_FD == rec_obj[i].Data[1]) {
          int index = 2 * (rec_obj[i].ID - REC_BASE_ID - 1);
          state.position[index] = FourByteHex2Int(&rec_obj[i].Data[4]);
          state.name[index] = state.name[index] + "_1";
        }
        if (VELOCITY_FD == rec_obj[i].Data[1]) {
          int index = 2 * (rec_obj[i].ID - REC_BASE_ID - 1);
          state.velocity[index] = FourByteHex2Int(&rec_obj[i].Data[4]);
        }
      }
      if (RIGHT_MOTOR == rec_obj[i].Data[2]) {
        if (POSITION_FD == rec_obj[i].Data[1]) {
          int index = 2 * (rec_obj[i].ID - REC_BASE_ID) - 1;
          state.position[index] = FourByteHex2Int(&rec_obj[i].Data[4]);
          state.name[index] = state.name[index] + "_1";
        }
        if (VELOCITY_FD == rec_obj[i].Data[1]) {
          int index = 2 * (rec_obj[i].ID - REC_BASE_ID) - 1;
          state.velocity[index] = FourByteHex2Int(&rec_obj[i].Data[4]);
        }
      }
    }

    bool if_pub = true;
    for (size_t i = 0; i < state.velocity.size(); i++) {
      if (10000.0 == state.velocity[i]) {
        delete[] rec_obj;
        if_pub = false;
        ROS_WARN("no enough velocity feedback!!");
        break;
      }
      if (state.name[i].find("_1") > state.name[i].size()) {
        delete[] rec_obj;
        if_pub = false;
        ROS_WARN("no enough position feedback!!");
        break;
      }
    }

    if (if_pub) {
      std::vector<float> raw_state;
      raw_state.resize(8);
      raw_state = {(float)state.velocity[0], (float)state.velocity[1],
                   (float)state.velocity[2], (float)state.velocity[3],
                   (float)state.position[4], (float)state.position[5],
                   (float)state.position[6], (float)state.position[7]};
      //  this scheme might be incorrect
      
       
      // double dt = cur_time - pre_time;
      // SendTf(raw_state, dt);
      
      pre_time = cur_time;
      delete[] rec_obj;
    }
    r.sleep();
  }
}

void Tele::SendTf(const std::vector<float>& raw_state, const double& dt) {

  tf::TransformListener state_listen;
  tf::StampedTransform car2base;
  try {
    state_listen.waitForTransform("map", "base_link", ros::Time(0), ros::Duration(2.0));
    state_listen.lookupTransform("map", "base_link", ros::Time(0), car2base);
  } catch (tf::TransformException ex) {
    ROS_ERROR("Get error in car to base transformation -> %s", ex.what());
  }
  veh_pose.x = car2base.getOrigin().x();
  veh_pose.y = car2base.getOrigin().y();
  veh_pose.z = car2base.getOrigin().z();

  tf::Matrix3x3 mat(car2base.getRotation());
  mat.getEulerYPR(veh_pose.yaw, veh_pose.pitch, veh_pose.roll);

  double hypotenuse = sqrt(pow(wheel_dis_len, 2) + pow(wheel_dis_wid, 2));
  double r = 0.5 * hypotenuse;
  double v = raw_state[0] * M_PI * 0.15 / 60 / encoder_w;  // velocity per second
  double theta_d = v / r;

  int p_e = 100;  // permitted error
  if (abs(home[0] - raw_state[4]) < p_e && abs(home[1] - raw_state[5]) < p_e &&
      abs(home[2] - raw_state[6]) < p_e && abs(home[3] - raw_state[7]) < p_e) {
    veh_pose.x += v * cos(veh_pose.yaw) * dt;
    veh_pose.y += v * sin(veh_pose.yaw) * dt;
   } else {
     veh_pose.yaw += theta_d * dt;
   }


   tf::Transform trans_tmp;
   tf::TransformBroadcaster state_broad;
   trans_tmp.setOrigin(tf::Vector3(veh_pose.x, veh_pose.y, veh_pose.z));
   tf::Quaternion tf_quat;
   tf_quat.setRPY(veh_pose.roll, veh_pose.pitch, veh_pose.yaw);
   trans_tmp.setRotation(tf_quat);

   car2base = tf::StampedTransform(trans_tmp, ros::Time::now(), "odom", "base_foot_print");
}

void Tele::Loop() {
 
  stop_sub = nh.subscribe("stop", 100, &Tele::TeleStop, this);
  tele_sub = nh.subscribe("cmd_vel", 100, &Tele::TeleControl, this);

  double period = 0.1;
  fb_thread = new boost::thread(boost::bind(&Tele::TeleFeedback, this, period));
}

}  // namespace mobile


int main(int argc, char** argv) {
  ros::init(argc, argv, "teleop_node");

  mobile_base::Tele tele;
  // ros::Subscriber stop_sub = n.subscribe("stop", 100, &mobile::MotorReader::StopCallback, &motor);

  tele.Loop();
  ros::spin();
  return 0;
}
