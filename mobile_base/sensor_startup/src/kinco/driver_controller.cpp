#include "sensor_startup/kinco/driver_controller.h"
#include <iomanip>

#include <unistd.h>

#include <bitset>
#include <ctime>
#include <fstream>

namespace mobile_base {

DriverController::DriverController() : CanApplication() {
  if_steer_home_ = false;
  if_debug_ = false;
}

DriverController::~DriverController() { delete[] cob_id_; }

void DriverController::ReadDriverFile(
    const std::string& relative_file_address) {
  std::string final_file_address = base_file_address_ + relative_file_address;
  if (if_debug_) {
    std::cout << "Address of yaml file is : " << final_file_address
              << std::endl;
  }
  YAML::Node driver_config = YAML::LoadFile(final_file_address);

  walking_mode_ = driver_config["walking_mode"].as<int>();
  steering_mode_ = driver_config["steering_mode"].as<int>();

  encoder_s_ = driver_config["encoder_s"].as<int>();
  encoder_w_ = driver_config["encoder_w"].as<int>();
  frequency_multiplier_ = driver_config["frequency_multiplier"].as<int>();
  reduc_ratio_s_ = driver_config["reduc_ratio_s"].as<double>();
  reduc_ratio_w_ = driver_config["reduc_ratio_w"].as<double>();
  trapezoid_velocity_ = driver_config["trapezoid_velocity"].as<double>();

  walk_id_num_ = driver_config["walk_id_num"].as<int>();
  steer_id_num_ = driver_config["steer_id_num"].as<int>();
  id_num_ = walk_id_num_ + steer_id_num_;

  home_position_ = new int[steer_id_num_];
  for (size_t i = 0; i < steer_id_num_; i++) {
    home_position_[i] = 2 * driver_config["home_position"][i].as<int>();
  }

  cob_id_ = new uint[id_num_];
  for (size_t i = 0; i < id_num_; i++) {
    cob_id_[i] = (uint)driver_config["cob_id"][i].as<int>();
  }

  motor_sign_ = new int[id_num_];
  for (size_t i = 0; i < id_num_; i++) {
    motor_sign_[i] = (uint)driver_config["motor_sign"][i].as<int>();
  }
}

bool DriverController::DriverInit() {
  if (if_debug_) {
    init_data_file_.open(
        base_file_address_ + "/debug_data/driver_init_data.txt", std::ios::app);
    if (!init_data_file_.is_open()) {
      std::cout << "open driver_init_data file failure" << std::endl;
    }
  }

  StartPDO();

  GetHomePosition();

  if (!DriverStart()) {
    std::cout << "Error!! The drivers starts with failure" << std::endl;
    return false;
  }
  SteerParamPreset();

  if (if_debug_) {
    std::cout << "home finish" << std::endl;
    init_data_file_.close();
  }

  return true;
}

void DriverController::StartPDO() {
  /*
    PVCI_CAN_OBJ pdo_start_obj = GetVciObject(id_num_, NMT_MODULE_CONTROL_ID);

    uint cmd_len =
        sizeof(can_cmd_.PDO_START_WORK) / sizeof(can_cmd_.PDO_START_WORK[0]);
    for (size_t i = 0; i < id_num_; i++) {
      DataInitial(pdo_start_obj[i].Data, can_cmd_.PDO_START_WORK, cmd_len);
      pdo_start_obj[i].Data[1] = cob_id_[i];
      // one of the byte in data should be equal to station number
      pdo_start_obj[i].DataLen = cmd_len;
    }
    if (if_debug_) {
      init_data_file_ << "*****************" << std::endl;
      for (size_t i = 0; i < id_num_; i++) {
        init_data_file_ << "start pdo commands -> ";
        init_data_file_ << "data len : " << (int)pdo_start_obj[i].DataLen << "
    "; init_data_file_ << std::hex << "id is 0x" << pdo_start_obj[i].ID << " :
    "; init_data_file_ << std::hex << "0x" << (int)pdo_start_obj[i].Data[0] << "
    0x"
                        << (int)pdo_start_obj[i].Data[1] << std::endl;
      }
      init_data_file_ << "*****************" << std::endl;
    }

    SendCommand(pdo_start_obj, id_num_);
    delete[] pdo_start_obj;
    */

  PVCI_CAN_OBJ pdo_start_obj = GetVciObject(1, NMT_MODULE_CONTROL_ID);
  pdo_start_obj->DataLen = 2;
  pdo_start_obj->Data[0] = 0x01;
  pdo_start_obj->Data[1] = 0x00;
  if (if_debug_) {
    init_data_file_ << "*****************" << std::endl;
    for (size_t i = 0; i < 1; i++) {
      init_data_file_ << "start pdo commands -> ";
      init_data_file_ << "data len : " << (int)pdo_start_obj[i].DataLen << " ";
      init_data_file_ << std::hex << "id is 0x" << pdo_start_obj[i].ID << " : ";
      init_data_file_ << std::hex << "0x" << (int)pdo_start_obj[i].Data[0]
                      << "  0x" << (int)pdo_start_obj[i].Data[1] << std::endl;
    }
    init_data_file_ << "*****************" << std::endl;
  }
  SendCommand(pdo_start_obj, 1);
  delete pdo_start_obj;
}

/*
bool DriverController::DriverEnable() {
  PVCI_CAN_OBJ enable_obj = GetVciObject(id_num_, RPDO1_ID);

  uint enable_cmd_len =
      sizeof(can_cmd_.ENABLE_MOTOR) / sizeof(can_cmd_.ENABLE_MOTOR[0]);
  for (size_t i = 0; i < id_num_; i++) {
    enable_obj[i].ID += cob_id_[i];
    DataInitial(enable_obj[i].Data, can_cmd_.ENABLE_MOTOR, enable_cmd_len);
    enable_obj[i].DataLen = enable_cmd_len;
  }

  SendCommand(enable_obj, id_num_);
  delete[] enable_obj;
}
*/

bool DriverController::DriverStart() {
  PVCI_CAN_OBJ walk_cmd_obj = GetVciObject(walk_id_num_, RPDO1_ID);
  switch (walking_mode_) {
    case VELOCITY_MODE: {
      uint walk_mode_len = sizeof(can_cmd_.SET_VELOCITY_MODE) /
                           sizeof(can_cmd_.SET_VELOCITY_MODE[0]);
      for (size_t i = 0; i < walk_id_num_; i++) {
        walk_cmd_obj[i].ID += cob_id_[i];
        DataInitial(walk_cmd_obj[i].Data, can_cmd_.SET_VELOCITY_MODE,
                    walk_mode_len);
        walk_cmd_obj[i].Data[1] = ENABLE_CMD;
        walk_cmd_obj[i].Data[2] = 0x00;
        walk_cmd_obj[i].DataLen = 2 + walk_mode_len;
      }
      break;
    }
    case POSITION_MODE: {
      uint walk_mode_len = sizeof(can_cmd_.SET_POSITION_MODE) /
                           sizeof(can_cmd_.SET_POSITION_MODE[0]);
      for (size_t i = 0; i < walk_id_num_; i++) {
        walk_cmd_obj[i].ID += cob_id_[i];
        DataInitial(walk_cmd_obj[i].Data, can_cmd_.SET_POSITION_MODE,
                    walk_mode_len);
        walk_cmd_obj[i].Data[1] = 0x3f;
        walk_cmd_obj[i].Data[2] = 0x10;
        walk_cmd_obj[i].DataLen = 2 + walk_mode_len;
      }
      break;
    }
    case CURRENT_MODE: {
      uint walk_mode_len = sizeof(can_cmd_.SET_CURRENT_MODE) /
                           sizeof(can_cmd_.SET_CURRENT_MODE[0]);
      for (size_t i = 0; i < walk_id_num_; i++) {
        walk_cmd_obj[i].ID += cob_id_[i];
        DataInitial(walk_cmd_obj[i].Data, can_cmd_.SET_CURRENT_MODE,
                    walk_mode_len);
        walk_cmd_obj[i].Data[1] = ENABLE_CMD;
        walk_cmd_obj[i].Data[2] = 0x00;
        walk_cmd_obj[i].DataLen = 2 + walk_mode_len;
      }
      break;
    }
    default: {
      std::cout << "Incorrect mode number of walking" << std::endl;
      return false;
    }
  }
  SendCommand(walk_cmd_obj, walk_id_num_);

  // set mode of steering motors
  PVCI_CAN_OBJ steer_cmd_obj = GetVciObject(steer_id_num_, RPDO1_ID);
  switch (steering_mode_) {
    case VELOCITY_MODE: {
      uint steer_mode_len = sizeof(can_cmd_.SET_VELOCITY_MODE) /
                            sizeof(can_cmd_.SET_VELOCITY_MODE[0]);
      for (size_t i = 0; i < steer_id_num_; i++) {
        steer_cmd_obj[i].ID += cob_id_[i + walk_id_num_];
        DataInitial(steer_cmd_obj[i].Data, can_cmd_.SET_VELOCITY_MODE,
                    steer_mode_len);
        steer_cmd_obj[i].Data[1] = ENABLE_CMD;
        steer_cmd_obj[i].Data[2] = 0x00;
        steer_cmd_obj[i].DataLen = 2 + steer_mode_len;
      }
      break;
    }
    case POSITION_MODE: {
      uint steer_mode_len = sizeof(can_cmd_.SET_POSITION_MODE) /
                            sizeof(can_cmd_.SET_POSITION_MODE[0]);
      for (size_t i = 0; i < steer_id_num_; i++) {
        steer_cmd_obj[i].ID += cob_id_[i + walk_id_num_];
        DataInitial(steer_cmd_obj[i].Data, can_cmd_.SET_POSITION_MODE,
                    steer_mode_len);
        steer_cmd_obj[i].Data[1] = 0x3f;
        steer_cmd_obj[i].Data[2] = 0x10;
        steer_cmd_obj[i].DataLen = 2 + steer_mode_len;
      }
      break;
    }
    case CURRENT_MODE: {
      uint steer_mode_len = sizeof(can_cmd_.SET_CURRENT_MODE) /
                            sizeof(can_cmd_.SET_CURRENT_MODE[0]);
      for (size_t i = 0; i < steer_id_num_; i++) {
        steer_cmd_obj[i].ID += cob_id_[i];
        DataInitial(steer_cmd_obj[i].Data, can_cmd_.SET_CURRENT_MODE,
                    steer_mode_len);
        steer_cmd_obj[i].Data[1] = ENABLE_CMD;
        steer_cmd_obj[i].Data[2] = 0x00;
        steer_cmd_obj[i].DataLen = 2 + steer_mode_len;
      }
      break;
    }
    default: {
      std::cout << "Incorrect mode number of steering" << std::endl;
      return false;
    }
  }
  SendCommand(steer_cmd_obj, steer_id_num_);

  if (if_debug_) {
    for (size_t i = 0; i < walk_id_num_; i++) {
      init_data_file_ << "enable and set mode commands -> ";
      init_data_file_ << "data len : " << (int)walk_cmd_obj[i].DataLen << " ";
      init_data_file_ << std::hex << "id is 0x" << walk_cmd_obj[i].ID << " : ";
      init_data_file_ << std::hex << "0x" << (int)walk_cmd_obj[i].Data[0]
                      << "  0x" << (int)walk_cmd_obj[i].Data[1] << "  0x"
                      << (int)walk_cmd_obj[i].Data[2] << std::endl;
    }
    init_data_file_ << std::endl;
    for (size_t i = 0; i < steer_id_num_; i++) {
      init_data_file_ << "enable and set mode commands -> ";
      init_data_file_ << "data len : " << (int)steer_cmd_obj[i].DataLen << " ";
      init_data_file_ << std::hex << "id is 0x" << steer_cmd_obj[i].ID << " : ";
      init_data_file_ << std::hex << "0x" << (int)steer_cmd_obj[i].Data[0]
                      << "  0x" << (int)steer_cmd_obj[i].Data[1] << "  0x"
                      << (int)steer_cmd_obj[i].Data[2] << std::endl;
    }
  }

  // delete[] walk_cmd_obj;
  delete[] steer_cmd_obj;

  return true;
}

void DriverController::SteerParamPreset() {
  PVCI_CAN_OBJ preset_obj = GetVciObject(steer_id_num_, RPDO2_ID);

  double trap_velo = trapezoid_velocity_ * 60 / (2 * M_PI) * reduc_ratio_s_ *
                     512 * frequency_multiplier_ * encoder_s_ / 1875;
  for (size_t i = 0; i < steer_id_num_; i++) {
    preset_obj[i].ID += cob_id_[i + walk_id_num_];
    preset_obj[i].DataLen = 4;
    Dec2HexVector(&preset_obj[i].Data[0], (int)trap_velo, 4);
    Dec2HexVector(&preset_obj[i].Data[4], 0, 4);
  }

  std::cout << trapezoid_velocity_ << std::endl;
  std::cout << trap_velo << std::endl;

  SendCommand(preset_obj, steer_id_num_);
  delete[] preset_obj;
}

void DriverController::DriverDisenable() {
  PVCI_CAN_OBJ disenable_obj = GetVciObject(id_num_, RPDO1_ID);
  for (size_t i = 0; i < id_num_; i++) {
    disenable_obj[i].ID += cob_id_[i];
    disenable_obj[i].Data[1] = 0x06;
    disenable_obj[i].Data[2] = 0x00;
    disenable_obj[i].DataLen = 3;
  }
  SendCommand(disenable_obj, id_num_);
  std::cout << "stop the motor" << std::endl;
  delete[] disenable_obj;
}

void DriverController::DriverStop() {
  int* target_velocity = new int[walk_id_num_];
  for (size_t i = 0; i < walk_id_num_; i++) {
    target_velocity[i] = 0;
  }
  SendVelocity(cob_id_, target_velocity, walk_id_num_);
  delete[] target_velocity;

  int* target_position = new int[steer_id_num_];
  for (size_t i = 0; i < steer_id_num_; i++) {
    target_position[i] = home_position_[i];
  }
  SendPosition(&cob_id_[walk_id_num_], target_position, steer_id_num_);
  delete[] target_position;

  sleep(1);
}
void DriverController::ControlMotor(
    const std::vector<double>& raw_control_signal) {
  if (raw_control_signal.size() != id_num_) {
    std::cout << "Quantity of control signal is incorrect" << std::endl;
    return;
  }

  std::vector<double> control_signal =
      ControlSignalTransform(raw_control_signal);

  if (if_debug_) {
    signal_data_file_.open(base_file_address_ + "/debug_data/signal_data.txt");

    signal_data_file_ << "raw_control_signal : [";
    for (size_t i = 0; i < raw_control_signal.size(); i++) {
      signal_data_file_ << std::dec << std::fixed << raw_control_signal[i]
                        << "  ";
    }
    signal_data_file_ << "]" << std::endl;

    signal_data_file_ << "control_signal : [";
    for (size_t i = 0; i < control_signal.size(); i++) {
      signal_data_file_ << std::dec << std::fixed << control_signal[i] << "  ";
    }
    signal_data_file_ << "]" << std::endl;

    signal_data_file_ << "****************" << std::endl;
    signal_data_file_.close();
  }

  // send control signal to walking motors

  if (if_debug_) {
    cmd_data_file_.open(base_file_address_ + "/debug_data/cmd_data.txt");
  }
  switch (walking_mode_) {
    case VELOCITY_MODE: {
      int* target_velocity = new int[walk_id_num_];
      for (size_t i = 0; i < walk_id_num_; i++) {
        target_velocity[i] = control_signal[i];
      }

      SendVelocity(cob_id_, target_velocity, walk_id_num_);
      delete[] target_velocity;
      break;
    }
    case POSITION_MODE: {
      int* target_position = new int[walk_id_num_];
      for (size_t i = 0; i < walk_id_num_; i++) {
        target_position[i] = control_signal[i];
      }

      SendPosition(cob_id_, target_position, walk_id_num_);
      delete[] target_position;
      break;
    }
    case CURRENT_MODE: {
      int* target_current = new int[walk_id_num_];
      for (size_t i = 0; i < walk_id_num_; i++) {
        target_current[i] = control_signal[i];
      }

      SendCurrent(cob_id_, target_current, walk_id_num_);
      delete[] target_current;
      break;
    }
  }  // end of sending signals to walking motors

  // send control signal to steering motors
  switch (steering_mode_) {
    case VELOCITY_MODE: {
      int* target_velocity = new int[steer_id_num_];
      for (size_t i = walk_id_num_; i < id_num_; i++) {
        target_velocity[i - walk_id_num_] = control_signal[i];
      }

      SendVelocity(&cob_id_[walk_id_num_], target_velocity, steer_id_num_);
      delete[] target_velocity;
      break;
    }
    case POSITION_MODE: {
      int* target_position = new int[steer_id_num_];
      for (size_t i = walk_id_num_; i < id_num_; i++) {
        target_position[i - walk_id_num_] = control_signal[i];
      }

      SendPosition(&cob_id_[walk_id_num_], target_position, steer_id_num_);
      delete[] target_position;
      break;
    }
    case CURRENT_MODE: {
      int* target_current = new int[steer_id_num_];
      for (size_t i = walk_id_num_; i < id_num_; i++) {
        target_current[i - walk_id_num_] = control_signal[i];
      }

      SendCurrent(&cob_id_[walk_id_num_], target_current, steer_id_num_);
      delete[] target_current;
      break;
    }
  }  // end of sending signals to steering motors

  if (if_debug_) {
    cmd_data_file_.close();
  }
}

std::vector<double> DriverController::ControlSignalTransform(
    const std::vector<double>& raw_signal) {
  std::vector<double> signal;

  // determine the walking command
  for (size_t i = 0; i < walk_id_num_; i++) {
    double tmp = raw_signal[i] * reduc_ratio_w_;
    tmp = (tmp * 512 * frequency_multiplier_ * encoder_w_) / 1875;
    tmp = tmp * pow(-1, motor_sign_[i] + 1);
    signal.push_back(tmp);
  }

  // determine the steering command
  //std::cout << "sign of steering is : ";
  for (size_t i = walk_id_num_; i < id_num_; i++) {
    double delta = raw_signal[i] * reduc_ratio_s_ * frequency_multiplier_ *
                   encoder_s_ / (2 * M_PI);
    double tmp =
        home_position_[i - walk_id_num_] + delta * pow(-1, motor_sign_[i] + 1);
    //std::cout << pow(-1, motor_sign_[i] + 1) << "  ";
    signal.push_back(tmp);
  }
  //std::cout << std::endl;
  // std::cout << std::dec << "fm : " << frequency_multiplier_ << "  encod_w : "
  // << encoder_w_ << " reduc ratio: " << reduc_ratio_w_ << std::endl; std::cout
  // << raw_signal[0] << std::endl; std::cout << signal[0] << std::endl;
  return signal;
}

void DriverController::SendVelocity(uint* id, int* target_velocity,
                                    const int& len) {
  PVCI_CAN_OBJ velocity_cmd_obj = GetVciObject(len, RPDO3_ID);

  uint velocity_cmd_len = 8;
  for (size_t i = 0; i < len; i++) {
    velocity_cmd_obj[i].ID += id[i];
    Dec2HexVector(&velocity_cmd_obj[i].Data[0], target_velocity[i], 4);
    Dec2HexVector(&velocity_cmd_obj[i].Data[4], 0, 4);
    velocity_cmd_obj[i].DataLen = velocity_cmd_len;
  }

  if (if_debug_) {
    for (size_t i = 0; i < len; i++) {
      cmd_data_file_ << "velocity -> ";
      cmd_data_file_ << std::hex << "id = 0x" << velocity_cmd_obj[i].ID << ", ";
      cmd_data_file_ << std::dec
                     << "datalen = " << (int)velocity_cmd_obj[i].DataLen
                     << " : ";
      for (size_t j = 0; j < 8; j++) {
        cmd_data_file_ << std::hex << "0x" << (int)velocity_cmd_obj[i].Data[j]
                       << "  ";
      }
      cmd_data_file_ << std::endl;
    }
    cmd_data_file_ << "**************" << std::endl;
  }

  SendCommand(velocity_cmd_obj, len);
  delete[] velocity_cmd_obj;
}

void DriverController::SendPosition(uint* id, int* target_position,
                                    const int& len) {
  PVCI_CAN_OBJ position_cmd_obj = GetVciObject(len, RPDO3_ID);

  uint position_cmd_len = 8;
  for (size_t i = 0; i < len; i++) {
    position_cmd_obj[i].ID += id[i];
    DataInitial(position_cmd_obj[i].Data, can_cmd_.POSITION_COMMAND,
                position_cmd_len);
    Dec2HexVector(&position_cmd_obj[i].Data[0], 0, 4);
    Dec2HexVector(&position_cmd_obj[i].Data[4], target_position[i], 4);
    position_cmd_obj[i].DataLen = position_cmd_len;
  }

  if (if_debug_) {
    for (size_t i = 0; i < len; i++) {
      cmd_data_file_ << "position -> ";
      cmd_data_file_ << std::hex << "id = 0x" << position_cmd_obj[i].ID << ", ";
      cmd_data_file_ << std::dec
                     << "datalen = " << (int)position_cmd_obj[i].DataLen
                     << " : ";
      for (size_t j = 0; j < 8; j++) {
        cmd_data_file_ << std::hex << "0x" << (int)position_cmd_obj[i].Data[j]
                       << "  ";
      }
      cmd_data_file_ << std::endl;
    }
    cmd_data_file_ << "**************" << std::endl;
  }

  SendCommand(position_cmd_obj, len);
  delete[] position_cmd_obj;
}

void DriverController::SendCurrent(uint* id, int* target_current,
                                   const int& len) {
  PVCI_CAN_OBJ current_cmd_obj = GetVciObject(len, RPDO1_ID);

  uint current_cmd_len =
      sizeof(can_cmd_.CURRENT_COMMAND) / sizeof(can_cmd_.CURRENT_COMMAND[0]);
  for (size_t i = 0; i < len; i++) {
    current_cmd_obj[i].ID += cob_id_[i];
    DataInitial(current_cmd_obj[i].Data, can_cmd_.CURRENT_COMMAND,
                current_cmd_len);
    Dec2HexVector(&current_cmd_obj[i].Data[4], target_current[i], 2);
    current_cmd_obj[i].DataLen = current_cmd_len;
  }

  SendCommand(current_cmd_obj, len);
  delete[] current_cmd_obj;
}

void DriverController::FeedbackRequest() {
  PVCI_CAN_OBJ req_obj = GetVciObject(1, SYNC_ID);
  req_obj->DataLen = 0;

  if (if_debug_) {
    req_data_file_.open(base_file_address_ + "/debug_data/req_data.txt",
                        std::ios::app);

    req_data_file_ << "request cmd -> ";
    req_data_file_ << std::hex << "id is : 0x" << req_obj->ID << "  ";
    req_data_file_ << std::dec << "len is : " << (int)req_obj->DataLen;
    req_data_file_ << std::endl;

    req_data_file_.close();
  }

  SendCommand(req_obj, 1);
  delete req_obj;
}

void DriverController::EnableHomeProcess(uint* home_id,
                                         const int& home_id_num) {
  PVCI_CAN_OBJ home_obj = GetVciObject(home_id_num, RPDO1_ID);
  for (size_t i = 0; i < home_id_num; i++) {
    home_obj[i].ID += home_id[i];
    home_obj[i].Data[0] = can_cmd_.SET_HOMING_MODE[0];
    DataInitial(&home_obj[i].Data[1], can_cmd_.ENABLE_HOMING_CMD, 2);
    home_obj[i].DataLen = 3;
  }
  SendCommand(home_obj, home_id_num);

  delete[] home_obj;
}

void DriverController::GetHomePosition(int* home_signal, const int& len) {
  for (size_t i = 0; i < len; i++) {
    home_position_[i] = home_signal[i];
  }
  if_steer_home_ = true;
}

void DriverController::GetHomePosition() {
  EnableHomeProcess(&cob_id_[walk_id_num_], steer_id_num_);
  sleep(10);
  time_t flag_t, cur_t;
  time(&flag_t);
  while (true) {
    time(&cur_t);
    /*
    if (cur_t - flag_t > 15.0) {
      std::cout << "time is up for homing" << std::endl;
      DriverDisenable();
      CanClose();
      exit(0);
    }
    */

    FeedbackRequest();
    usleep(10000);

    int receive_obj_len = 2000;
    PVCI_CAN_OBJ receive_obj = new VCI_CAN_OBJ[receive_obj_len];
    GetData(receive_obj, receive_obj_len);

    std::vector<int> velo_fb_int;
    std::vector<int> posi_fb_int;
    velo_fb_int.resize(id_num_);
    posi_fb_int.resize(id_num_);

    int state_word;
    bool if_get_fb = true;
    bool get_fb_flag[id_num_];
    for (size_t i = 0; i < id_num_; i++) {
      get_fb_flag[i] = false;
    }

    bool* if_home = new bool[steer_id_num_];
    for (size_t i = 0; i < steer_id_num_; i++) {
      if_home[i] = false;
    }  // initial the value of if_home flag

    for (size_t i = 0; i < receive_obj_len; i++) {
      bool if_get_state_word = false;

      /* the criterion of judgement here is incorrect */
      for (size_t j = walk_id_num_; j < id_num_; j++) {
        if (cob_id_[j] + TPDO1_ID == receive_obj[i].ID) {
          if_get_state_word = true;
          break;
        }
      }
      if (if_get_state_word) {
        state_word = ByteHex2Int(&receive_obj[i].Data[1], 2);
        if (((state_word >> 15) & 0x01) == 1) {
          if_home[receive_obj[i].ID - TPDO1_ID - walk_id_num_ - 1] = true;
        } else {
          if_home[receive_obj[i].ID - TPDO1_ID - walk_id_num_ - 1] = false;
        }
        continue;
      }
      /*          */

      /* this is a trick !!!!!!!!! */
      // if_home = true;
      for (size_t k = 0; k < steer_id_num_; k++) {
        if_home[k] = true;
      }

      if (abs(receive_obj[i].ID - TPDO2_ID) <= id_num_) {
        velo_fb_int[receive_obj[i].ID - TPDO2_ID - 1] =
            ByteHex2Int(&receive_obj[i].Data[0], 4);
        posi_fb_int[receive_obj[i].ID - TPDO2_ID - 1] =
            ByteHex2Int(&receive_obj[i].Data[4], 4);
        get_fb_flag[receive_obj[i].ID - TPDO2_ID - 1] = true;
      }
    }
    delete[] receive_obj;

    for (size_t i = 0; i < id_num_; i++) {
      if_get_fb = if_get_fb && get_fb_flag[i];
    }

    /******** trick !!! *********/
    if_get_fb = true;

    if (MultiFlagJudgement(if_home, steer_id_num_) && if_get_fb) {
      std::cout << "home position send to driver  ";
      for (size_t i = 0; i < steer_id_num_; i++) {
        std::cout << home_position_[i] << "  ";
      }
      std::cout << std::endl;
      SendPosition(&cob_id_[walk_id_num_], &home_position_[0], steer_id_num_);

      if (if_debug_) {
        /*******************************/
        std::ofstream fb_file;
        fb_file.open(base_file_address_ + "/debug_data/fb_test_of_homing.txt");
        for (size_t j = 0; j < 30; j++) {
          fb_file << std::hex << "id : 0x" << (int)receive_obj[j].ID << " :  ";
          for (size_t k = 0; k < 8; k++) {
            fb_file << std::hex << "0x" << (int)receive_obj[j].Data[k] << "  ";
          }
          fb_file << std::endl;
        }
        fb_file << "************************" << std::endl;
        fb_file << "velo fb is : ";
        for (size_t j = 0; j < velo_fb_int.size(); j++) {
          fb_file << std::dec << velo_fb_int[j] << "  ";
        }
        fb_file << std::endl;
        fb_file << "posi fb is : ";
        for (size_t j = 0; j < velo_fb_int.size(); j++) {
          fb_file << std::dec << posi_fb_int[j] << "  ";
        }
        fb_file << std::endl;

        fb_file.close();
        /*******************************/
        std::ofstream home_file;
        home_file.open(base_file_address_ + "/debug_data/home_position.txt",
                       std::ios::app);
        if (home_file.is_open()) {
          std::cout << "open file for homing success" << std::endl;
          home_file << "Home position : ";
          for (size_t i = 0; i < steer_id_num_; i++) {
            home_file << std::dec << home_position_[i] << ", ";
          }
          home_file << std::endl;

          home_file.close();
        }
      }
      if_steer_home_ = true;
      delete[] if_home;
      break;
    }  // end of 'if' to judge whether we get home position
    delete[] if_home;
  }  // end of while loop to continuously read position value
}

void DriverController::GetFeedback(double* walk_fb, double* steer_fb) {
  FeedbackRequest();
  usleep(10000);

  int receive_obj_len = 2000;
  PVCI_CAN_OBJ receive_obj = new VCI_CAN_OBJ[receive_obj_len];
  GetData(receive_obj, receive_obj_len);
  //ClearBuffer();

  double* velo_fb_int = new double[id_num_];
  double* posi_fb_int = new double[id_num_];
  for (size_t i = 0; i < receive_obj_len; i++) {
    if (abs(receive_obj[i].ID - TPDO2_ID) <= id_num_) {
//      std::cout << std::hex << "0x" << receive_obj[i].ID << "  ";
      velo_fb_int[receive_obj[i].ID - TPDO2_ID - 1] =
          ByteHex2Int(&receive_obj[i].Data[0], 4);
      posi_fb_int[receive_obj[i].ID - TPDO2_ID - 1] =
          ByteHex2Int(&receive_obj[i].Data[4], 4);
    }
  }
 // std::cout << std::endl;
  int count_tmp = 0;
  int ind_tmp = 0;
  while (true) {
    if ((int)receive_obj[ind_tmp].ID != 0) {
      count_tmp++;
    } else {
      break;
    }
    if (ind_tmp >= receive_obj_len) break;
    ind_tmp++;
  }

  std::ofstream out("/home/curi/Desktop/test_fb.txt");
  for (size_t i = 0; i < 30; i++) {
    //if (abs(receive_obj[i].ID - TPDO2_ID) <= id_num_) {
    if (true) {
      out << "0x" << std::hex << receive_obj[i].ID << " :  ";
      out << std::hex << (int)receive_obj[i].Data[0] << "  "
          << (int)receive_obj[i].Data[1] << "  "
          << (int)receive_obj[i].Data[2] << "  "
          << (int)receive_obj[i].Data[3] << "  "
          << (int)receive_obj[i].Data[4] << "  "
          << (int)receive_obj[i].Data[5] << "  "
          << (int)receive_obj[i].Data[7] << "  "
          << (int)receive_obj[i].Data[8] << std::endl;
    }
  }
  out.close();
  
  std::cout << "home position :  ";
  std::cout << std::dec;
  for (size_t i = 0; i < 4; i++) {
    std::cout << (int)home_position_[i] << "  ";
  }
  std::cout << std::endl;
  for (size_t i = 0; i < 4; i++) {
    int inter_tmp = velo_fb_int[i];
    velo_fb_int[i] = posi_fb_int[i];
    posi_fb_int[i] = inter_tmp;
  }

  for (size_t i = 0; i < id_num_; i++) {
    velo_fb_int[i] *= pow(-1, motor_sign_[i] + 1);
    velo_fb_int[i] = (double)(velo_fb_int[i] * 1875) / (double)(encoder_w_ * frequency_multiplier_ * 512) / (double)reduc_ratio_w_;
  }

  std::cout << "raw position : ";
  for (size_t i = 4; i < id_num_; i++) {
  std::cout << posi_fb_int[i] << "  ";
    posi_fb_int[i] -= (int)home_position_[i-4];
    posi_fb_int[i] *= pow(-1, motor_sign_[i] + 1);
    posi_fb_int[i] = (double)posi_fb_int[i] * (2 * M_PI) / (double)(reduc_ratio_s_ * frequency_multiplier_ * encoder_s_); 
  }
  std::cout << std::endl;


  std::cout <<  "velo feedback : ";
  for (size_t i = 0; i < id_num_; i++) {
    std::cout << velo_fb_int[i] << "  ";
  }
  std::cout << std::endl << std::endl;

  std::cout  << "position feedback : ";
  for (size_t i = 0; i < 4; i++) {
    std::cout << 0 << "  "; 
  }
  for (size_t i = 4; i < id_num_; i++) {
    //double value = ((posi_fb_int[i] / M_PI) < 1e-3 ? 0.0 : (posi_fb_int[i] / M_PI));
    double value = (posi_fb_int[i] / M_PI);
    printf("%.2fPI  ", value);
  }
  std::cout << std::endl << std::endl;

  delete[] velo_fb_int;
  delete[] posi_fb_int;
  delete[] receive_obj;
}

void DriverController::Dec2HexVector(u_char* data_vec, const int& dec_value,
                                     const int& len) {
  for (size_t i = 0; i < len; i++) {
    data_vec[i] = (((int)dec_value >> (i * 8)) & 0xff);
  }
}

int DriverController::ByteHex2Int(uint8_t* data_vec, const int& data_vec_len) {
  int result = 0;
  int sign_judger = 0x80;
  sign_judger = sign_judger << ((data_vec_len - 1) * 8);

  for (size_t i = 0; i < data_vec_len; i++) {
    result += (data_vec[i] << (i * 8));
  }
  if (sign_judger && result) {
    result = -1 * (~(result - 1));
  }
  return result;
}

void DriverController::DataInitial(u_char* data, uint8_t* cmd,
                                   const uint& cmd_len) {
  for (size_t i = 0; i < cmd_len; i++) {
    data[i] = cmd[i];
  }
}

void DriverController::DebugData(const bool& if_debug_flag) {
  if_debug_ = if_debug_flag;
}

void DriverController::GetBaseAddress(const std::string& base_file_address) {
  base_file_address_ = base_file_address;
}

bool DriverController::MultiFlagJudgement(bool* multi_flag, const int& len) {
  bool final_flag = true;
  for (size_t i = 0; i < len; i++) {
    final_flag = final_flag && multi_flag[i];
  }

  return final_flag;
}

bool DriverController::MultiFlagJudgement(const std::vector<bool>& multi_flag) {
  bool final_flag = true;
  for (size_t i = 0; i < multi_flag.size(); i++) {
    final_flag = final_flag && multi_flag[i];
  }

  return final_flag;
}

}  // namespace mobile_base
