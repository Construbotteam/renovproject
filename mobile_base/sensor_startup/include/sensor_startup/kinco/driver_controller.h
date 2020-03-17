#ifndef DRIVER_READER_H
#define DRIVER_READER_H

#include <cmath>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

#include "can_application.h"
#include "can_command.h"

typedef unsigned char u_char;

namespace mobile_base {

class DriverController : public CanApplication {
 public:
  DriverController();
  virtual ~DriverController();
  void ReadDriverFile(const std::string& relative_file_address);

  bool DriverInit();
  void StartPDO();
  // bool DriverEnable();
  void SteerParamPreset();
  bool DriverStart();
  void DriverStop();
  void DriverDisenable();

  void ControlMotor(const std::vector<double>& control_signal);
  std::vector<double> ControlSignalTransform(const std::vector<double>& raw_signal);
  void SendVelocity(uint* id, int* target_velocity, const int& len);
  void SendPosition(uint* id, int* target_position, const int& len);
  void SendCurrent(uint* id, int* target_current, const int& len);
  void FeedbackRequest();
  void GetHomePosition(int* home_signal, const int& len);
  void GetHomePosition();
  void EnableHomeProcess(uint* home_id, const int& home_id_num);
  void GetFeedback(double* walk_fb, double* steer_fb);
  void DriverDiagnostic();

  void DataInitial(u_char* data, uint8_t* cmd, const uint& cmd_len);
  void Dec2HexVector(u_char* data_vec, const int& dec_value, const int& len);
  int ByteHex2Int(uint8_t* data_vec, const int& data_vec_len);
  void DebugData(const bool& if_debug_flag);
  void GetBaseAddress(const std::string& base_file_address);
  bool MultiFlagJudgement(bool* multi_flag, const int& len);
  bool MultiFlagJudgement(const std::vector<bool>& multi_flag);

 protected:
  /* PARAMETERS READ FROM YAML FILE */
  int id_num_;
  int walk_id_num_;
  int steer_id_num_;

 private:
  /* PARAMETERS READ FROM YAML FILE */
  int walking_mode_;
  int steering_mode_;
  int encoder_s_;
  int encoder_w_;
  int frequency_multiplier_;
  double reduc_ratio_s_;
  double reduc_ratio_w_;
  double trapezoid_velocity_;
  uint* cob_id_;
  int* motor_sign_;

  /* */
  CanCommand can_cmd_;
  int* home_position_;
  bool if_steer_home_;
  bool if_debug_;
  std::ofstream init_data_file_;
  std::ofstream signal_data_file_;
  std::ofstream cmd_data_file_;
  std::ofstream req_data_file_;
  std::string base_file_address_;

};  // class DriverController

}  // namespace mobile_base

#endif
