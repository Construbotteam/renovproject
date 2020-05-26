#include "sensor_startup/kinco/can_application.h"

using mobile_base::CanApplication;

CanApplication::CanApplication() {}

CanApplication::~CanApplication() {}

void CanApplication::LoadConfig(const std::string& file_address) {
  YAML::Node can_config = YAML::LoadFile(file_address);

  device_type_ = can_config["device_type"].as<int>();
  device_index_ = can_config["device_index"].as<int>();
  can_index_ = can_config["can_index"].as<int>();
  wait_time_ = can_config["wait_time"].as<int>();
  frame_len_ = can_config["frame_len"].as<int>();

}

void CanApplication::ActivateCAN(const std::string& file_address) {
  LoadConfig(file_address);

  int open_state = VCI_OpenDevice(device_type_, device_index_, 0);
  CanDiagnostic("open can device", open_state);

  VCI_INIT_CONFIG can_init_config;
  can_init_config.AccCode = 0x00000000;
  can_init_config.AccMask = 0xFFFFFFFF;
  can_init_config.Filter = 0;
  can_init_config.Mode = 0;
  can_init_config.Timing0 = 0x00;
  can_init_config.Timing1 = 0x1c;

  int init_state =
      VCI_InitCAN(device_type_, device_index_, can_index_, &can_init_config);
  CanDiagnostic("init can", init_state);

  int start_state = VCI_StartCAN(device_type_, device_index_, can_index_);
  CanDiagnostic("start can", start_state);
}

void CanApplication::CloseCAN() {
  VCI_CloseDevice(device_type_, device_index_);
}

void CanApplication::CanDiagnostic(const std::string& description,
                                   const int& state) {
  switch (state) {
    case 1: {
      std::cout << "Successful! " << description << std::endl;
      break;
    }
    case 0: {
      std::cout << "Failure! " << description << std::endl;
      break;
    }
    case -1: {
      std::cout << "Lose USB Error! " << description << std::endl;
      break;
    }
    default: {
      std::cout << "Incorrect state feedback of CAN!" << std::endl;
      break;
    }
  }
}

PVCI_CAN_OBJ CanApplication::GetVciObject(const int& obj_num,
                                          const uint& initial_id) {
  PVCI_CAN_OBJ obj_ptr;
  obj_ptr = new VCI_CAN_OBJ[obj_num];
  for (size_t i = 0; i < obj_num; i++) {
    obj_ptr[i].ID = initial_id;
    obj_ptr[i].RemoteFlag = 0;
    obj_ptr[i].SendType = 0;
    obj_ptr[i].RemoteFlag = 0;
    obj_ptr[i].ExternFlag = 0;
  }

  return obj_ptr;
}

void CanApplication::SendCommand(PVCI_CAN_OBJ obj, const uint& obj_len, bool single_frame) {
  for (size_t i = 0; i < obj_len; i++) {
    if (!single_frame) {
      VCI_Transmit(device_type_, device_index_, can_index_, obj, frame_len_);
    } else {
      VCI_Transmit(device_type_, device_index_, can_index_, obj, 1);
    }
  }
}

void CanApplication::GetData(PVCI_CAN_OBJ obj, const int& obj_len) {
  int data_num = VCI_GetReceiveNum(device_type_, device_index_, can_index_);
  if (-1 == data_num) {
    std::cout << "Get data number failure!" << std::endl;
  } else if (0 == data_num) {
    std::cout << "No data in the buffer" << std::endl;
  }

  int receive_num = VCI_Receive(device_type_, device_index_, can_index_, obj,
                                obj_len, wait_time_);
}

void CanApplication::ClearBuffer() {
  VCI_ClearBuffer(device_type_, device_index_, can_index_);
}
