#include <iostream>
#include <serial/serial.h>
#include <string>
#include <unistd.h>

#include "ros/ros.h"
#include "sensor_msgs/Range.h"

#include "sensor_startup/controlcan.h"
/*
#define ENCODER_1 0x01
#define ENCODER_2 0x02
#define ENCODER_3 0x03
#define ENCODER_4 0x04

typedef uint8_t _u8;
typedef uint16_t _u16;
_u8 cmd[8] = {0x01, 0x03, 0x00, 0x00, 0x00, 0x02, 0xc4, 0x0b};

_u8 REQUEST_DATA_1[8] = {0x01, 0x03, 0x00, 0x00, 0x00, 0x02, 0xc4, 0x0b};
_u8 REQUEST_DATA_2[8] = {0x02, 0x03, 0x00, 0x00, 0x00, 0x02, 0xc4, 0x38};
_u8 REQUEST_DATA_3[8] = {0x03, 0x03, 0x00, 0x00, 0x00, 0x02, 0xc5, 0xe9};
_u8 REQUEST_DATA_4[8] = {0x04, 0x03, 0x00, 0x00, 0x00, 0x02, 0xc4, 0x5e};
serial::Serial encod_ser;

void InitSerial(const std::string& port_id, const int& baudrate);
void SetAddress();
void ReadEncoder();

int main(int argc, char** argv) {
  ros::init(argc, argv, "abs_encoder");
  ros::NodeHandle n;

  std::string port;
  int baudrate;
  if (!ros::param::get("port", port)) {
    port = "/dev/ttyUSB0";
  }
  if (!ros::param::get("baudrate", baudrate)) {
    baudrate = 9600;
  }
  
  ros::Publisher pub = n.advertise<sensor_msgs::Range>("abd_impulse", 100);
  ros::Rate r(10);

  InitSerial(port, baudrate);
  if (encod_ser.isOpen()) {
    while(ros::ok()) {
      ReadEncoder();
      ros::spinOnce();
      r.sleep();
    }
  }

  return 0;

}


void InitSerial(const std::string& port_id, const int& baudrate) {
  try {
    encod_ser.setPort(port_id);
    encod_ser.setBaudrate(baudrate);
    encod_ser.setParity(serial::parity_none);
    serial::Timeout time_out = serial::Timeout::simpleTimeout(1000);
    encod_ser.setTimeout(time_out);
    encod_ser.open();
  } catch (const serial::IOException& ex) {
    ROS_INFO("Unable to open port on %s -> [%s]", port_id, ex.what());
    return ;
  }

}

void ReadEncoder() {
  std::vector<_u8> data_1, data_2, data_3, data_4;

  encod_ser.write(REQUEST_DATA_1, 8);
  if (encod_ser.available()) {
    int size = encod_ser.available();
    encod_ser.read(data_1, size);
  }
  int result_1;
  result_1 = ((_u16)data_1[3] << 24) + ((_u16)data_1[4] << 16) +
             ((_u16)data_1[5] << 8) + (_u16)data_1[6];

  encod_ser.write(REQUEST_DATA_2, 8);
  if (encod_ser.available()) {
    int size = encod_ser.available();
    encod_ser.read(data_2, size);
  }
  int result_2;
  result_2 = ((_u16)data_2[3] << 24) + ((_u16)data_2[4] << 16) +
             ((_u16)data_2[5] << 8) + (_u16)data_2[6];

  encod_ser.write(REQUEST_DATA_3, 8);
  if (encod_ser.available()) {
    int size = encod_ser.available();
    encod_ser.read(data_3, size);
  }
  int result_3;
  result_3 = ((_u16)data_3[3] << 24) + ((_u16)data_3[4] << 16) +
             ((_u16)data_3[5] << 8) + (_u16)data_3[6];

  encod_ser.write(REQUEST_DATA_4, 8);
  if (encod_ser.available()) {
    int size = encod_ser.available();
    encod_ser.read(data_4, size);
  }
  int result_4;
  result_4 = ((_u16)data_4[3] << 24) + ((_u16)data_4[4] << 16) +
             ((_u16)data_4[5] << 8) + (_u16)data_4[6];
}

*/

typedef uint8_t _u8;
unsigned int dev_type = VCI_USBCAN2; 
unsigned int dev_ind = 0;
unsigned int can_ind = 0;
_u8 REQUEST_ENCODER_1[4] = {0x04, 0x01, 0x01, 0x00};
_u8 REQUEST_ENCODER_2[4] = {0x04, 0x02, 0x01, 0x00};
_u8 REQUEST_ENCODER_3[4] = {0x04, 0x03, 0x01, 0x00};
_u8 REQUEST_ENCODER_4[4] = {0x04, 0x04, 0x01, 0x00};

void CanState(const uint& s, const std::string mode);
void StartCan();
void DataTransform(BYTE* data, uint8_t* cmd, const int& len);

int main(int argc, char** argv) {
  ros::init(argc, argv, "abs_encoder_can");
  ros::NodeHandle nh;

  StartCan();
  
  
  

  
}

PVCI_CAN_OBJ GetVciObject(const int& num) {
  PVCI_CAN_OBJ obj;
  obj = new VCI_CAN_OBJ[num];
  for (size_t i = 0; i < num; i++) {
    obj[i].ID = 0x00000000;
    obj[i].ID += (i + 1);
    obj[i].DataLen = 4;

    obj[i].ExternFlag = 0;
    obj[i].RemoteFlag = 0;
    obj[i].SendType = 0;
  }
  return obj;
}

void SendRequest() {
  int num = 4;
  PVCI_CAN_OBJ obj = GetVciObject(num);

  DataTransform(obj[0].Data, REQUEST_ENCODER_1, 4);
  DataTransform(obj[1].Data, REQUEST_ENCODER_2, 4);
  DataTransform(obj[2].Data, REQUEST_ENCODER_3, 4);
  DataTransform(obj[3].Data, REQUEST_ENCODER_4, 4);

  VCI_Transmit(dev_type, dev_ind, can_ind, obj, num);
  // for (size_t i = 0; i < num; i++) {
  //   VCI_Transmit(dev_type, dev_ind, can_ind, &obj[i], 1);
  //   usleep(1000);
  // }
  delete[] obj;
}

void ReadLoop() {
  SendRequest();
  int data_num = VCI_GetReceiveNum(dev_type, dev_ind, can_ind);
  if (-1 == data_num) { 
    ROS_WARN("Get quantity of CAN data of absolute encoder failure");
    return ;
  }

  PVCI_CAN_OBJ rec_obj;
  std::vector<int> encod_data;
  int encod_count;  // this is for checking whether enough data is obtained

  int rec_num =
      VCI_Receive(dev_type, dev_ind, can_ind, rec_obj, data_num, 0);
  if (-1 == rec_num) {
    ROS_WARN("Get data of absolute encoder from CAN failure");
    return ;
  } else if (0 != rec_num) {
    for (size_t i = 0; i < rec_num; i++) {
      switch (rec_obj[i].ID) {
        case 0x00000001: {
          encod_data[0] = (rec_obj[i].Data[6] << 24) +
                          (rec_obj[i].Data[5] << 16) +
                          (rec_obj[i].Data[4] << 8) + (rec_obj[i].Data[3] << 0);
          encod_count++;
          break;
        }
        case 0x00000002: {
          encod_data[1] = (rec_obj[i].Data[6] << 24) +
                          (rec_obj[i].Data[5] << 16) +
                          (rec_obj[i].Data[4] << 8) + (rec_obj[i].Data[3] << 0);
          encod_count++;
          break;
        }
        case 0x00000003: {
          encod_data[2] = (rec_obj[i].Data[6] << 24) +
                          (rec_obj[i].Data[5] << 16) +
                          (rec_obj[i].Data[4] << 8) + (rec_obj[i].Data[3] << 0);
          encod_count++;
          break;
        }
        case 0x00000004: {
          encod_data[3] = (rec_obj[i].Data[6] << 24) +
                          (rec_obj[i].Data[5] << 16) +
                          (rec_obj[i].Data[4] << 8) + (rec_obj[i].Data[3] << 0);
          encod_count++;
          break;
        }
        default: { break; }
      }
    }
    ROS_INFO("encoder data is [1: %d, 2: %d, 3: %d, 4: %d", 
             encod_data[0], encod_data[1], encod_data[2], encod_data[3]);
  }
}

void CanState(const uint& s, const std::string mode) {
  switch (s) {
    case 1: {
      std::cout << mode << " : ";
      std::cout << "successfull" << std::endl;
      break;
    }
    case 0: {
      std::cout << mode << " : ";
      std::cout << "failure" << std::endl;
      break;
    }
    case -1: {
      std::cout << mode << " : ";
      std::cout << "lose USB" << std::endl;
      break;
    }
    default: break;
  }
}

void StartCan() {


  int state = VCI_OpenDevice(dev_type, dev_ind, 0);
  CanState(state, "open device"); 
  if (state != 1) { exit(0); }
   
  VCI_INIT_CONFIG config;
  config.AccCode = 0x00000000;
  config.AccMask = 0xFFFFFFFF;
  config.Filter = 0;
  config.Mode = 0;
  config.Timing0 = 0x00;
  config.Timing1 = 0x1c;

  state = VCI_InitCAN(dev_type, dev_ind, can_ind, &config);
  CanState(state, "init");
  if (state != 1) { exit(0); }

  state = VCI_StartCAN(dev_type, dev_ind, can_ind);
  CanState(state, "start");
  if (state != 1) { exit(0); }

}

void DataTransform(BYTE* data, uint8_t* cmd, const int& len) {
  for (size_t i = 0; i < len; i++) {
    data[i] = cmd[i];
  } 
}
