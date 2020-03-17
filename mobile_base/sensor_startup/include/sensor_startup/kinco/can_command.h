#ifndef CAN_COMMAND_H
#define CAN_COMMAND_H

#define TPDO1_ID 0x180
#define TPDO2_ID 0x280
#define TPDO3_ID 0x380
#define TPDO4_ID 0x480

#define RPDO1_ID 0x200
#define RPDO2_ID 0x300
#define RPDO3_ID 0x400
#define RPDO4_ID 0x500

#define NMT_MODULE_CONTROL_ID 0x00
#define SYNC_ID 0x80

#define VELOCITY_MODE 0
#define POSITION_MODE 1
#define CURRENT_MODE 2

#define ENABLE_CMD 0x0f
#define ENABLE_HOMING 0x1f
#define DISENABLE_CMD 0x06

#define IMMEDIATE_VELOCITY 0xfd

typedef uint8_t _u8;

namespace mobile_base {

struct CanCommand {
  _u8 PDO_START_WORK[2] = {0x01, 0x00};

  _u8 SET_VELOCITY_MODE[1] = {0xfd};
  _u8 SET_POSITION_MODE[1] = {0x01};
  _u8 SET_CURRENT_MODE[1] = {1};
  _u8 SET_HOMING_MODE[1] = {0x06};

  _u8 ENABLE_VELOCITY_CMD[2] = {0x0f, 0x00};
  _u8 ENABLE_POSITION_CMD[2] = {0x3f, 0x10};
  _u8 ENABLE_HOMING_CMD[2] = {0x1f, 0x00};

  _u8 VELOCITY_COMMAND[1] = {1};
  _u8 POSITION_COMMAND[1] = {1};
  _u8 CURRENT_COMMAND[1] = {1};

  _u8 SET_TRAPEZOID_VELOCITY[8] = {0x81, 0x60, 0x20, 0x00,
                                   0x00, 0x00, 0x00, 0x00};
};  // struct CanCommand

}  // namespace mobile_base

#endif
