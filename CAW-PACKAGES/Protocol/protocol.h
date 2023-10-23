#ifndef __PROTOCOL_H__
#define __PROTOCOL_H__

#include <stdint.h>

// |cmd|action(0:set 1:get)|version|accuracy(0:int32)|data(4bytes)|

#define PROTOCOL_VERSION 0x05  // 0b00000101 (v1.01)

typedef enum _CMD_E {
  PROTOCOL_STOP = 0x00,
  PROTOCOL_START = 0x01,
  PROTOCOL_CONTROL_TYPE = 0x02,
  PROTOCOL_CURRENT_P = 0x03,
  PROTOCOL_CURRENT_I = 0x04,
  PROTOCOL_CURRENT_D = 0x05,
  PROTOCOL_VELOCITY_P = 0x06,
  PROTOCOL_VELOCITY_I = 0x07,
  PROTOCOL_VELOCITY_D = 0x08,
  PROTOCOL_ANGLE_P = 0x09,
  PROTOCOL_ANGLE_I = 0x0A,
  PROTOCOL_ANGLE_D = 0x0B,
  PROTOCOL_LPF_CURRENT = 0x0C,
  PROTOCOL_LPF_VELOCITY = 0x0D,
  PROTOCOL_LPF_ANGLE = 0x0E,
  PROTOCOL_VELOCITY_SP = 0x0F,  // shaft_velocity_sp
  PROTOCOL_TARGET = 0x10,
  PROTOCOL_CURRENT_VALUE = 0x11,
  PROTOCOL_VELOCITY_VALUE = 0x12,
  PROTOCOL_ANGLE_VALUE = 0x13,
  PROTOCOL_REPLY = 0xff,
} CMD_E;

typedef enum _ACTION_E {
  PROTOCOL_NONE = 0x00,
  PROTOCOL_SET = 0x01,
  PROTOCOL_GET = 0x02,
} ACTION_E;

typedef enum _CONTROL_TYPE_E {
  PROTOCOL_TORQUE = 0x00,
  PROTOCOL_VELOCITY = 0x01,
  PROTOCOL_ANGLE = 0x02,
  PROTOCOL_VELOCITY_OPENLOOP = 0x03,
  PROTOCOL_ANGLE_OPENLOOP = 0x04,
} CONTROL_TYPE_E;

#pragma pack(1)
typedef struct _PROTOCOL_T {
  uint8_t cmd;
  uint8_t action;
  uint8_t version;
  uint8_t accuracy;
  float data;
} PROTOCOL_T;
#pragma pack()

#define PROTOCOL_DATA_OFFSET offsetof(PROTOCOL_T, data)

void PROTOCOL_Unpack(PROTOCOL_T* protocol, const uint8_t* buf);

void PROTOCOL_Pack(uint8_t cmd, uint8_t action, uint8_t accuracy, float data,
                   uint8_t* buf);

#endif