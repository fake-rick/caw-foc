#ifndef __AS5047P_H__
#define __AS5047P_H__

#include <stdint.h>

#include "spi.h"

#define AS5047P_RESOLUTION 16384

typedef struct {
  SPI_HandleTypeDef *hspi;
  float prev_angle;
  unsigned long prev_angle_ts;
  float rotation_offset;

  float vel_rotation_offset;        // 计算速度使用的圈数累加
  float vel_prev_angle;             // 计算速度使用的角度
  unsigned long vel_prev_angle_ts;  // 计算速度使用的时间记录

  uint16_t raw_angle;  // 原始角度数据

  int err_code;
} AS5047P_T;

int AS5047P_Init(AS5047P_T *a, SPI_HandleTypeDef *hspi);

uint16_t AS5047P_GetRawAngle(AS5047P_T *a);
float AS5047P_GetOnceAngle(AS5047P_T *a);
float AS5047P_GetAngle(AS5047P_T *a);
int AS5047P_Update(AS5047P_T *a);
float AS5047P_GetVelocity(AS5047P_T *a);

#endif