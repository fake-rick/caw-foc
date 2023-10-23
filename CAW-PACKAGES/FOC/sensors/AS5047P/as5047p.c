#include "as5047p.h"

#include "caw_status.h"
#include "log.h"

#define abs(x) ((x) > 0 ? (x) : -(x))
#define _2PI 6.283185307179586f
#define _PI 3.141592653589793f

// SPI通讯NSS
#define _NSS(x) HAL_GPIO_WritePin(SPI1_NSS_GPIO_Port, SPI1_NSS_Pin, x)

static inline int _ReadWrite(AS5047P_T *a, uint16_t tx, uint16_t *rx) {
  _NSS(0);
  int ret = HAL_SPI_TransmitReceive(a->hspi, &tx, rx, 1, HAL_MAX_DELAY);
  _NSS(1);
  if (ret) CAW_LOG_ERROR("_ReadWrite failed");
  return HAL_OK == ret ? CAW_OK : CAW_ERR;
}

static inline int _ReadData(AS5047P_T *a) {
  uint16_t v;
_AGAIN:
  _ReadWrite(a, 0xffff, &v);
  _ReadWrite(a, 0xffff, &(a->raw_angle));
  if (a->raw_angle & 0x4000) {
    _ReadWrite(a, 0x4001, &v);
    _ReadWrite(a, 0xc000, &v);
    // CAW_LOG_ERROR("ERR CODE: %x", v);
    goto _AGAIN;
  }
  a->raw_angle &= 0x3fff;
  return CAW_OK;
}

int AS5047P_Init(AS5047P_T *a, SPI_HandleTypeDef *hspi) {
  a->hspi = hspi;

  _ReadData(a);
  a->prev_angle = a->raw_angle;
  a->prev_angle_ts = 0;
  a->rotation_offset = 0;

  a->vel_rotation_offset = 0;
  a->vel_prev_angle = a->raw_angle;
  a->vel_prev_angle_ts = 0;
  a->err_code = 0;
  return CAW_OK;
}

uint16_t AS5047P_GetRawAngle(AS5047P_T *a) {
  _ReadData(a);
  return a->raw_angle;
}

float AS5047P_GetOnceAngle(AS5047P_T *a) {
  _ReadData(a);
  return a->raw_angle * (360.0 / AS5047P_RESOLUTION) * _PI / 180.0;
}

float AS5047P_GetAngle(AS5047P_T *a) {
  return (a->rotation_offset +
          (a->prev_angle / (float)AS5047P_RESOLUTION) * _2PI);
}

int AS5047P_Update(AS5047P_T *a) {
  _ReadData(a);
  float delta = a->raw_angle - a->prev_angle;
  a->prev_angle_ts = HAL_GetTick();
  if (abs(delta) > (0.8 * AS5047P_RESOLUTION)) {
    a->rotation_offset += (delta > 0 ? -_2PI : _2PI);
  }
  a->prev_angle = a->raw_angle;
  return CAW_OK;
}

float AS5047P_GetVelocity(AS5047P_T *a) {
  float ts = (a->prev_angle_ts - a->vel_prev_angle_ts) * 1e-3;
  if (ts <= 0) ts = 1e-3f;
  float vel =
      ((a->rotation_offset - a->vel_rotation_offset) +
       (a->prev_angle - a->vel_prev_angle) / (float)AS5047P_RESOLUTION * _2PI) /
      ts;
  a->vel_prev_angle = a->prev_angle;
  a->vel_rotation_offset = a->rotation_offset;
  a->vel_prev_angle_ts = a->prev_angle_ts;
  return vel;
}