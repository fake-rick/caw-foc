#ifndef __SENSOR_H__
#define __SENSOR_H__
#include <stdint.h>

#include "spi.h"

int SENSOR_Init(SPI_HandleTypeDef *hspi);
uint16_t SENSOR_GetRawAngle();
float SENSOR_GetOnceAngle();
float SENSOR_GetAngle();
int SENSOR_Update();
float SENSOR_GetVelocity();

#endif