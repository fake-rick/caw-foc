#include "sensor.h"

#include "as5047p.h"

AS5047P_T g_sensor;

int SENSOR_Init(SPI_HandleTypeDef *hspi) {
  return AS5047P_Init(&g_sensor, hspi);
}

uint16_t SENSOR_GetRawAngle() { return AS5047P_GetRawAngle(&g_sensor); }

float SENSOR_GetOnceAngle() { return AS5047P_GetOnceAngle(&g_sensor); }

float SENSOR_GetAngle() { return AS5047P_GetAngle(&g_sensor); }

int SENSOR_Update() { return AS5047P_Update(&g_sensor); }

float SENSOR_GetVelocity() { return AS5047P_GetVelocity(&g_sensor); }