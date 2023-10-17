#ifndef __CURRENT_SENSOR_H__
#define __CURRENT_SENSOR_H__
#include <stdint.h>

#include "adc.h"
#include "foc_utils.h"

typedef struct _CURRENT_SENSOR_T {
  ADC_HandleTypeDef *hadc;
  float CSA_GAIN;  //
  float gain_a, gain_b, gain_c;
  // float current_a, current_b, current_c;
  PHASE_CURRENT_T current;
  float offset_ia, offset_ib;
  int err_code;
  float shunt_resistor;
  float volte_to_amps_ratio;
} CURRENT_SENSOR_T;

void CURRENT_SENSOR_Init(CURRENT_SENSOR_T *sensor, ADC_HandleTypeDef *hadc);
int CURRENT_SENSOR_Update(CURRENT_SENSOR_T *sensor);
void CURRENT_SENSOR_CalibrateOffsets(CURRENT_SENSOR_T *sensor);

#endif