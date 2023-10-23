#include "current_sensor.h"

#include "caw_status.h"
#include "log.h"

float ADC_Get_Value(ADC_HandleTypeDef *hadc, uint32_t ch) {
  ADC_ChannelConfTypeDef cfg = {0};
  cfg.Channel = ch;
  cfg.Rank = 1;
  cfg.SamplingTime = ADC_SAMPLETIME_3CYCLES;
  if (HAL_ADC_ConfigChannel(hadc, &cfg)) {
    CAW_LOG_ERROR("HAL_ADC_ConfigChannel failed");
    return 0;
  }
  HAL_ADC_Start(hadc);
  HAL_ADC_PollForConversion(hadc, HAL_MAX_DELAY);
  float ret = (HAL_ADC_GetValue(hadc) & 0x0fff) * 3.3f / 4096.0f;
  HAL_ADC_Stop(hadc);
  return ret;
}

void CURRENT_SENSOR_Init(CURRENT_SENSOR_T *sensor, ADC_HandleTypeDef *hadc) {
  sensor->hadc = hadc;
  sensor->CSA_GAIN = 40.0f;
  sensor->shunt_resistor = 0.001f;  // 1毫欧
  sensor->volte_to_amps_ratio =
      1.0f / sensor->shunt_resistor / sensor->CSA_GAIN;
  sensor->gain_a = sensor->volte_to_amps_ratio;
  sensor->gain_b = sensor->volte_to_amps_ratio;
  sensor->gain_c = sensor->volte_to_amps_ratio;
  sensor->offset_ia = 0.0f;
  sensor->offset_ib = 0.0f;
}

/**
 * @description: 计算零位飘移
 * @param {CURRENT_SENSOR_T*} sensor
 * @return {*}
 */
void CURRENT_SENSOR_CalibrateOffsets(CURRENT_SENSOR_T *sensor) {
  const int calibration_rounds = 1000;
  for (int i = 0; i < calibration_rounds; i++) {
    sensor->offset_ia += ADC_Get_Value(sensor->hadc, ADC_CHANNEL_8);
    sensor->offset_ib += ADC_Get_Value(sensor->hadc, ADC_CHANNEL_9);
    HAL_Delay(1);
  }
  sensor->offset_ia = sensor->offset_ia / calibration_rounds;
  sensor->offset_ib = sensor->offset_ib / calibration_rounds;
}

int CURRENT_SENSOR_Update(CURRENT_SENSOR_T *sensor) {
  sensor->current.a =
      (ADC_Get_Value(sensor->hadc, ADC_CHANNEL_8) - sensor->offset_ia) *
      sensor->gain_a;
  sensor->current.b =
      (ADC_Get_Value(sensor->hadc, ADC_CHANNEL_9) - sensor->offset_ib) *
      sensor->gain_b;
  sensor->current.c = 0.0f;
  return CAW_OK;
}