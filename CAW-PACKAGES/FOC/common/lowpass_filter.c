#include "lowpass_filter.h"

#include "stm32f4xx_hal.h"

void LOWPASS_FILTER_Init(LOWPASS_FILTER_T* filter, float time_constant) {
  filter->Tf = time_constant;
  filter->y_prev = 0.0f;
  filter->timestamp_prev = HAL_GetTick();
}

float LOWPASS_FILTER_Calc(LOWPASS_FILTER_T* filter, float x) {
  uint64_t timestamp = HAL_GetTick();
  float dt = (timestamp - filter->timestamp_prev) * 1e-3f;

  if (dt < 0.0f)
    dt = 1e-3f;
  else if (dt > 0.3f) {
    // 如果大于300ms则不处理
    filter->y_prev = x;
    filter->timestamp_prev = timestamp;
    return x;
  }
  float alpha = filter->Tf / (filter->Tf + dt);
  float y = alpha * filter->y_prev + (1.0f - alpha) * x;
  filter->y_prev = y;
  filter->timestamp_prev = timestamp;
  return y;
}