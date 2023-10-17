#ifndef __BLDC_DRIVER_6PWM_H__
#define __BLDC_DRIVER_6PWM_H__
#include <stdint.h>

#include "tim.h"

typedef struct _DRIVER_6PWM_T {
  TIM_HandleTypeDef* pwm_tim;  // PWM定时器
  uint16_t pwm_period;         // PWM周期
  uint64_t pwm_frequency;      // PWM频率（Hz）
  float voltage_power_supply;  // 电源电压
  float voltage_limit;         // 电压限制
  float dead_zone;             // 死区时间
} DRIVER_6PWM_T;

void DRIVER_6PWM_Init(DRIVER_6PWM_T* driver, TIM_HandleTypeDef* pwm_tim,
                      uint16_t pwm_period);
void DRIVER_6PWM_SetPWM(DRIVER_6PWM_T* driver, float Ua, float Ub, float Uc);

#endif