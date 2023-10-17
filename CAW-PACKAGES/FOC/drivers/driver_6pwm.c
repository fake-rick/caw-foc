#include "driver_6pwm.h"

#include "../common/foc_utils.h"

void DRIVER_6PWM_Init(DRIVER_6PWM_T* driver, TIM_HandleTypeDef* pwm_tim,
                      uint16_t pwm_period) {
  driver->pwm_tim = pwm_tim;
  driver->pwm_period = pwm_period;
  driver->dead_zone = 0.02f;

  HAL_TIM_PWM_Start(driver->pwm_tim, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(driver->pwm_tim, TIM_CHANNEL_2);
  HAL_TIM_PWM_Start(driver->pwm_tim, TIM_CHANNEL_3);

  HAL_TIMEx_PWMN_Start(driver->pwm_tim, TIM_CHANNEL_1);
  HAL_TIMEx_PWMN_Start(driver->pwm_tim, TIM_CHANNEL_2);
  HAL_TIMEx_PWMN_Start(driver->pwm_tim, TIM_CHANNEL_3);

  __HAL_TIM_SET_COMPARE(driver->pwm_tim, TIM_CHANNEL_1, 0);
  __HAL_TIM_SET_COMPARE(driver->pwm_tim, TIM_CHANNEL_2, 0);
  __HAL_TIM_SET_COMPARE(driver->pwm_tim, TIM_CHANNEL_3, 0);
}

void DRIVER_6PWM_SetPWM(DRIVER_6PWM_T* driver, float Ua, float Ub, float Uc) {
  Ua = _constrain(Ua, 0, driver->voltage_limit);
  Ub = _constrain(Ub, 0, driver->voltage_limit);
  Uc = _constrain(Uc, 0, driver->voltage_limit);

  float dc_a = _constrain(Ua / driver->voltage_power_supply, 0.0f, 1.0f);
  float dc_b = _constrain(Ub / driver->voltage_power_supply, 0.0f, 1.0f);
  float dc_c = _constrain(Uc / driver->voltage_power_supply, 0.0f, 1.0f);

  __HAL_TIM_SET_COMPARE(driver->pwm_tim, TIM_CHANNEL_1,
                        dc_a * driver->pwm_period);
  __HAL_TIM_SET_COMPARE(driver->pwm_tim, TIM_CHANNEL_2,
                        dc_b * driver->pwm_period);
  __HAL_TIM_SET_COMPARE(driver->pwm_tim, TIM_CHANNEL_3,
                        dc_c * driver->pwm_period);
}