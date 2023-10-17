#include "motor.h"

#include <math.h>

#include "adc.h"
#include "caw_status.h"
#include "foc_utils.h"
#include "log.h"
#include "sensor.h"
#include "stm32f4xx_hal.h"

void MOTOR_Init(MOTOR_T* motor, int pp) {
  motor->pole_pairs = pp;
  motor->modulation_centered = 1;

  PID_Init(&(motor->PID_velocity), 0.5f, 20.0f, 0.0f, 1000.0f, 24.0f);
  PID_Init(&(motor->PID_angle), 20.0f, 0, 0, 0, 24.0f);

  PID_Init(&(motor->PID_current_q), 5.0f, 200.0f, 0, 0.0f, 24.0f);
  PID_Init(&(motor->PID_current_d), 5.0f, 300.0f, 0, 0.0f, 24.0f);

  LOWPASS_FILTER_Init(&(motor->LPF_velocity), 0.005f);
  LOWPASS_FILTER_Init(&(motor->LPF_angle), 0.005f);

  LOWPASS_FILTER_Init(&(motor->LPF_current_q), 0.01f);
  LOWPASS_FILTER_Init(&(motor->LPF_current_d), 0.01f);

  motor->voltage_sensor_align = 3.0f;
  motor->sensor_direction = -1;
  motor->feed_forward_velocity = 0.0f;
  motor->sensor_offset = 0.0f;
  // 电流值初始化
  motor->current_sp = 0;
  motor->current.d = 0;
  motor->current.q = 0;

  motor->open_loop_timestamp = 0;

  SENSOR_Init(&hspi1);
  CURRENT_SENSOR_Init(&(motor->current_sensor), &hadc1);
  CURRENT_SENSOR_CalibrateOffsets(&(motor->current_sensor));
}

void MOTOR_LinkDriver(MOTOR_T* motor, DRIVER_6PWM_T* driver) {
  motor->driver = driver;
}

void MOTOR_SetPhaseVoltage(MOTOR_T* motor, float Uq, float Ud, float angle_el) {
  int sector;
  float Uout;

  if (Ud) {
    Uout = sqrt(Ud * Ud + Uq * Uq) / motor->driver->voltage_limit;
    angle_el = _normalizeAngle(angle_el + atan2(Uq, Ud));
  } else {
    Uout = Uq / motor->driver->voltage_limit;
    angle_el = _normalizeAngle(angle_el + _PI_2);
  }
  sector = floor(angle_el / _PI_3) + 1;
  float T1 = _SQRT3 * sin(sector * _PI_3 - angle_el) * Uout;
  float T2 = _SQRT3 * sin(angle_el - (sector - 1.0f) * _PI_3) * Uout;
  float T0 = 0;
  if (motor->modulation_centered) {
    T0 = 1 - T1 - T2;
  }

  // 计算占空比
  float Ta, Tb, Tc;
  switch (sector) {
    case 1:
      Ta = T1 + T2 + T0 / 2;
      Tb = T2 + T0 / 2;
      Tc = T0 / 2;
      break;
    case 2:
      Ta = T1 + T0 / 2;
      Tb = T1 + T2 + T0 / 2;
      Tc = T0 / 2;
      break;
    case 3:
      Ta = T0 / 2;
      Tb = T1 + T2 + T0 / 2;
      Tc = T2 + T0 / 2;
      break;
    case 4:
      Ta = T0 / 2;
      Tb = T1 + T0 / 2;
      Tc = T1 + T2 + T0 / 2;
      break;
    case 5:
      Ta = T2 + T0 / 2;
      Tb = T0 / 2;
      Tc = T1 + T2 + T0 / 2;
      break;
    case 6:
      Ta = T1 + T2 + T0 / 2;
      Tb = T0 / 2;
      Tc = T1 + T0 / 2;
      break;
    default:
      // 错误状态
      Ta = 0;
      Tb = 0;
      Tc = 0;
  }

  // 计算相位电压和中心
  motor->Ua = Ta * motor->driver->voltage_limit;
  motor->Ub = Tb * motor->driver->voltage_limit;
  motor->Uc = Tc * motor->driver->voltage_limit;

  DRIVER_6PWM_SetPWM(motor->driver, motor->Ua, motor->Ub, motor->Uc);
}

float MOTOR_VelocityOpenloop(MOTOR_T* motor, float target_velocity) {
  uint64_t now_us = HAL_GetTick();
  float Ts = (now_us - motor->open_loop_timestamp) * 1e-3f;
  if (Ts <= 0 || Ts > 0.5f) Ts = 1e-3f;

  motor->shaft_angle =
      _normalizeAngle(motor->shaft_angle + target_velocity * Ts);
  motor->shaft_velocity = target_velocity;

  float Uq = motor->voltage_limit;
  MOTOR_SetPhaseVoltage(
      motor, Uq, 0, _electricalAngle(motor->shaft_angle, motor->pole_pairs));

  motor->open_loop_timestamp = now_us;
  return Uq;
}

void MOTOR_Step(MOTOR_T* motor) {
  if (SENSOR_Update()) return;
  motor->electrical_angle = MOTOR_ElectricalAngle(motor);

  if (CURRENT_SENSOR_Update(&(motor->current_sensor))) return;

  MOTOR_GetCurrent(motor, &(motor->current));

  motor->current.q =
      LOWPASS_FILTER_Calc(&(motor->LPF_current_q), motor->current.q);
  motor->voltage.q =
      PID_Calc(&(motor->PID_current_q), motor->current_sp - motor->current.q);

  MOTOR_SetPhaseVoltage(motor, motor->voltage.q, 0, motor->electrical_angle);
}

float MOTOR_ElectricalAngle(MOTOR_T* motor) {
  return _normalizeAngle((float)(motor->sensor_direction * motor->pole_pairs) *
                             SENSOR_GetAngle() -
                         motor->zero_electric_angle);
}

int MOTOR_AlignSensor(MOTOR_T* motor) {
  MOTOR_SetPhaseVoltage(motor, motor->voltage_sensor_align, 0, _3PI_2);
  HAL_Delay(700);
  SENSOR_Update();
  motor->zero_electric_angle = 0;
  motor->zero_electric_angle = MOTOR_ElectricalAngle(motor);
  HAL_Delay(20);
  MOTOR_SetPhaseVoltage(motor, 0, 0, 0);
  HAL_Delay(200);
}

void MOTOR_Move(MOTOR_T* motor, float new_target) {
  motor->shaft_angle = MOTOR_ShaftAngle(motor);
  motor->shaft_velocity = MOTOR_ShaftVelocity(motor);
  //! 闭环速度控制
  // motor->shaft_velocity_sp = new_target;
  // motor->current_sp = PID_Calc(
  //     &(motor->PID_velocity), motor->shaft_velocity_sp -
  //     motor->shaft_velocity);

  // motor->voltage.q = motor->current_sp;
  // motor->voltage.d = 0;

  //! 闭环角度控制
  motor->shaft_angle_sp = new_target;
  // calculate velocity set point
  motor->shaft_velocity_sp =
      motor->feed_forward_velocity +
      PID_Calc(&(motor->PID_angle), motor->shaft_angle_sp - motor->shaft_angle);
  motor->shaft_angle_sp = _constrain(
      motor->shaft_angle_sp, -motor->velocity_limit, motor->velocity_limit);
  // calculate the torque command - sensor precision: this calculation is ok,
  // but based on bad value from previous calculation
  motor->current_sp =
      PID_Calc(&(motor->PID_velocity),
               motor->shaft_velocity_sp -
                   motor->shaft_velocity);  // if voltage torque control
  // if torque controlled through voltage
  motor->voltage.q = motor->current_sp;
  motor->voltage.d = 0;

  // motor->voltage.d = 0;
  // motor->current_sp = new_target;
}

float MOTOR_ShaftVelocity(MOTOR_T* motor) {
  return motor->sensor_direction *
         LOWPASS_FILTER_Calc(&(motor->LPF_velocity), SENSOR_GetVelocity());
}

float MOTOR_ShaftAngle(MOTOR_T* motor) {
  return motor->sensor_direction *
             LOWPASS_FILTER_Calc(&(motor->LPF_angle), SENSOR_GetAngle()) -
         motor->sensor_offset;
}

void MOTOR_GetCurrent(MOTOR_T* motor, DQ_CURRENT_T* current) {
  float i_alpha = motor->current_sensor.current.a;
  float i_beta = _1_SQRT3 * motor->current_sensor.current.a +
                 _2_SQRT3 * motor->current_sensor.current.b;
  float ct = cos(motor->electrical_angle);
  float st = sin(motor->electrical_angle);
  current->d = i_alpha * ct + i_beta * st;
  current->q = i_beta * ct - i_alpha * st;
}