#ifndef __BLDC_MOTOR_H__
#define __BLDC_MOTOR_H__
#include "./drivers/driver_6pwm.h"
#include "current_sensor.h"
#include "foc_utils.h"
#include "lowpass_filter.h"
#include "pid.h"

typedef enum _MOTION_CONTROL_TYPE_E {
  TORQUE = 0x00,
  VELOCITY = 0x01,
  ANGLE = 0x02,
  VELOCITY_OPENLOOP = 0x03,
  ANGLE_OPENLOOP = 0x04,
} MOTION_CONTROL_TYPE_E;

typedef struct _MOTOR_T {
  int pole_pairs;              // 电机极对数
  int8_t modulation_centered;  // flag (1) centered modulation around driver
                               // limit /2  or  (0) pulled to 0
  DRIVER_6PWM_T* driver;
  float Ua, Ub, Uc;        // 用于设定的相电压Ua,Ub,Uc
  float shaft_angle;       // 当前电机角度
  float shaft_velocity;    // 当前电机速度
  float electrical_angle;  // 当前电角度
  float voltage_limit;     // 电压限制变量 - 全局限制
  float velocity_limit;    // 速度极限变量 - 全局极限
  uint64_t open_loop_timestamp;

  float phase_resistance;  // 电机相电阻

  PID_T PID_velocity;
  PID_T PID_angle;
  PID_T PID_current_q;
  PID_T PID_current_d;
  LOWPASS_FILTER_T LPF_velocity;
  LOWPASS_FILTER_T LPF_angle;
  LOWPASS_FILTER_T LPF_current_q;
  LOWPASS_FILTER_T LPF_current_d;

  float voltage_sensor_align;  // 传感器和电机对齐电压参数
  int sensor_direction;        // 传感器方向
  float zero_electric_angle;   // 绝对零电角
  float sensor_offset;         // 自定义传感器零点偏移

  CURRENT_SENSOR_T current_sensor;  // 电流传感器

  DQ_VOLTAGE_T voltage;
  DQ_CURRENT_T current;

  float current_sp;             // 目标电流 （q 电流）
  float shaft_angle_sp;         // 当前目标角度
  float shaft_velocity_sp;      // 当前目标速度
  float feed_forward_velocity;  // 当前前馈速度

  MOTION_CONTROL_TYPE_E controller;
} MOTOR_T;

void MOTOR_Init(MOTOR_T* motor, int pp);
void MOTOR_LinkDriver(MOTOR_T* motor, DRIVER_6PWM_T* driver);
void MOTOR_SetPhaseVoltage(MOTOR_T* motor, float Uq, float Ud, float angle_el);
float MOTOR_VelocityOpenloop(MOTOR_T* motor, float target_velocity);
void MOTOR_Step(MOTOR_T* motor);
float MOTOR_ElectricalAngle(MOTOR_T* motor);
int MOTOR_AlignSensor(MOTOR_T* motor);
void MOTOR_Move(MOTOR_T* motor, float new_target);
float MOTOR_ShaftVelocity(MOTOR_T* motor);
float MOTOR_ShaftAngle(MOTOR_T* motor);
void MOTOR_GetCurrent(MOTOR_T* motor, DQ_CURRENT_T* current);
#endif