#ifndef __PID_H__
#define __PID_H__

#include <stdint.h>

typedef struct _PID_T {
  float P;            // 比例增益
  float I;            // 积分增益
  float D;            // 微分增益
  float output_ramp;  // 输出值的最大变化速度
  float limit;        // 最大输出值

  float error_prev;         // 跟踪最后一次误差值
  float output_prev;        // 最后一次PID输出值
  float integral_prev;      // 最后一次积分值
  uint64_t timestamp_prev;  // 记录最后一次时间戳
} PID_T;

void PID_Init(PID_T* pid, float P, float I, float D, float ramp, float limit);
float PID_Calc(PID_T* pid, float error);
void PID_Reset(PID_T* pid);

#endif