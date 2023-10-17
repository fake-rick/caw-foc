#ifndef __LOWPASS_FILTER_H__
#define __LOWPASS_FILTER_H__
#include <stdint.h>

typedef struct _LOWPASS_FILTER_T {
  float Tf;                 // 低通滤波时间常数
  uint64_t timestamp_prev;  // 记录上次执行的时间戳
  float y_prev;             // 上一个时间步的过滤值
} LOWPASS_FILTER_T;

void LOWPASS_FILTER_Init(LOWPASS_FILTER_T* filter, float time_constant);
float LOWPASS_FILTER_Calc(LOWPASS_FILTER_T* filter, float x);

#endif