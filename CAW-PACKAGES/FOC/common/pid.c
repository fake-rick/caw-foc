#include "pid.h"

#include "foc_utils.h"
#include "stm32f4xx_hal.h"

void PID_Init(PID_T* pid, float P, float I, float D, float ramp, float limit) {
  pid->P = P;
  pid->I = I;
  pid->D = D;
  pid->output_ramp = ramp;
  pid->limit = limit;
  pid->error_prev = 0.0f;
  pid->output_prev = 0.0f;
  pid->integral_prev = 0.0f;
  pid->timestamp_prev = HAL_GetTick();
}

float PID_Calc(PID_T* pid, float error) {
  uint64_t timestamp_now = HAL_GetTick();
  float Ts = (timestamp_now - pid->timestamp_prev) * 1e-3f;
  if (Ts <= 0 || Ts > 0.5f) Ts = 1e-3f;

  float proportional = pid->P * error;
  float integral =
      pid->integral_prev + pid->I * Ts * 0.5f * (error + pid->error_prev);
  integral = _constrain(integral, -pid->limit, pid->limit);
  float derivative = pid->D * (error - pid->error_prev) / Ts;

  float output = proportional + integral + derivative;

  output = _constrain(output, -pid->limit, pid->limit);

  if (pid->output_ramp > 0) {
    float output_rate = (output - pid->output_prev) / Ts;
    if (output_rate > pid->output_ramp)
      output = pid->output_prev + pid->output_ramp * Ts;
    else if (output_rate < -pid->output_ramp)
      output = pid->output_prev - pid->output_ramp * Ts;
  }

  pid->integral_prev = integral;
  pid->output_prev = output;
  pid->error_prev = error;
  pid->timestamp_prev = timestamp_now;
  return output;
}

void PID_Reset(PID_T* pid) {
  pid->integral_prev = 0.0f;
  pid->output_prev = 0.0f;
  pid->error_prev = 0.0f;
}