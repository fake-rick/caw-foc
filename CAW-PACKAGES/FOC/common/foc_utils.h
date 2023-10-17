#ifndef __FOC_UTILS_H__
#define __FOC_UTILS_H__

#define _constrain(amt, low, high) \
  ((amt) < (low) ? (low) : ((amt) > (high) ? (high) : (amt)))

// utility defines
#define _2_SQRT3 1.15470053838f
#define _SQRT3 1.73205080757f
#define _1_SQRT3 0.57735026919f
#define _SQRT3_2 0.86602540378f
#define _SQRT2 1.41421356237f
#define _120_D2R 2.09439510239f
#define _PI 3.14159265359f
#define _PI_2 1.57079632679f
#define _PI_3 1.0471975512f
#define _2PI 6.28318530718f
#define _3PI_2 4.71238898038f
#define _PI_6 0.52359877559f
#define _RPM_TO_RADS 0.10471975512f

typedef struct _DQ_CURRENT_T {
  float d;
  float q;
} DQ_CURRENT_T;

typedef struct _PHASE_CURRENT_T {
  float a;
  float b;
  float c;
} PHASE_CURRENT_T;

typedef struct _DQ_VOLTAGE_T {
  float d;
  float q;
} DQ_VOLTAGE_T;

// normalizing radian angle to [0,2PI]
__attribute__((weak)) float _normalizeAngle(float angle);

// Electrical angle calculation
float _electricalAngle(float shaft_angle, int pole_pairs);

#endif