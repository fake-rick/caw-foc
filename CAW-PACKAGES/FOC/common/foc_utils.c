#include "foc_utils.h"

__attribute__((weak)) float _normalizeAngle(float angle) {
  float a = fmod(angle, _2PI);
  return a >= 0 ? a : (a + _2PI);
}

float _electricalAngle(float shaft_angle, int pole_pairs) {
  return (shaft_angle * pole_pairs);
}