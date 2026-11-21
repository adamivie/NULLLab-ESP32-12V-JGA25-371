#include "SkidSteer.h"

#include <math.h>

namespace {
float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }
}  // namespace

float shapeAxis(float value, float deadzone, float expo) {
  if (!isfinite(value)) return 0.0f;
  value = clampf(value, -1.0f, 1.0f);
  deadzone = clampf(deadzone, 0.0f, 0.95f);
  expo = clampf(expo, 0.0f, 1.0f);

  float magnitude = fabsf(value);
  if (magnitude <= deadzone) return 0.0f;
  magnitude = (magnitude - deadzone) / (1.0f - deadzone);
  magnitude = (1.0f - expo) * magnitude + expo * magnitude * magnitude * magnitude;
  return value < 0.0f ? -magnitude : magnitude;
}

SideSpeeds mixArcade(float throttle, float turn, float turnGain) {
  if (!isfinite(throttle)) throttle = 0.0f;
  if (!isfinite(turn)) turn = 0.0f;
  throttle = clampf(throttle, -1.0f, 1.0f);
  turn = clampf(turn, -1.0f, 1.0f);

  float left = throttle + turn * turnGain;
  float right = throttle - turn * turnGain;
  const float largest = fmaxf(fabsf(left), fabsf(right));
  if (largest > 1.0f) {
    left /= largest;
    right /= largest;
  }
  return {left, right};
}
