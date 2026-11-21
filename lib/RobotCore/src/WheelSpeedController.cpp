#include "WheelSpeedController.h"

#include <math.h>

namespace {
float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }
}  // namespace

float WheelSpeedController::update(float target, float measured, float dtS) {
  if (!isfinite(target) || !isfinite(measured) || fabsf(target) < kStopThreshold) {
    reset();
    return 0.0f;
  }
  if (!(dtS > 0.0f)) dtS = 0.0f;

  // Advance the reference model (exact discretisation of a first-order lag).
  if (gains_.refTauS > 0.0f) {
    reference_ += (target - reference_) * (1.0f - expf(-dtS / gains_.refTauS));
  } else {
    reference_ = target;
  }

  const float error = reference_ - measured;
  const float feedForward = gains_.kF * target + (target > 0.0f ? gains_.kS : -gains_.kS);
  const float proportional = gains_.kP * error;

  const float candidate = clampf(integral_ + gains_.kI * error * dtS, -gains_.iLimit, gains_.iLimit);
  const float unclamped = feedForward + proportional + candidate;
  const bool saturatedSameWay = (unclamped > 1.0f && error > 0.0f) || (unclamped < -1.0f && error < 0.0f);
  if (!saturatedSameWay) integral_ = candidate;

  float duty = clampf(feedForward + proportional + integral_, -1.0f, 1.0f);
  // Never command the wheel backwards to slow it down: plain braking (duty 0
  // in brake-decay PWM) is gentler on the gearbox than reverse drive.
  if (target > 0.0f && duty < 0.0f) duty = 0.0f;
  if (target < 0.0f && duty > 0.0f) duty = 0.0f;
  return duty;
}
