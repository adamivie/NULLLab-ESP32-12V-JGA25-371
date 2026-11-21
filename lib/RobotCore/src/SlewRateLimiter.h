#pragma once

// Limits how fast a value may change, with separate rates for speeding up
// (moving away from zero) and slowing down (moving toward zero).
//
// A reversal never happens in one step: the value is first brought to zero
// (at the deceleration rate) and only then accelerates the other way. That
// matters for these gearmotors, whose datasheet warns against instantaneous
// load reversal.
class SlewRateLimiter {
 public:
  // Rates are in "units per second" and must be > 0.
  SlewRateLimiter(float accelPerS, float decelPerS)
      : accel_(accelPerS), decel_(decelPerS) {}

  void setRates(float accelPerS, float decelPerS) {
    accel_ = accelPerS;
    decel_ = decelPerS;
  }

  float update(float target, float dtS) {
    if (dtS <= 0.0f) return value_;

    // Heading toward zero (or across it) uses the deceleration rate.
    const bool sameSignOrZero = (value_ >= 0.0f && target >= value_) || (value_ <= 0.0f && target <= value_);
    const float rate = sameSignOrZero ? accel_ : decel_;
    const float maxStep = rate * dtS;

    float delta = target - value_;
    if (delta > maxStep) delta = maxStep;
    if (delta < -maxStep) delta = -maxStep;

    float next = value_ + delta;
    // Stop at zero when crossing it; the next update accelerates the other way.
    if ((value_ > 0.0f && next < 0.0f) || (value_ < 0.0f && next > 0.0f)) next = 0.0f;

    value_ = next;
    return value_;
  }

  void reset(float value = 0.0f) { value_ = value; }
  float value() const { return value_; }

 private:
  float accel_;
  float decel_;
  float value_ = 0.0f;
};
