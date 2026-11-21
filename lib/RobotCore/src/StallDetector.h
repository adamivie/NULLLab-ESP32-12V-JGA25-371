#pragma once

#include <math.h>

// Flags a wheel that is being driven hard but is not turning (jammed, wheel
// against a wall, wiring fault). Driving a stalled gearmotor heats the motor
// and stresses the gearbox, so the robot stops when this trips.
class StallDetector {
 public:
  StallDetector(float dutyThreshold, float speedThreshold, float timeoutS)
      : dutyThreshold_(dutyThreshold), speedThreshold_(speedThreshold), timeoutS_(timeoutS) {}

  // duty in [-1,1], measuredSpeed normalised. Returns true once stalled.
  bool update(float duty, float measuredSpeed, float dtS) {
    if (fabsf(duty) >= dutyThreshold_ && fabsf(measuredSpeed) < speedThreshold_) {
      elapsedS_ += dtS;
    } else {
      elapsedS_ = 0.0f;
    }
    return stalled();
  }

  bool stalled() const { return elapsedS_ >= timeoutS_; }
  void reset() { elapsedS_ = 0.0f; }

 private:
  float dutyThreshold_;
  float speedThreshold_;
  float timeoutS_;
  float elapsedS_ = 0.0f;
};
