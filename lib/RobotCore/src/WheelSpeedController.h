#pragma once

// Closed-loop speed controller for one wheel.
//
// All speeds are normalised: 1.0 = the wheel's measured no-load top speed at
// full duty (config::kMaxWheelSpeedCps). The output is a motor duty in [-1, 1].
//
// Two-degree-of-freedom structure:
//
//   duty = kF*target + kS*sign(target)        feed-forward (does most of the work)
//        + kP*e + integral(kI*e)              feedback
//   e    = reference - measured
//   reference = target passed through a first-order lag of refTauS
//
// The reference model is what the wheel *should* be doing given its own
// inertia and the measurement lag. Comparing against it (instead of the raw
// target) means the feedback only reacts to real disturbances such as load,
// friction and battery sag, not to the motor's normal response time. Without
// it the integrator winds up during every acceleration and the wheel
// overshoots when the ramp ends.
//
// The integral also uses conditional integration (it stops growing while the
// output is saturated in the direction of the error), which prevents wind-up
// when a wheel is held or the battery is low.
//
// Defaults were chosen by simulating a JGA25-371-like motor including encoder
// quantisation, the velocity-estimator lag and one period of actuation delay
// (see host_tests). Re-tune on the real robot with the bring-up `step` test.
struct WheelGains {
  float kP = 0.5f;       // duty per unit speed error
  float kI = 6.0f;       // duty per unit speed error per second
  float kF = 1.0f;       // duty per unit target speed
  float kS = 0.05f;      // static-friction duty added in the direction of travel
  float iLimit = 0.5f;   // |integral| clamp, in duty
  float refTauS = 0.12f; // reference-model time constant (motor + measurement lag)
};

class WheelSpeedController {
 public:
  explicit WheelSpeedController(const WheelGains& gains = WheelGains()) : gains_(gains) {}

  void setGains(const WheelGains& gains) { gains_ = gains; }
  const WheelGains& gains() const { return gains_; }

  // A target smaller than this is treated as "stop": output 0 (brake) and the
  // controller state is cleared so the wheel does not creep or hum at standstill.
  static constexpr float kStopThreshold = 1.0e-3f;

  float update(float target, float measured, float dtS);
  void reset() {
    integral_ = 0.0f;
    reference_ = 0.0f;
  }
  float integral() const { return integral_; }
  float reference() const { return reference_; }

 private:
  WheelGains gains_;
  float integral_ = 0.0f;
  float reference_ = 0.0f;
};
