#pragma once

#include "QuadratureEncoder.h"
#include "Tb67h450Motor.h"
#include "robot_config.h"

// The four motor + encoder pairs, built from config::kWheels.
class WheelSet {
 public:
  WheelSet();

  // Starts PWM and the pulse counters. Returns false if any wheel failed
  // (details are printed to Serial). Motors are left braked.
  bool begin();

  // true  = use invertMotor / invertEncoder from config (normal operation)
  // false = raw hardware directions (used by the bring-up direction wizard)
  void applyConfiguredInversion(bool apply);

  Tb67h450Motor& motor(int index) { return motors_[index]; }
  QuadratureEncoder& encoder(int index) { return encoders_[index]; }
  const config::WheelHw& hw(int index) const { return config::kWheels[index]; }

  void brakeAll();
  void coastAll();

 private:
  Tb67h450Motor motors_[config::kWheelCount];
  QuadratureEncoder encoders_[config::kWheelCount];
};
