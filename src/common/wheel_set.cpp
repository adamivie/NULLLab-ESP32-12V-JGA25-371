#include "wheel_set.h"

#include <Arduino.h>

static_assert(config::kWheelCount == 4, "WheelSet is written for four wheels");

WheelSet::WheelSet()
    : motors_{{config::kWheels[0].motorIn1, config::kWheels[0].motorIn2},
              {config::kWheels[1].motorIn1, config::kWheels[1].motorIn2},
              {config::kWheels[2].motorIn1, config::kWheels[2].motorIn2},
              {config::kWheels[3].motorIn1, config::kWheels[3].motorIn2}},
      encoders_{{config::kWheels[0].encoderA, config::kWheels[0].encoderB},
                {config::kWheels[1].encoderA, config::kWheels[1].encoderB},
                {config::kWheels[2].encoderA, config::kWheels[2].encoderB},
                {config::kWheels[3].encoderA, config::kWheels[3].encoderB}} {}

bool WheelSet::begin() {
  bool ok = true;
  for (int i = 0; i < config::kWheelCount; ++i) {
    const config::WheelHw& w = config::kWheels[i];
    if (!motors_[i].begin(config::kPwmFrequencyHz, config::kPwmResolutionBits)) {
      Serial.printf("[hw] %s: motor PWM setup FAILED (GPIO %u/%u)\n", w.name, w.motorIn1, w.motorIn2);
      ok = false;
    }
    if (!encoders_[i].begin(config::kEncoderGlitchFilterNs)) {
      Serial.printf("[hw] %s: encoder setup FAILED (GPIO %u/%u)\n", w.name, w.encoderA, w.encoderB);
      ok = false;
    }
  }
  applyConfiguredInversion(true);
  brakeAll();
  return ok;
}

void WheelSet::applyConfiguredInversion(bool apply) {
  for (int i = 0; i < config::kWheelCount; ++i) {
    motors_[i].setInverted(apply && config::kWheels[i].invertMotor);
    encoders_[i].setInverted(apply && config::kWheels[i].invertEncoder);
  }
}

void WheelSet::brakeAll() {
  for (auto& m : motors_) m.brake();
}

void WheelSet::coastAll() {
  for (auto& m : motors_) m.coast();
}
