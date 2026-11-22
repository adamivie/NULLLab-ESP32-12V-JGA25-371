#include "Tb67h450Motor.h"

#include <Arduino.h>
#include <math.h>

bool Tb67h450Motor::begin(uint32_t pwmFrequencyHz, uint8_t resolutionBits) {
  // Drive both inputs low (coast) before PWM takes over so the motor cannot
  // twitch during start-up.
  pinMode(pinIn1_, OUTPUT);
  pinMode(pinIn2_, OUTPUT);
  digitalWrite(pinIn1_, LOW);
  digitalWrite(pinIn2_, LOW);

  if (!ledcAttach(pinIn1_, pwmFrequencyHz, resolutionBits)) return false;
  if (!ledcAttach(pinIn2_, pwmFrequencyHz, resolutionBits)) {
    ledcDetach(pinIn1_);
    return false;
  }
  maxDuty_ = (1u << resolutionBits) - 1u;
  ready_ = true;
  coast();
  return true;
}

void Tb67h450Motor::writeRaw(uint32_t in1, uint32_t in2) {
  if (!ready_) return;
  ledcWrite(pinIn1_, in1);
  ledcWrite(pinIn2_, in2);
}

void Tb67h450Motor::setDuty(float duty) {
  if (!isfinite(duty)) duty = 0.0f;
  if (duty > 1.0f) duty = 1.0f;
  if (duty < -1.0f) duty = -1.0f;
  duty_ = duty;
  if (inverted_) duty = -duty;

  const uint32_t magnitude = static_cast<uint32_t>(lroundf(fabsf(duty) * static_cast<float>(maxDuty_)));
  if (magnitude == 0) {
    brake();
    return;
  }
  // The pulsed input is HIGH (= brake) for the off-part of the cycle, so its
  // duty is the complement of the drive fraction.
  const uint32_t pulsed = maxDuty_ - magnitude;
  if (duty > 0.0f) {
    writeRaw(maxDuty_, pulsed);  // IN1 high, IN2 low while driving
  } else {
    writeRaw(pulsed, maxDuty_);  // IN2 high, IN1 low while driving
  }
}

void Tb67h450Motor::brake() {
  duty_ = 0.0f;
  writeRaw(maxDuty_, maxDuty_);
}

void Tb67h450Motor::coast() {
  duty_ = 0.0f;
  writeRaw(0, 0);
}
