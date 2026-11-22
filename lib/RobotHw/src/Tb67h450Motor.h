#pragma once

#include <stdint.h>

// One DC motor on a Toshiba TB67H450FNG H-bridge (one chip per motor port on
// the Maker-ESP32).
//
// TB67H450FNG truth table:
//   IN1 IN2   OUT1 OUT2
//    H   L     H    L     forward
//    L   H     L    H     reverse
//    H   H     L    L     brake (short)
//    L   L     Z    Z     coast; after ~1 ms the chip enters standby
//
// Speed is set with sign-magnitude PWM using *brake* decay: one input is held
// high and the other is pulsed, so the bridge alternates drive <-> brake.
// Compared with drive <-> coast this gives speed nearly proportional to duty,
// works well at low speed, and never lets the chip drop into standby while
// running. The chip has its own ~3 A current limit (VREF divider and 0.15 ohm
// sense resistor on the board) and thermal shutdown.
class Tb67h450Motor {
 public:
  Tb67h450Motor(uint8_t pinIn1, uint8_t pinIn2) : pinIn1_(pinIn1), pinIn2_(pinIn2) {}

  // Attaches both pins to LEDC PWM. Returns false on failure.
  // 20 kHz is above hearing and well within the chip's input rating.
  bool begin(uint32_t pwmFrequencyHz = 20000, uint8_t resolutionBits = 10);

  // duty in [-1, 1]; positive = "forward" after inversion. 0 = brake.
  void setDuty(float duty);
  void brake();
  void coast();

  // Flip the meaning of forward (motor wired or mounted the other way round).
  void setInverted(bool inverted) { inverted_ = inverted; }
  bool inverted() const { return inverted_; }

  float duty() const { return duty_; }
  bool ready() const { return ready_; }

 private:
  void writeRaw(uint32_t in1, uint32_t in2);

  uint8_t pinIn1_;
  uint8_t pinIn2_;
  uint32_t maxDuty_ = 0;
  bool inverted_ = false;
  bool ready_ = false;
  float duty_ = 0.0f;
};
