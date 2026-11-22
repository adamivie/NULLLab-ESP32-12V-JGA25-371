#pragma once

#include <stdint.h>

#include "driver/pulse_cnt.h"

// Hall-effect quadrature encoder counted entirely in the ESP32's PCNT
// hardware, in 4x mode (every edge of A and B), so no counts are lost to
// interrupt latency however busy the CPU or Wi-Fi is.
//
// One encoder uses one PCNT unit; the ESP32 has 8, and four wheels use four.
// The hardware counter is 16-bit; the driver accumulates overflows, and this
// class extends that again to 64 bits so the count never wraps in practice.
//
// Pull-ups: the driver enables the internal pull-up on each pin that has one.
// GPIO 34-39 have none, so any encoder on those pins must drive its outputs
// actively or have pull-up resistors on the encoder board.
class QuadratureEncoder {
 public:
  QuadratureEncoder(int pinA, int pinB) : pinA_(pinA), pinB_(pinB) {}
  ~QuadratureEncoder();
  QuadratureEncoder(const QuadratureEncoder&) = delete;
  QuadratureEncoder& operator=(const QuadratureEncoder&) = delete;

  // Pulses shorter than glitchFilterNs are ignored (motor noise). The ESP32
  // hardware supports up to ~12700 ns. Returns false on failure.
  bool begin(uint32_t glitchFilterNs = 1000);

  // Signed count, positive = forward after inversion.
  int64_t count();
  void reset();

  void setInverted(bool inverted) { inverted_ = inverted; }
  bool inverted() const { return inverted_; }
  bool ready() const { return unit_ != nullptr; }

 private:
  void release();

  int pinA_;
  int pinB_;
  bool inverted_ = false;
  pcnt_unit_handle_t unit_ = nullptr;
  pcnt_channel_handle_t channelA_ = nullptr;
  pcnt_channel_handle_t channelB_ = nullptr;
  int lastRaw_ = 0;
  int64_t accumulated_ = 0;
};
