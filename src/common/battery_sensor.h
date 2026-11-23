#pragma once

#include <stdint.h>

#include "BatteryMonitor.h"

// Reads the PM02 power module's two analog outputs and feeds BatteryMonitor.
// Call sample() regularly (every 20 ms) from ONE task; not thread-safe.
class BatterySensor {
 public:
  BatterySensor();

  void begin();
  // Takes one oversampled reading and updates the monitor.
  void sample(uint32_t nowMs);

  const BatteryMonitor& monitor() const { return monitor_; }
  bool enabled() const;

  // Last raw readings (ADC pin millivolts, averaged) and the instantaneous
  // pack volts/amps computed from them - for the bring-up `batt` command.
  uint32_t rawVoltageMv() const { return rawVoltageMv_; }
  uint32_t rawCurrentMv() const { return rawCurrentMv_; }
  float instantVolts() const { return instantVolts_; }
  float instantAmps() const { return instantAmps_; }

  // Converts ADC pin millivolts to pack volts / amps with the config scales.
  static float packVolts(uint32_t pinMv, float voltsPerVolt);
  static float packAmps(uint32_t pinMv, float ampsPerVolt, float offsetV);

 private:
  BatteryMonitor monitor_;
  uint32_t lastMs_ = 0;
  bool started_ = false;
  uint32_t rawVoltageMv_ = 0;
  uint32_t rawCurrentMv_ = 0;
  float instantVolts_ = 0.0f;
  float instantAmps_ = 0.0f;
};
