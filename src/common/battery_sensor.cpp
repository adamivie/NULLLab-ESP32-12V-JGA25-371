#include "battery_sensor.h"

#include <Arduino.h>

#include "robot_config.h"

namespace {
constexpr int kOversample = 16;  // averages ADC noise (ESP32 ADC is noisy)

BatteryConfig makeConfig() {
  BatteryConfig c;
  c.cells = config::kBatteryCells;
  c.cellLowV = config::kCellLowV;
  c.cellCriticalV = config::kCellCriticalV;
  return c;
}

uint32_t readMv(uint8_t pin) {
  uint32_t sum = 0;
  for (int i = 0; i < kOversample; ++i) sum += analogReadMilliVolts(pin);  // eFuse-calibrated
  return (sum + kOversample / 2) / kOversample;
}
}  // namespace

BatterySensor::BatterySensor() : monitor_(makeConfig()) {}

bool BatterySensor::enabled() const { return config::kBatteryMonitorEnabled; }

void BatterySensor::begin() {
  if (!enabled()) return;
  // Voltage: 12.6 V / 18.18 = 0.69 V. 6 dB attenuation reads up to ~1.75 V
  // accurately (~32 V pack), plenty for the board's 16 V limit.
  // Current: a few amps = tens of mV, so 0 dB (most resolution, up to ~0.95 V).
  analogSetPinAttenuation(config::kBatteryVoltagePin, ADC_6db);
  analogSetPinAttenuation(config::kBatteryCurrentPin, ADC_0db);
}

float BatterySensor::packVolts(uint32_t pinMv, float voltsPerVolt) {
  return static_cast<float>(pinMv) * 0.001f * voltsPerVolt;
}

float BatterySensor::packAmps(uint32_t pinMv, float ampsPerVolt, float offsetV) {
  return (static_cast<float>(pinMv) * 0.001f - offsetV) * ampsPerVolt;
}

void BatterySensor::sample(uint32_t nowMs) {
  if (!enabled()) return;
  rawVoltageMv_ = readMv(config::kBatteryVoltagePin);
  rawCurrentMv_ = readMv(config::kBatteryCurrentPin);
  instantVolts_ = packVolts(rawVoltageMv_, config::kBatteryVoltsPerVolt);
  instantAmps_ = packAmps(rawCurrentMv_, config::kBatteryAmpsPerVolt, config::kBatteryCurrentOffsetV);

  const float dtS = started_ ? static_cast<float>(nowMs - lastMs_) * 0.001f : 0.02f;
  started_ = true;
  lastMs_ = nowMs;
  // Clamp: a long gap (e.g. Wi-Fi start-up blocking loop()) must not count
  // as seconds spent below a threshold, nor as a big charge step.
  monitor_.update(instantVolts_, instantAmps_, dtS > 0.2f ? 0.2f : dtS);
}
