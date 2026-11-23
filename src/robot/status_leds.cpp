#include "status_leds.h"

#include <math.h>

#include "robot_config.h"

StatusLeds::StatusLeds() : strip_(config::kRgbLedCount, config::kRgbLedPin, NEO_GRB + NEO_KHZ800) {}

void StatusLeds::begin() {
  strip_.begin();
  strip_.setBrightness(config::kRgbLedBrightness);
  strip_.clear();
  strip_.show();
}

void StatusLeds::showStarting() { fill(255, 255, 255); }

void StatusLeds::fill(uint8_t r, uint8_t g, uint8_t b) {
  const uint32_t color = Adafruit_NeoPixel::Color(r, g, b);
  if (color == lastColor_) return;  // avoid needless writes
  lastColor_ = color;
  strip_.fill(color);
  strip_.show();
}

void StatusLeds::update(const DriveStatus& s, bool clientConnected, BatteryLevel battery, uint32_t nowMs) {
  const bool blinkOn = (nowMs / 250) % 2 == 0;
  const bool armed = s.state == DriveState::Armed || s.state == DriveState::Stopping;
  if (s.state == DriveState::Fault || !s.hardwareOk) {
    blinkOn ? fill(255, 0, 0) : fill(0, 0, 0);
  } else if (battery == BatteryLevel::Critical && !armed) {
    // Slow red pulse (2 s period), distinct from the fast fault blink.
    const float phase = static_cast<float>(nowMs % 2000) / 2000.0f;
    fill(static_cast<uint8_t>(255 * (0.1f + 0.9f * (0.5f - 0.5f * cosf(phase * 2.0f * 3.14159265f)))), 0, 0);
  } else if (armed && !s.linkOk) {
    blinkOn ? fill(255, 160, 0) : fill(0, 0, 0);
  } else if (armed) {
    fill(0, 255, 0);
  } else if (battery == BatteryLevel::Low) {
    fill(255, 110, 0);
  } else if (clientConnected) {
    fill(0, 60, 255);
  } else {
    // Slow "breathing" blue: 3 s period, quantised so we only redraw ~25x/s.
    const float phase = static_cast<float>(nowMs % 3000) / 3000.0f;
    const float level = 0.15f + 0.85f * (0.5f - 0.5f * cosf(phase * 2.0f * 3.14159265f));
    fill(0, static_cast<uint8_t>(15 * level), static_cast<uint8_t>(255 * level));
  }
}
