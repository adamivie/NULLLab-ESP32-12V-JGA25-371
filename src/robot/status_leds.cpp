#include "status_leds.h"

#include <Preferences.h>
#include <math.h>

#include "robot_config.h"

namespace {
constexpr char kPrefsNamespace[] = "leds";
constexpr char kPrefsKey[] = "cfg";
constexpr uint32_t kSaveDelayMs = 2000;  // colour pickers send many changes; save once they settle
constexpr float kMovingThreshold = 0.03f;  // fraction of top speed
}  // namespace

StatusLeds::StatusLeds() : strip_(config::kRgbLedCount, config::kRgbLedPin, NEO_GRB + NEO_KHZ800) {}

void StatusLeds::begin() {
  strip_.begin();
  strip_.clear();
  strip_.show();

  config_.brightness = config::kRgbLedBrightness;
  Preferences prefs;
  if (prefs.begin(kPrefsNamespace, false)) {
    uint32_t fields[kLedConfigFields] = {};
    if (prefs.isKey(kPrefsKey) && prefs.getBytes(kPrefsKey, fields, sizeof(fields)) == sizeof(fields)) {
      ledConfigFromFields(fields, &config_);
    }
    prefs.end();
  }
}

void StatusLeds::setConfig(const LedConfig& config) {
  portENTER_CRITICAL(&lock_);
  config_ = config;
  unsaved_ = true;
  changedMs_ = millis();
  portEXIT_CRITICAL(&lock_);
}

LedConfig StatusLeds::config() const {
  portENTER_CRITICAL(&lock_);
  const LedConfig copy = config_;
  portEXIT_CRITICAL(&lock_);
  return copy;
}

void StatusLeds::showStarting() { fill(255, 255, 255); }

void StatusLeds::show(const LedRgb ring[kLedRingSize], uint8_t brightness) {
  bool changed = false;
  for (int led = 0; led < config::kRgbLedCount && led < kLedRingSize; ++led) {
    const LedRgb& c = ring[config::kLedRingPosition[led] % kLedRingSize];
    const uint32_t color = Adafruit_NeoPixel::Color(static_cast<uint8_t>(c.r * brightness / 255),
                                                    static_cast<uint8_t>(c.g * brightness / 255),
                                                    static_cast<uint8_t>(c.b * brightness / 255));
    if (color != lastFrame_[led]) {
      lastFrame_[led] = color;
      strip_.setPixelColor(led, color);
      changed = true;
    }
  }
  if (changed) strip_.show();  // avoid needless writes
}

void StatusLeds::fill(uint8_t r, uint8_t g, uint8_t b) {
  const LedRgb ring[kLedRingSize] = {{r, g, b}, {r, g, b}, {r, g, b}, {r, g, b}};
  show(ring, config::kRgbLedBrightness);
}

void StatusLeds::drawStatus(const DriveStatus& s, bool clientConnected, uint32_t nowMs) {
  const bool blinkOn = (nowMs / 250) % 2 == 0;
  if (s.state == DriveState::Fault || !s.hardwareOk) {
    blinkOn ? fill(255, 0, 0) : fill(0, 0, 0);
  } else if ((s.state == DriveState::Armed || s.state == DriveState::Stopping) && !s.linkOk) {
    blinkOn ? fill(255, 160, 0) : fill(0, 0, 0);
  } else if (s.state == DriveState::Armed || s.state == DriveState::Stopping) {
    fill(0, 255, 0);
  } else if (clientConnected) {
    fill(0, 60, 255);
  } else {
    // Slow "breathing" blue: 3 s period.
    const float phase = static_cast<float>(nowMs % 3000) / 3000.0f;
    const float level = 0.15f + 0.85f * (0.5f - 0.5f * cosf(phase * 2.0f * 3.14159265f));
    fill(0, static_cast<uint8_t>(15 * level), static_cast<uint8_t>(255 * level));
  }
}

void StatusLeds::update(const DriveStatus& s, bool clientConnected, uint32_t nowMs) {
  const float dtS = fminf(static_cast<float>(nowMs - lastUpdateMs_) * 0.001f, 0.1f);
  lastUpdateMs_ = nowMs;

  portENTER_CRITICAL(&lock_);
  const LedConfig cfg = config_;
  const bool save = unsaved_ && nowMs - changedMs_ >= kSaveDelayMs;
  if (save) unsaved_ = false;
  portEXIT_CRITICAL(&lock_);

  if (save) {
    uint32_t fields[kLedConfigFields];
    ledConfigToFields(cfg, fields);
    Preferences prefs;
    if (prefs.begin(kPrefsNamespace, false)) {
      prefs.putBytes(kPrefsKey, fields, sizeof(fields));
      prefs.end();
    }
  }

  // Warnings always win over a light effect.
  const bool driving = s.state == DriveState::Armed || s.state == DriveState::Stopping;
  const bool warning = s.state == DriveState::Fault || !s.hardwareOk || (driving && !s.linkOk);
  if (cfg.mode == LedMode::Status || warning) {
    drawStatus(s, clientConnected, nowMs);
    return;
  }

  // Measured motion from the wheels: left and right side averages.
  float left = 0.0f, right = 0.0f;
  int leftCount = 0, rightCount = 0;
  for (int i = 0; i < config::kWheelCount; ++i) {
    if (config::kWheels[i].isLeft) {
      left += s.wheels[i].speedMps;
      ++leftCount;
    } else {
      right += s.wheels[i].speedMps;
      ++rightCount;
    }
  }
  if (leftCount > 0) left /= leftCount * config::kMaxWheelSpeedMps;
  if (rightCount > 0) right /= rightCount * config::kMaxWheelSpeedMps;
  LedMotion motion;
  motion.forward = (left + right) * 0.5f;
  motion.turn = (left - right) * 0.5f;  // left side faster = turning right
  motion.moving = fabsf(motion.forward) > kMovingThreshold || fabsf(motion.turn) > kMovingThreshold;

  clock_.advance(cfg, motion, dtS);
  LedRgb ring[kLedRingSize];
  renderLeds(cfg, motion, clock_, ring);
  show(ring, cfg.brightness);
}
