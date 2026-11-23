#pragma once

#include <Adafruit_NeoPixel.h>

#include "LedEffects.h"
#include "drive_system.h"

// The board's four WS2812 LEDs (GPIO 16).
//
// In the default "Status" mode they show the robot state at a glance:
//
//   white            starting up
//   blue, breathing  disarmed, nobody connected
//   blue, solid      disarmed, a phone/browser is connected
//   green            armed - the robot will move
//   yellow, blinking armed but the joystick link dropped (robot is stopping)
//   red, blinking    fault (stalled wheel or hardware error) - press RESET
//
// The web page can pick a light effect instead (pulse, spin, front/back
// colours, palette sweep, rainbow; see LedEffects.h). The yellow and red
// warnings always win over an effect. The chosen effect is saved in flash.
class StatusLeds {
 public:
  StatusLeds();
  void begin();  // also loads the saved effect
  void showStarting();
  void update(const DriveStatus& status, bool clientConnected, uint32_t nowMs);

  // Safe to call from any task. Saved to flash a moment after the last change.
  void setConfig(const LedConfig& config);
  LedConfig config() const;

 private:
  void show(const LedRgb ring[kLedRingSize], uint8_t brightness);
  void fill(uint8_t r, uint8_t g, uint8_t b);
  void drawStatus(const DriveStatus& status, bool clientConnected, uint32_t nowMs);

  Adafruit_NeoPixel strip_;
  mutable portMUX_TYPE lock_ = portMUX_INITIALIZER_UNLOCKED;
  LedConfig config_;
  LedClock clock_;
  uint32_t lastFrame_[kLedRingSize] = {0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF, 0xFFFFFFFF};
  uint32_t lastUpdateMs_ = 0;
  uint32_t changedMs_ = 0;
  bool unsaved_ = false;
};
