#pragma once

#include <Adafruit_NeoPixel.h>

#include "drive_system.h"

// The board's four WS2812 LEDs (GPIO 16) show the robot state at a glance:
//
//   white            starting up
//   blue, breathing  disarmed, nobody connected
//   blue, solid      disarmed, a phone/browser is connected
//   green            armed - the robot will move
//   yellow, blinking armed but the joystick link dropped (robot is stopping)
//   red, blinking    fault (stalled wheel or hardware error) - press RESET
class StatusLeds {
 public:
  StatusLeds();
  void begin();
  void showStarting();
  void update(const DriveStatus& status, bool clientConnected, uint32_t nowMs);

 private:
  void fill(uint8_t r, uint8_t g, uint8_t b);
  Adafruit_NeoPixel strip_;
  uint32_t lastColor_ = 0xFFFFFFFF;
};
