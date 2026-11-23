#pragma once

#include <stdint.h>

// Light effects for the robot's four RGB LEDs. Pure maths (no hardware), so it
// can be unit-tested on a PC; src/robot/status_leds.cpp feeds it and drives
// the real LEDs.

enum class LedMode : uint8_t {
  Status = 0,     // robot state colours (white/blue/green/yellow/red)
  Solid,          // colour 1
  Pulse,          // colour 1 fades in and out
  Blink,          // colour 1 / colour 2 alternate
  SpinFade,       // a comet of colour 1 runs round the robot, colour 2 behind it
  SpinBlink,      // one LED of colour 1 steps round the robot
  FrontBack,      // front = colour 1, back = colour 2; the leading end is brighter
  PaletteSweep,   // sweeps through the chosen palette
  Rainbow,        // rainbow running round the robot
  Count
};

enum class LedPalette : uint8_t { Fire = 0, Ocean, Forest, Sunset, Party, Ice, Custom, Count };

struct LedConfig {
  LedMode mode = LedMode::Status;
  uint32_t color1 = 0x00FF40;  // 0xRRGGBB
  uint32_t color2 = 0x000000;
  LedPalette palette = LedPalette::Party;
  uint8_t speed = 50;        // 1..100
  uint8_t brightness = 40;   // 1..255
  bool movingOnly = true;    // animate only while the robot moves
};

constexpr int kLedConfigFields = 7;
// Field order used on the wire and in storage:
//   mode, color1, color2, palette, speed, brightness, movingOnly
// Returns false (and leaves *out alone) if any value is out of range.
bool ledConfigFromFields(const uint32_t fields[kLedConfigFields], LedConfig* out);
void ledConfigToFields(const LedConfig& config, uint32_t fields[kLedConfigFields]);

// What the effect needs to know about the robot right now.
struct LedMotion {
  float forward = 0.0f;  // -1..1, + = driving forward
  float turn = 0.0f;     // -1..1, + = turning right (clockwise seen from above)
  bool moving = false;
};

struct LedRgb {
  uint8_t r = 0, g = 0, b = 0;
};

constexpr int kLedRingSize = 4;

// Animation time. Advance once per frame; the phase only runs while the
// effect is "active" so animations freeze cleanly instead of jumping.
struct LedClock {
  float phase = 0.0f;  // cycles, wraps at 1000
  void advance(const LedConfig& config, const LedMotion& motion, float dtS);
};

// Renders one frame. out[] is indexed by ring position, clockwise seen from
// above: 0 = front-left, 1 = front-right, 2 = rear-right, 3 = rear-left.
// Not used for LedMode::Status (the caller draws that).
void renderLeds(const LedConfig& config, const LedMotion& motion, const LedClock& clock, LedRgb out[kLedRingSize]);
