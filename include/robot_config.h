#pragma once
// =============================================================================
//  Robot configuration - the ONLY file you should need to edit.
//
//  Board : NULLLAB Maker-ESP32 v1.7x (ESP32-WROOM-32E, 4x TB67H450FNG drivers)
//  Motors: 4x JGA25-371 12 V, ~500 RPM (1:9.6), 11-pulse Hall encoder
//  Wheels: 65 mm, skid steer (4WD)
//
//  Values marked CALIBRATE come from the bring-up firmware (`pio run -e
//  bringup -t upload`, then open the serial monitor and type `help`).
// =============================================================================

#include <stdint.h>

#include "WheelSpeedController.h"

namespace config {

// -----------------------------------------------------------------------------
// Wheels. Order = motor port M1..M4 on the board. Motor pins are fixed by the
// board; encoder pins are where YOU wire the encoder's yellow (A) and green (B)
// wires (see docs/WIRING.md).
//
//  invertMotor   : set by the bring-up `dir` wizard so +duty drives the
//                  robot forward
//  invertEncoder : set by the same wizard so forward motion counts up
//  isLeft        : which side of the robot the wheel is on
// -----------------------------------------------------------------------------
struct WheelHw {
  const char* name;
  uint8_t motorIn1;
  uint8_t motorIn2;
  uint8_t encoderA;
  uint8_t encoderB;
  bool invertMotor;
  bool invertEncoder;
  bool isLeft;
};

constexpr int kWheelCount = 4;

// CALIBRATE: replace this block with the one printed by the `dir` wizard.
constexpr WheelHw kWheels[kWheelCount] = {
    //  name             IN1 IN2  encA encB  invMotor invEnc  isLeft
    {"M1 front-left",   27, 13,   34,  18,   false,   false,  true},
    {"M2 rear-left",     4,  2,   35,  19,   false,   false,  true},
    {"M3 front-right",  17, 12,   36,  23,   true,    false,  false},
    {"M4 rear-right",   15, 14,   39,   5,   true,    false,  false},
};

// -----------------------------------------------------------------------------
// Drivetrain
// -----------------------------------------------------------------------------
constexpr float kWheelDiameterM = 0.065f;

// Encoder counts per WHEEL revolution in 4x mode:
//   11 pulses x 4 edges x 9.6 gear ratio = 422.4 (nominal)
// CALIBRATE with the bring-up `cal` command.
constexpr float kCountsPerWheelRev = 422.4f;

// Wheel speed (counts/s) that corresponds to "100 %" in the controller. Use
// the value suggested by the bring-up `maxspeed` command (a little below the
// slowest wheel's full-duty speed, so every wheel can actually reach it).
// Nominal: ~620 rpm no-load at 12 V -> 620/60*422.4 = ~4360; default leaves headroom.
// CALIBRATE.
constexpr float kMaxWheelSpeedCps = 3900.0f;

// Speed-controller gains (normalised units, see WheelSpeedController.h).
// kS CALIBRATE with the bring-up `friction` command; the rest with `step`.
inline WheelGains wheelGains() {
  WheelGains g;
  g.kP = 0.5f;
  g.kI = 6.0f;
  g.kF = 1.0f;
  g.kS = 0.05f;
  g.iLimit = 0.5f;
  g.refTauS = 0.12f;
  return g;
}

// -----------------------------------------------------------------------------
// Driving feel
// -----------------------------------------------------------------------------
constexpr float kDefaultSpeedLimit = 0.5f;  // fraction of top speed at start-up
constexpr float kStickDeadzone = 0.06f;
constexpr float kStickExpo = 0.35f;          // 0 = linear, 1 = very soft centre
constexpr float kTurnGain = 0.65f;           // turning authority vs throttle
constexpr float kAccelPerS = 1.5f;           // 0 -> full speed in 0.67 s
constexpr float kDecelPerS = 3.0f;           // full speed -> 0 in 0.33 s

// -----------------------------------------------------------------------------
// Safety
// -----------------------------------------------------------------------------
// No joystick message for this long -> stop (browser sends ~20 per second).
constexpr uint32_t kCommandTimeoutMs = 500;
// Link lost for this long -> disarm completely.
constexpr uint32_t kLinkLostDisarmMs = 3000;
// Stall = duty >= kStallDuty while speed < kStallSpeed for kStallTimeS.
constexpr float kStallDuty = 0.6f;
constexpr float kStallSpeed = 0.04f;
constexpr float kStallTimeS = 1.0f;

// -----------------------------------------------------------------------------
// Low-level hardware
// -----------------------------------------------------------------------------
constexpr uint32_t kControlPeriodMs = 10;        // 100 Hz speed loop
constexpr uint32_t kPwmFrequencyHz = 20000;      // inaudible
constexpr uint8_t kPwmResolutionBits = 10;
constexpr uint32_t kEncoderGlitchFilterNs = 2000;  // edges at top speed are ~200 us apart
constexpr uint8_t kRgbLedPin = 16;
constexpr uint8_t kRgbLedCount = 4;
constexpr uint8_t kRgbLedBrightness = 40;  // 0-255

// -----------------------------------------------------------------------------
// Wi-Fi. By default the robot creates its own network (works anywhere):
//   SSID "MakerBot-XXXX" (last 4 hex digits of its MAC), password below,
//   then browse to http://192.168.4.1
// To join your home Wi-Fi instead, copy include/wifi_secrets.example.h to
// include/wifi_secrets.h (git-ignored) and fill it in; the robot falls back
// to its own network if it cannot connect.
// -----------------------------------------------------------------------------
constexpr const char* kApSsidPrefix = "MakerBot-";
constexpr const char* kApPassword = "makerbot123";  // >= 8 chars; CHANGE ME
constexpr uint8_t kApChannel = 6;
constexpr const char* kHostname = "makerbot";       // http://makerbot.local
constexpr uint32_t kStationConnectTimeoutMs = 12000;

// Derived values -------------------------------------------------------------
constexpr float kPi = 3.14159265358979f;
constexpr float kMetersPerCount = kPi * kWheelDiameterM / kCountsPerWheelRev;
constexpr float kMaxWheelSpeedMps = kMaxWheelSpeedCps * kMetersPerCount;

}  // namespace config
