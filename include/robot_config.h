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

// Physical layout (top view, front at the top):
//
//        FRONT
//     M2      M4
//     M1      M3
//        REAR
//
// Printed by the bring-up `dir` wizard on this robot; re-run it after rewiring.
constexpr WheelHw kWheels[kWheelCount] = {
    //  name             IN1 IN2  encA encB  invMotor invEnc  isLeft
    {"M1 rear-left",    27, 13,   19,  23,   false ,  false , true},
    {"M2 front-left",    4,  2,    5,  18,   true  ,  true  , true},
    {"M3 rear-right",   17, 12,   33,  32,   true  ,  true  , false},
    {"M4 front-right",  15, 14,   26,  25,   true  ,  true  , false},
};

// -----------------------------------------------------------------------------
// Drivetrain
// -----------------------------------------------------------------------------
constexpr float kWheelDiameterM = 0.065f;

// Encoder counts per WHEEL revolution in 4x mode, one value per port M1..M4:
//   11 pulses x 4 edges x gear ratio (e.g. 9.6 -> 422.4)
// The values may differ: the firmware scales each wheel so all tyres run at
// the same surface speed. On this robot M2 has a different gearbox (~1:9.3)
// from the other three (~1:15.5).
// CALIBRATE with the bring-up `cal` command.
constexpr float kWheelCountsPerRev[kWheelCount] = {680.4f, 408.7f, 681.2f, 683.6f};

// Encoder speed (counts/s) that every MOTOR can reach at full duty. Use the
// value suggested by the bring-up `maxspeed` command (a little below the
// slowest motor's full-duty speed).
// CALIBRATE.
constexpr float kMaxWheelSpeedCps = 5220.0f;

// Speed-controller gains (normalised units, see WheelSpeedController.h).
// kS CALIBRATE with the bring-up `friction` command; the rest with `step`.
inline WheelGains wheelGains() {
  WheelGains g;
  g.kP = 0.5f;
  g.kI = 6.0f;
  g.kF = 1.0f;
  g.kS = 0.026f;
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
constexpr uint8_t kRgbLedBrightness = 40;  // 0-255; status colours and the default for effects
// Where each of the board's LEDs (in wiring order) sits on the robot, seen
// from above: 0 = front-left, 1 = front-right, 2 = rear-right, 3 = rear-left.
// Used by the spin and front/back light effects. Change this if "spin" does
// not run in a circle or the front colour shows at the back.
constexpr uint8_t kLedRingPosition[kRgbLedCount] = {0, 1, 2, 3};

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
// Password for firmware updates over Wi-Fi (pio run -e robot_ota -t upload).
// Must match upload_flags in platformio.ini. CHANGE ME (in both places).
constexpr const char* kOtaPassword = "makerbot123";

// Derived values -------------------------------------------------------------
constexpr float kPi = 3.14159265358979f;
constexpr float maxCountsPerRev() {
  float highest = kWheelCountsPerRev[0];
  for (int i = 1; i < kWheelCount; ++i)
    if (kWheelCountsPerRev[i] > highest) highest = kWheelCountsPerRev[i];
  return highest;
}
// "100 %" wheel speed: the fastest every wheel can turn. The wheel with the
// most counts per revolution (highest gear ratio) sets it.
constexpr float kMaxWheelRevPerS = kMaxWheelSpeedCps / maxCountsPerRev();
constexpr float kMaxWheelSpeedMps = kMaxWheelRevPerS * kPi * kWheelDiameterM;
// Encoder counts/s of wheel i at 100 % wheel speed.
constexpr float wheelMaxCps(int i) { return kMaxWheelRevPerS * kWheelCountsPerRev[i]; }
// Converts wheel i's normalised wheel speed to the motor-normalised units the
// speed controller works in (1.0 = kMaxWheelSpeedCps).
constexpr float wheelMotorScale(int i) { return wheelMaxCps(i) / kMaxWheelSpeedCps; }

}  // namespace config
