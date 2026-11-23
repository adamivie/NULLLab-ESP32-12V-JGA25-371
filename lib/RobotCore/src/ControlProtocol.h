#pragma once

#include <stddef.h>
#include <stdint.h>

#include "LedEffects.h"

// Messages the browser sends over the WebSocket. Plain text keeps the
// firmware free of a JSON parser and makes the protocol easy to debug:
//
//   D <turn> <throttle>   joystick, both in [-1, 1] (sent ~20x per second;
//                         doubles as the heartbeat)
//   A <0|1>               disarm / arm
//   L <limit>             speed limit in [0.1, 1]
//   S                     emergency stop (brake + disarm)
//   C                     take control of the robot from another client
//   T <expo> <turn>       driving feel: stick expo in [0, 1], turn rate in [0.2, 1]
//   E <mode> <rgb1> <rgb2> <palette> <speed> <brightness> <movingOnly>
//                         light effect settings, all whole numbers (colours
//                         as decimal 0xRRGGBB); ranges: see LedEffects.h
//
// Anything malformed, out of range or non-finite is rejected as Invalid; the
// firmware never acts on a partially parsed command.
enum class MessageType { Invalid, Drive, Arm, SpeedLimit, Stop, Claim, Leds, Feel };

struct Message {
  MessageType type = MessageType::Invalid;
  float a = 0.0f;  // Drive: turn, Arm: 0/1, SpeedLimit: limit, Feel: expo
  float b = 0.0f;  // Drive: throttle, Feel: turn rate
  LedConfig leds;  // Leds: the validated settings
};

constexpr size_t kMaxMessageLength = 48;
constexpr float kMinSpeedLimit = 0.1f;
constexpr float kMinTurnGain = 0.2f;

Message parseMessage(const char* data, size_t length);
