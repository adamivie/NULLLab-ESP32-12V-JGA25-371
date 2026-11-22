#pragma once

#include <stddef.h>

// Messages the browser sends over the WebSocket. Plain text keeps the
// firmware free of a JSON parser and makes the protocol easy to debug:
//
//   D <turn> <throttle>   joystick, both in [-1, 1] (sent ~20x per second;
//                         doubles as the heartbeat)
//   A <0|1>               disarm / arm
//   L <limit>             speed limit in [0.1, 1]
//   S                     emergency stop (brake + disarm)
//   C                     take control of the robot from another client
//
// Anything malformed, out of range or non-finite is rejected as Invalid; the
// firmware never acts on a partially parsed command.
enum class MessageType { Invalid, Drive, Arm, SpeedLimit, Stop, Claim };

struct Message {
  MessageType type = MessageType::Invalid;
  float a = 0.0f;  // Drive: turn, Arm: 0/1, SpeedLimit: limit
  float b = 0.0f;  // Drive: throttle
};

constexpr size_t kMaxMessageLength = 48;
constexpr float kMinSpeedLimit = 0.1f;

Message parseMessage(const char* data, size_t length);
