#include "ControlProtocol.h"

#include <math.h>
#include <stdlib.h>
#include <string.h>

namespace {

// Parses one float starting at *cursor, which must be preceded by exactly the
// separator handling done by the caller. Advances *cursor past the number.
bool parseFloat(const char** cursor, float* out) {
  const char* start = *cursor;
  if (*start == ' ') ++start;
  else return false;
  if (*start == '\0' || *start == ' ') return false;
  char* end = nullptr;
  const float value = strtof(start, &end);
  if (end == start || !isfinite(value)) return false;
  *cursor = end;
  *out = value;
  return true;
}

bool inRange(float v, float lo, float hi) { return v >= lo && v <= hi; }

}  // namespace

Message parseMessage(const char* data, size_t length) {
  Message invalid;
  if (data == nullptr || length == 0 || length > kMaxMessageLength) return invalid;

  char buffer[kMaxMessageLength + 1];
  memcpy(buffer, data, length);
  buffer[length] = '\0';
  if (strlen(buffer) != length) return invalid;  // embedded NUL

  const char* cursor = buffer + 1;
  Message msg;
  switch (buffer[0]) {
    case 'D': {
      float turn = 0.0f, throttle = 0.0f;
      if (!parseFloat(&cursor, &turn) || !parseFloat(&cursor, &throttle)) return invalid;
      if (!inRange(turn, -1.0f, 1.0f) || !inRange(throttle, -1.0f, 1.0f)) return invalid;
      msg = {MessageType::Drive, turn, throttle};
      break;
    }
    case 'A': {
      float value = 0.0f;
      if (!parseFloat(&cursor, &value) || (value != 0.0f && value != 1.0f)) return invalid;
      msg = {MessageType::Arm, value, 0.0f};
      break;
    }
    case 'L': {
      float value = 0.0f;
      if (!parseFloat(&cursor, &value) || !inRange(value, kMinSpeedLimit, 1.0f)) return invalid;
      msg = {MessageType::SpeedLimit, value, 0.0f};
      break;
    }
    case 'S':
      msg = {MessageType::Stop, 0.0f, 0.0f};
      break;
    case 'C':
      msg = {MessageType::Claim, 0.0f, 0.0f};
      break;
    default:
      return invalid;
  }
  if (*cursor != '\0') return invalid;  // trailing garbage
  return msg;
}
