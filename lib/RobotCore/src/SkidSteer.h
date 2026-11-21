#pragma once

// Joystick shaping and arcade mixing for a skid-steer (tank-style) robot.

struct SideSpeeds {
  float left;
  float right;
};

// Applies a centre dead-zone and an expo curve to one joystick axis.
//  - value, result in [-1, 1]
//  - deadzone in [0, 1): stick travel ignored around centre
//  - expo in [0, 1]: 0 = linear, 1 = cubic (finer control near centre)
// The output is continuous: it starts at 0 at the dead-zone edge.
float shapeAxis(float value, float deadzone, float expo);

// Arcade mix: throttle forward/back, turn > 0 turns right (clockwise seen from
// above). turnGain scales turning authority relative to throttle. If either
// side would exceed +/-1 both are scaled down together, which keeps the
// commanded curve instead of clipping one side.
SideSpeeds mixArcade(float throttle, float turn, float turnGain);
