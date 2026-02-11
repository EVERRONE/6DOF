#ifndef TYPES_H
#define TYPES_H

#include <Arduino.h>

// Joint position in degrees
struct JointAngles {
  float angles[6];

  JointAngles() {
    for (int i = 0; i < 6; i++) angles[i] = 0.0;
  }

  float& operator[](int index) { return angles[index]; }
  const float& operator[](int index) const { return angles[index]; }
};

// Motion command
struct MotionCommand {
  JointAngles targetAngles;
  float speed;  // degrees/second
  bool isValid;

  MotionCommand() : speed(0), isValid(false) {}
};

// Robot state
enum RobotState {
  STATE_IDLE,
  STATE_MOVING,
  STATE_HOMING,
  STATE_ERROR,
  STATE_ESTOPPED
};

// Endstop state
struct EndstopState {
  bool triggered[6];

  EndstopState() {
    for (int i = 0; i < 6; i++) triggered[i] = false;
  }
};

#endif
