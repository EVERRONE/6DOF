#ifndef TYPES_H
#define TYPES_H

// Shared value types. Host-safe: no <Arduino.h> dependency, so the motion
// planner can be unit-tested off-target.

#include <stdint.h>
#include "config.h"

/** Joint position in degrees, ordered J1..J6. */
struct JointAngles {
  float angles[NUM_AXES];

  JointAngles() {
    for (int i = 0; i < NUM_AXES; i++) angles[i] = 0.0f;
  }

  float& operator[](int index) { return angles[index]; }
  const float& operator[](int index) const { return angles[index]; }
};

/** Endstop switch states, ordered J1..J6. */
struct EndstopState {
  bool triggered[NUM_AXES];

  EndstopState() {
    for (int i = 0; i < NUM_AXES; i++) triggered[i] = false;
  }
};

/** Top-level controller state, reported to the host. */
enum RobotState {
  STATE_IDLE,
  STATE_MOVING,
  STATE_HOMING,
  STATE_ERROR,
  STATE_ESTOPPED
};

#endif
