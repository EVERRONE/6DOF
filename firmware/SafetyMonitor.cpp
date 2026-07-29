#include "SafetyMonitor.h"

SafetyMonitor::SafetyMonitor(StepperController& stepper, HomingController& homing)
    : stepper_(stepper), homing_(homing), tripped_(false) {}

int SafetyMonitor::update() {
  // Homing drives deliberately onto the switches, so it polices itself.
  if (homing_.isBusy()) return -1;

  if (!stepper_.isMoving()) {
    // Re-arm once the arm has stopped, so a stuck switch reports once per
    // movement rather than on every loop pass.
    tripped_ = false;
    return -1;
  }

  if (tripped_) return -1;

  for (int axis = 0; axis < NUM_AXES; axis++) {
    if (!homing_.isEndstopTriggered(axis)) continue;

    // Stop immediately rather than decelerate. Deceleration is gentler on the
    // mechanics but takes distance - J3 at 60 deg/s needs about 12 degrees within
    // its acceleration limit - which is more than the margin between an endstop
    // and its hard stop. Cutting the pulses can lose steps, so this also marks
    // the position untrusted and the arm needs re-homing, which is the right
    // outcome after an unexpected limit hit.
    stepper_.emergencyStop();
    tripped_ = true;
    return axis;
  }

  return -1;
}
