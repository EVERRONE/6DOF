#include "HomingController.h"

HomingController::HomingController(StepperController& stepper)
  : stepper_(stepper) {}

void HomingController::begin() {
  // Initialize endstop pins
  for (int i = 0; i < 6; i++) {
    if (HAS_ENDSTOP[i]) {
      pinMode(ENDSTOP_PINS[i], INPUT_PULLUP);
    }
  }
}

bool HomingController::isEndstopTriggered(int jointIndex) {
  if (!HAS_ENDSTOP[jointIndex]) return false;
  return digitalRead(ENDSTOP_PINS[jointIndex]) == LOW;  // Active LOW
}

EndstopState HomingController::getEndstopState() {
  EndstopState state;
  for (int i = 0; i < 6; i++) {
    state.triggered[i] = isEndstopTriggered(i);
  }
  return state;
}

bool HomingController::homeJoint(int jointIndex) {
  if (!HAS_ENDSTOP[jointIndex]) {
    Serial.println("ERROR No endstop on this joint");
    return false;
  }

  // Cancel any ongoing movement before starting homing
  stepper_.emergencyStop();
  stepper_.clearStop();

  // 1. Fast approach to endstop
  if (!findEndstop(jointIndex)) {
    if (stepper_.isStopRequested()) {
      Serial.println("OK Homing aborted");
      return false;
    }
    Serial.println("ERROR Endstop not found");
    return false;
  }

  // 2. Back off
  backOff(jointIndex);
  if (stepper_.isStopRequested()) {
    Serial.println("OK Homing aborted");
    return false;
  }

  // 3. Slow fine approach
  fineApproach(jointIndex);
  if (stepper_.isStopRequested()) {
    Serial.println("OK Homing aborted");
    return false;
  }

  // 4. Set zero position
  JointAngles zeros;
  stepper_.setCurrentAngles(zeros);

  // 5. Move to post-home position
  JointAngles postHome;
  postHome[jointIndex] = POST_HOME_ANGLES[jointIndex];
  stepper_.setTargetAngles(postHome, HOMING_SPEED);

  while (stepper_.isMoving()) {
    stepper_.update();
    checkSerialForStop();
    if (stepper_.isStopRequested()) {
      Serial.println("OK Homing aborted");
      return false;
    }
  }

  Serial.print("HOMED ");
  Serial.println(jointIndex + 1);

  return true;
}

bool HomingController::findEndstop(int jointIndex) {
  int direction = HOME_TOWARD_MIN[jointIndex] ? -1 : 1;
  int maxSteps = (int)(360.0 * USTEPS_PER_DEG[jointIndex]);  // Max travel

  for (int i = 0; i < maxSteps; i++) {
    if (isEndstopTriggered(jointIndex)) return true;
    if (stepper_.isStopRequested()) return false;

    // Check serial for E-Stop every 100 steps
    if (i % 100 == 0) checkSerialForStop();

    stepper_.stepJoint(jointIndex, direction, HOMING_SPEED);
  }

  return false;  // Endstop not found within range
}

void HomingController::backOff(int jointIndex) {
  int direction = HOME_TOWARD_MIN[jointIndex] ? 1 : -1;
  int steps = (int)(BACKOFF_DISTANCE * USTEPS_PER_DEG[jointIndex]);

  for (int i = 0; i < steps; i++) {
    if (stepper_.isStopRequested()) return;

    stepper_.stepJoint(jointIndex, direction, HOMING_SPEED);

    // Stop if endstop released
    if (!isEndstopTriggered(jointIndex)) break;
  }
}

void HomingController::fineApproach(int jointIndex) {
  int direction = HOME_TOWARD_MIN[jointIndex] ? -1 : 1;
  float fineSpeed = HOMING_SPEED * 0.2;  // 20% of homing speed
  int maxSteps = (int)(BACKOFF_DISTANCE * 2 * USTEPS_PER_DEG[jointIndex]);
  int count = 0;

  while (!isEndstopTriggered(jointIndex) && count < maxSteps) {
    if (stepper_.isStopRequested()) return;

    stepper_.stepJoint(jointIndex, direction, fineSpeed);
    count++;
  }
}

void HomingController::checkSerialForStop() {
  // Poll serial for 'S' (E-Stop) during blocking homing operations
  while (Serial.available()) {
    char c = Serial.read();
    if (c == 'S' || c == 's') {
      stepper_.emergencyStop();
      Serial.println("OK Emergency stop");
    }
  }
}

bool HomingController::homeAll() {
  // Home in sequence: J2, J3, J4, J5
  int homeSequence[] = {1, 2, 3, 4};  // 0-indexed

  for (int i = 0; i < 4; i++) {
    int joint = homeSequence[i];
    if (!homeJoint(joint)) {
      return false;
    }
  }

  Serial.println("OK All joints homed");
  return true;
}
