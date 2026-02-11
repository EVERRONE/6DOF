#include "StepperController.h"

StepperController::StepperController()
  : speed_(DEFAULT_SPEED), isMoving_(false), isEnabled_(false), stopRequested_(false) {
  for (int i = 0; i < 6; i++) {
    currentSteps_[i] = 0;
    targetSteps_[i] = 0;
    lastStepTime_[i] = 0;
    stepInterval_[i] = 0;
  }
}

void StepperController::begin() {
  // Initialize step and direction pins
  for (int i = 0; i < 6; i++) {
    pinMode(JOINT_PINS[i].step, OUTPUT);
    pinMode(JOINT_PINS[i].dir, OUTPUT);
    digitalWrite(JOINT_PINS[i].step, LOW);
    digitalWrite(JOINT_PINS[i].dir, LOW);
  }

  // Initialize enable pins (active LOW) - one per CNC shield
  pinMode(ENABLE_PIN_1, OUTPUT);
  pinMode(ENABLE_PIN_2, OUTPUT);
  disable();  // Start disabled for safety
}

void StepperController::enable() {
  digitalWrite(ENABLE_PIN_1, LOW);  // Active LOW
  digitalWrite(ENABLE_PIN_2, LOW);  // Active LOW
  isEnabled_ = true;
  stopRequested_ = false;  // Clear stop flag so movement works again
}

void StepperController::disable() {
  digitalWrite(ENABLE_PIN_1, HIGH);  // Active LOW
  digitalWrite(ENABLE_PIN_2, HIGH);  // Active LOW
  isEnabled_ = false;
  isMoving_ = false;
}

void StepperController::emergencyStop() {
  stopRequested_ = true;
  isMoving_ = false;
  // Set target to current position
  for (int i = 0; i < 6; i++) {
    targetSteps_[i] = currentSteps_[i];
  }
}

void StepperController::requestStop() {
  stopRequested_ = true;
}

void StepperController::clearStop() {
  stopRequested_ = false;
}

bool StepperController::isStopRequested() const {
  return stopRequested_;
}

void StepperController::setTargetAngles(const JointAngles& target, float speed) {
  // Clamp to limits
  JointAngles clampedTarget = target;
  for (int i = 0; i < 6; i++) {
    clampedTarget[i] = constrain(target[i], JOINT_MIN[i], JOINT_MAX[i]);
  }

  targetAngles_ = clampedTarget;
  speed_ = speed;

  degreesToSteps(targetAngles_, targetSteps_);
  calculateStepIntervals();

  isMoving_ = true;
}

void StepperController::degreesToSteps(const JointAngles& angles, long steps[6]) {
  for (int i = 0; i < 6; i++) {
    steps[i] = (long)(angles[i] * USTEPS_PER_DEG[i]);
  }
}

void StepperController::stepsToDegrees(const long steps[6], JointAngles& angles) {
  for (int i = 0; i < 6; i++) {
    angles[i] = (float)steps[i] / USTEPS_PER_DEG[i];
  }
}

void StepperController::calculateStepIntervals() {
  // Find the joint with the largest angular displacement
  float maxDegrees = 0;
  for (int i = 0; i < 6; i++) {
    float degrees = (float)abs(targetSteps_[i] - currentSteps_[i]) / USTEPS_PER_DEG[i];
    if (degrees > maxDegrees) maxDegrees = degrees;
  }

  if (maxDegrees < 0.001) {
    isMoving_ = false;
    return;
  }

  // Calculate time to complete motion at given speed (deg/s)
  float totalTime = maxDegrees / speed_;  // seconds

  // Calculate step intervals for each joint (coordinated motion)
  for (int i = 0; i < 6; i++) {
    long steps = abs(targetSteps_[i] - currentSteps_[i]);
    if (steps > 0) {
      stepInterval_[i] = (unsigned long)((totalTime * 1000000.0) / steps);
    } else {
      stepInterval_[i] = 0;
    }
  }
}

void StepperController::update() {
  if (!isMoving_ || !isEnabled_) return;

  unsigned long currentTime = micros();
  bool anyMoving = false;

  for (int i = 0; i < 6; i++) {
    if (currentSteps_[i] == targetSteps_[i]) continue;

    if (currentTime - lastStepTime_[i] >= stepInterval_[i]) {
      // Set direction
      bool dir = (targetSteps_[i] > currentSteps_[i]);
      if (INVERT_DIR[i]) dir = !dir;
      digitalWrite(JOINT_PINS[i].dir, dir ? HIGH : LOW);

      // Generate step pulse
      digitalWrite(JOINT_PINS[i].step, HIGH);
      delayMicroseconds(PULSE_WIDTH_US);
      digitalWrite(JOINT_PINS[i].step, LOW);

      // Update position
      currentSteps_[i] += (targetSteps_[i] > currentSteps_[i]) ? 1 : -1;
      lastStepTime_[i] = currentTime;

      anyMoving = true;
    } else {
      anyMoving = true;  // Still waiting for interval
    }
  }

  if (!anyMoving) {
    isMoving_ = false;
    stepsToDegrees(currentSteps_, currentAngles_);
  }
}

bool StepperController::isMoving() const {
  return isMoving_;
}

JointAngles StepperController::getCurrentAngles() const {
  return currentAngles_;
}

void StepperController::setCurrentAngles(const JointAngles& angles) {
  currentAngles_ = angles;
  degreesToSteps(currentAngles_, currentSteps_);
}

void StepperController::stepJoint(int jointIndex, int steps, float speed) {
  // For homing - direct step control
  if (jointIndex < 0 || jointIndex >= 6) return;
  if (stopRequested_) return;

  bool dir = (steps > 0);
  if (INVERT_DIR[jointIndex]) dir = !dir;
  digitalWrite(JOINT_PINS[jointIndex].dir, dir ? HIGH : LOW);

  unsigned long interval = (unsigned long)(1000000.0 / (speed * USTEPS_PER_DEG[jointIndex]));

  for (int i = 0; i < abs(steps); i++) {
    if (stopRequested_) break;

    digitalWrite(JOINT_PINS[jointIndex].step, HIGH);
    delayMicroseconds(PULSE_WIDTH_US);
    digitalWrite(JOINT_PINS[jointIndex].step, LOW);
    delayMicroseconds(interval - PULSE_WIDTH_US);

    currentSteps_[jointIndex] += (steps > 0) ? 1 : -1;
  }

  stepsToDegrees(currentSteps_, currentAngles_);
}
