#include "StepperController.h"
#include <EEPROM.h>
#include <math.h>

namespace {
float rampSpeedTowardDesired(
  float currentSpeedDegS,
  float desiredSpeedDegS,
  unsigned long lastUpdateMicros,
  unsigned long nowMicros
) {
  if (lastUpdateMicros == 0 || nowMicros <= lastUpdateMicros) {
    return currentSpeedDegS;
  }

  float dt = (float)(nowMicros - lastUpdateMicros) / 1000000.0f;
  if (dt < 0.0001f) dt = 0.0001f;

  float maxDeltaSpeed = STREAM_SPEED_ACCEL_DEG_S2 * dt;
  float deltaSpeed = desiredSpeedDegS - currentSpeedDegS;
  if (deltaSpeed > maxDeltaSpeed) deltaSpeed = maxDeltaSpeed;
  if (deltaSpeed < -maxDeltaSpeed) deltaSpeed = -maxDeltaSpeed;

  float nextSpeed = currentSpeedDegS + deltaSpeed;
  if (nextSpeed < STREAM_MIN_SPEED_DEG_S) nextSpeed = STREAM_MIN_SPEED_DEG_S;
  if (nextSpeed > STREAM_MAX_SPEED_DEG_S) nextSpeed = STREAM_MAX_SPEED_DEG_S;
  return nextSpeed;
}
}

StepperController* StepperController::timerInstance_ = nullptr;

StepperController::StepperController()
  : motionProfile_(),
    isMoving_(false),
    isEnabled_(false),
    stopRequested_(false),
    streamingActive_(false),
    pulseWidthTicks_(1),
    dirSetupTicks_(1) {
  for (int i = 0; i < 6; i++) {
    currentSteps_[i] = 0;
    targetSteps_[i] = 0;
    stepInterval_[i] = 0;
    stepPulseHigh_[i] = false;
    stepPulseTicksRemaining_[i] = 0;
    dirSetupTicksRemaining_[i] = 0;
    stepCountdownTicks_[i] = 0;
    stepIntervalTicks_[i] = 1;
    pendingStepDir_[i] = 0;
    calibration_[i].scale = DEFAULT_CAL_SCALE[i];
    calibration_[i].offset = DEFAULT_CAL_OFFSET[i];
  }
}

void StepperController::begin() {
  for (int i = 0; i < 6; i++) {
    pinMode(JOINT_PINS[i].step, OUTPUT);
    pinMode(JOINT_PINS[i].dir, OUTPUT);
    digitalWrite(JOINT_PINS[i].step, LOW);
    digitalWrite(JOINT_PINS[i].dir, LOW);
  }

  pinMode(ENABLE_PIN_1, OUTPUT);
  pinMode(ENABLE_PIN_2, OUTPUT);
  disable();

  if (!loadCalibration()) {
    resetCalibration();
  }

  pulseWidthTicks_ = (uint16_t)max(1UL, (PULSE_WIDTH_US + STEPPER_TIMER_PERIOD_US - 1) / STEPPER_TIMER_PERIOD_US);
  dirSetupTicks_ = (uint16_t)((DIR_SETUP_US + STEPPER_TIMER_PERIOD_US - 1) / STEPPER_TIMER_PERIOD_US);

  timerInstance_ = this;
  stepTimer_.begin(onStepTimerISR, STEPPER_TIMER_PERIOD_US);
}

void StepperController::enable() {
  digitalWrite(ENABLE_PIN_1, LOW);
  digitalWrite(ENABLE_PIN_2, LOW);
  isEnabled_ = true;
  stopRequested_ = false;
}

void StepperController::disable() {
  digitalWrite(ENABLE_PIN_1, HIGH);
  digitalWrite(ENABLE_PIN_2, HIGH);
  noInterrupts();
  isEnabled_ = false;
  isMoving_ = false;
  streamingActive_ = false;
  motionProfile_ = MotionProfileState();
  for (int i = 0; i < 6; i++) {
    stepInterval_[i] = 0;
    stepCountdownTicks_[i] = 0;
    stepIntervalTicks_[i] = 0;
    dirSetupTicksRemaining_[i] = 0;
    stepPulseTicksRemaining_[i] = 0;
    if (stepPulseHigh_[i]) {
      digitalWrite(JOINT_PINS[i].step, LOW);
    }
    stepPulseHigh_[i] = false;
    pendingStepDir_[i] = 0;
  }
  interrupts();
}

void StepperController::emergencyStop() {
  stopRequested_ = true;
  noInterrupts();
  isMoving_ = false;
  streamingActive_ = false;
  motionProfile_ = MotionProfileState();
  for (int i = 0; i < 6; i++) {
    targetSteps_[i] = currentSteps_[i];
    stepInterval_[i] = 0;
    stepCountdownTicks_[i] = 0;
    stepIntervalTicks_[i] = 0;
    dirSetupTicksRemaining_[i] = 0;
    stepPulseTicksRemaining_[i] = 0;
    if (stepPulseHigh_[i]) {
      digitalWrite(JOINT_PINS[i].step, LOW);
    }
    stepPulseHigh_[i] = false;
    pendingStepDir_[i] = 0;
  }
  interrupts();
  recomputeAnglesFromSteps();
}

bool StepperController::isEnabled() const {
  return isEnabled_;
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
  MotionCommandContext context;
  context.mode = STEPPER_MOTION_POINT_TO_POINT;
  context.requestedSpeedDegS = clampCommandSpeed(speed);
  context.preserveCountdown = captureWasMoving();
  context.resetProfileFromFloor = !context.preserveCountdown;
  context.slewImmediately = false;
  applyMotionCommand(target, context);
}

void StepperController::setStreamingCommand(const StreamingMotionCommand& command) {
  // Capture moving state BEFORE updating streamingActive_ so the first command
  // correctly gets preserveCountdown=false (starting from rest).
  bool wasMoving = captureWasMoving();

  // Determine whether any joint has a non-negligible commanded velocity.
  bool hasVelocity = false;
  for (int i = 0; i < 6; i++) {
    if (fabs(command.targetVelocityDegS[i]) > 0.001f) {
      hasVelocity = true;
      break;
    }
  }
  streamingActive_ = hasVelocity;

  // Store per-joint velocity BEFORE applyMotionCommand calls calculateStepIntervals.
  noInterrupts();
  motionProfile_.streamingVelocity = command.targetVelocityDegS;
  interrupts();

  MotionCommandContext context;
  context.mode = STEPPER_MOTION_STREAMING;
  context.requestedSpeedDegS = speedFromVelocityVector(command.targetVelocityDegS);
  context.preserveCountdown = wasMoving;
  context.resetProfileFromFloor = !wasMoving;
  context.slewImmediately = wasMoving;
  applyMotionCommand(command.targetAngles, context);
}

void StepperController::setStreamingTargetAngles(const JointAngles& target, float speed) {
  // This API carries no per-joint velocity; fall back to distance-based intervals.
  streamingActive_ = false;
  noInterrupts();
  for (int i = 0; i < 6; i++) motionProfile_.streamingVelocity[i] = 0.0f;
  interrupts();

  MotionCommandContext context;
  context.mode = STEPPER_MOTION_STREAMING;
  context.requestedSpeedDegS = clampCommandSpeed(speed);
  context.preserveCountdown = captureWasMoving();
  context.resetProfileFromFloor = !context.preserveCountdown;
  context.slewImmediately = context.preserveCountdown;
  applyMotionCommand(target, context);
}

void StepperController::applyMotionCommand(const JointAngles& target, const MotionCommandContext& context) {
  JointAngles clampedTarget = clampTargetAngles(target);
  targetAngles_ = clampedTarget;

  unsigned long nowMicros = micros();
  motionProfile_.mode = context.mode;
  motionProfile_.requestedSpeedDegS = context.requestedSpeedDegS;

  if (context.resetProfileFromFloor || motionProfile_.appliedSpeedDegS < STREAM_MIN_SPEED_DEG_S) {
    motionProfile_.appliedSpeedDegS = STREAM_MIN_SPEED_DEG_S;
  }

  if (context.mode == STEPPER_MOTION_STREAMING) {
    // In streaming mode the Hermite polynomial already provides a smooth velocity
    // profile. Applying an additional speed slew here causes the applied speed to
    // lag behind the commanded speed by up to 300 deg/s² × 1ms = 0.3 deg/s per
    // tick, which makes the arm fall behind the planned trajectory during
    // acceleration phases and then catch up with a jerk. Set the velocity
    // directly so the step intervals track the Hermite tangent without lag.
    motionProfile_.appliedSpeedDegS = motionProfile_.requestedSpeedDegS;
  } else if (context.slewImmediately) {
    motionProfile_.appliedSpeedDegS = rampSpeedTowardDesired(
      motionProfile_.appliedSpeedDegS,
      motionProfile_.requestedSpeedDegS,
      motionProfile_.lastUpdateMicros,
      nowMicros
    );
  }

  motionProfile_.lastUpdateMicros = nowMicros;

  long steps[6];
  degreesToSteps(targetAngles_, steps);
  storeTargetSteps(steps);
  syncMotionExecutionState(context.preserveCountdown);
}

bool StepperController::captureWasMoving() const {
  noInterrupts();
  bool moving = isMoving_ || streamingActive_;
  interrupts();
  return moving;
}

float StepperController::clampCommandSpeed(float speed) const {
  float desiredSpeedDegS = (speed > 0.0f) ? speed : DEFAULT_SPEED;
  if (desiredSpeedDegS < STREAM_MIN_SPEED_DEG_S) desiredSpeedDegS = STREAM_MIN_SPEED_DEG_S;
  if (desiredSpeedDegS > STREAM_MAX_SPEED_DEG_S) desiredSpeedDegS = STREAM_MAX_SPEED_DEG_S;
  return desiredSpeedDegS;
}

float StepperController::speedFromVelocityVector(const JointAngles& velocityDegS) const {
  float maxAbs = 0.0f;
  for (int i = 0; i < 6; i++) {
    float value = fabs(velocityDegS[i]);
    if (value > maxAbs) maxAbs = value;
  }
  return clampCommandSpeed(maxAbs);
}

JointAngles StepperController::clampTargetAngles(const JointAngles& target) const {
  JointAngles clamped = target;
  for (int i = 0; i < 6; i++) {
    clamped[i] = constrain(target[i], JOINT_MIN[i], JOINT_MAX[i]);
  }
  return clamped;
}

void StepperController::storeTargetSteps(const long steps[6]) {
  noInterrupts();
  for (int i = 0; i < 6; i++) {
    targetSteps_[i] = steps[i];
  }
  interrupts();
}

bool StepperController::syncMotionExecutionState(bool preserveCountdown) {
  calculateStepIntervals();
  refreshTimerStepIntervals(preserveCountdown);

  bool anyDelta = false;
  noInterrupts();
  for (int i = 0; i < 6; i++) {
    if (targetSteps_[i] != currentSteps_[i]) {
      anyDelta = true;
      break;
    }
  }
  // In velocity-based streaming mode the arm is still "moving" even when all integer
  // deltas are momentarily zero (joint sits between two step positions).
  isMoving_ = anyDelta || streamingActive_;
  if (!anyDelta && !streamingActive_) {
    motionProfile_.mode = STEPPER_MOTION_POINT_TO_POINT;
    motionProfile_.lastUpdateMicros = 0;
  }
  interrupts();
  return anyDelta;
}

bool StepperController::slewMotionProfile(unsigned long nowMicros) {
  bool activePointToPoint = false;
  noInterrupts();
  activePointToPoint = isMoving_;
  interrupts();

  if (!activePointToPoint || motionProfile_.mode != STEPPER_MOTION_POINT_TO_POINT) {
    return false;
  }

  if (motionProfile_.lastUpdateMicros == 0) {
    motionProfile_.lastUpdateMicros = nowMicros;
    return false;
  }

  float nextSpeedDegS = rampSpeedTowardDesired(
    motionProfile_.appliedSpeedDegS,
    motionProfile_.requestedSpeedDegS,
    motionProfile_.lastUpdateMicros,
    nowMicros
  );

  bool changed = fabs(nextSpeedDegS - motionProfile_.appliedSpeedDegS) > 0.0001f;
  motionProfile_.appliedSpeedDegS = nextSpeedDegS;
  motionProfile_.lastUpdateMicros = nowMicros;

  if (changed) {
    calculateStepIntervals();
    refreshTimerStepIntervals(true);
  }

  return changed;
}

void StepperController::update() {
  slewMotionProfile(micros());

  long currentStepsSnapshot[6];
  long targetStepsSnapshot[6];
  bool moving = false;
  snapshotStepState(currentStepsSnapshot, targetStepsSnapshot, &moving);

  if (!moving) {
    stepsToDegrees(currentStepsSnapshot, currentAngles_);
    stepsToDegrees(targetStepsSnapshot, targetAngles_);
    return;
  }

  bool anyMoving = false;
  for (int i = 0; i < 6; i++) {
    if (currentStepsSnapshot[i] != targetStepsSnapshot[i]) {
      anyMoving = true;
      break;
    }
  }

  if (!anyMoving) {
    noInterrupts();
    // Do not clear isMoving_ during velocity-based streaming: all deltas can be
    // transiently zero (joint between integer step positions) while motion continues.
    if (!streamingActive_) {
      isMoving_ = false;
      motionProfile_.mode = STEPPER_MOTION_POINT_TO_POINT;
      motionProfile_.lastUpdateMicros = 0;
    }
    interrupts();
  }

  stepsToDegrees(currentStepsSnapshot, currentAngles_);
  stepsToDegrees(targetStepsSnapshot, targetAngles_);
}

bool StepperController::isMoving() const {
  return captureWasMoving();
}

JointAngles StepperController::getCurrentAngles() const {
  return currentAngles_;
}

JointAngles StepperController::getTargetAngles() const {
  return targetAngles_;
}

void StepperController::setCurrentAngles(const JointAngles& angles) {
  currentAngles_ = angles;

  long steps[6];
  degreesToSteps(currentAngles_, steps);

  noInterrupts();
  for (int i = 0; i < 6; i++) {
    currentSteps_[i] = steps[i];
    targetSteps_[i] = steps[i];
    stepInterval_[i] = 0;
    stepCountdownTicks_[i] = 0;
    stepIntervalTicks_[i] = 0;
    dirSetupTicksRemaining_[i] = 0;
    stepPulseTicksRemaining_[i] = 0;
    if (stepPulseHigh_[i]) {
      digitalWrite(JOINT_PINS[i].step, LOW);
    }
    stepPulseHigh_[i] = false;
    pendingStepDir_[i] = 0;
  }
  isMoving_ = false;
  streamingActive_ = false;
  motionProfile_ = MotionProfileState();
  interrupts();

  targetAngles_ = angles;
}

float StepperController::getCurrentRawDegrees(int jointIndex) const {
  if (jointIndex < 0 || jointIndex >= 6) return 0.0f;
  noInterrupts();
  long steps = currentSteps_[jointIndex];
  interrupts();
  return rawDegreesFromSteps(steps, jointIndex);
}

void StepperController::stepJoint(int jointIndex, int steps, float speed) {
  if (jointIndex < 0 || jointIndex >= 6) return;
  if (stopRequested_) return;

  noInterrupts();
  isMoving_ = false;
  streamingActive_ = false;
  motionProfile_ = MotionProfileState();
  stepCountdownTicks_[jointIndex] = 0;
  stepIntervalTicks_[jointIndex] = 0;
  dirSetupTicksRemaining_[jointIndex] = 0;
  stepPulseTicksRemaining_[jointIndex] = 0;
  if (stepPulseHigh_[jointIndex]) {
    digitalWrite(JOINT_PINS[jointIndex].step, LOW);
  }
  stepPulseHigh_[jointIndex] = false;
  pendingStepDir_[jointIndex] = 0;
  targetSteps_[jointIndex] = currentSteps_[jointIndex];
  interrupts();

  bool dir = (steps > 0);
  if (INVERT_DIR[jointIndex]) dir = !dir;
  digitalWrite(JOINT_PINS[jointIndex].dir, dir ? HIGH : LOW);

  float safeSpeedDegS = (speed > 0.0f) ? speed : DEFAULT_SPEED;
  float scaledSpeedDegS = safeSpeedDegS / calibration_[jointIndex].scale;
  unsigned long intervalUs = (unsigned long)(1000000.0f / (scaledSpeedDegS * USTEPS_PER_DEG[jointIndex]));
  if (intervalUs < MIN_STEP_INTERVAL_US) intervalUs = MIN_STEP_INTERVAL_US;

  for (int i = 0; i < abs(steps); i++) {
    if (stopRequested_) break;

    if (DIR_SETUP_US > 0) {
      delayMicroseconds(DIR_SETUP_US);
    }
    digitalWrite(JOINT_PINS[jointIndex].step, HIGH);
    delayMicroseconds(PULSE_WIDTH_US);
    digitalWrite(JOINT_PINS[jointIndex].step, LOW);
    if (intervalUs > PULSE_WIDTH_US) {
      delayMicroseconds(intervalUs - PULSE_WIDTH_US);
    }

    noInterrupts();
    currentSteps_[jointIndex] += (steps > 0) ? 1 : -1;
    targetSteps_[jointIndex] = currentSteps_[jointIndex];
    interrupts();
  }

  long currentStepsSnapshot[6];
  long targetStepsSnapshot[6];
  snapshotStepState(currentStepsSnapshot, targetStepsSnapshot, nullptr);
  stepsToDegrees(currentStepsSnapshot, currentAngles_);
  stepsToDegrees(targetStepsSnapshot, targetAngles_);
}

void StepperController::setCalibration(int jointIndex, float scale, float offset) {
  if (jointIndex < 0 || jointIndex >= 6) return;
  calibration_[jointIndex].scale = (scale >= MIN_CAL_SCALE) ? scale : MIN_CAL_SCALE;
  calibration_[jointIndex].offset = offset;
  recomputeAnglesFromSteps();
}

JointCalibration StepperController::getCalibration(int jointIndex) const {
  if (jointIndex < 0 || jointIndex >= 6) {
    JointCalibration empty = {1.0f, 0.0f};
    return empty;
  }
  return calibration_[jointIndex];
}

void StepperController::setCalibrationAll(const float scale[6], const float offset[6]) {
  for (int i = 0; i < 6; i++) {
    calibration_[i].scale = (scale[i] >= MIN_CAL_SCALE) ? scale[i] : MIN_CAL_SCALE;
    calibration_[i].offset = offset[i];
  }
  recomputeAnglesFromSteps();
}

void StepperController::resetCalibration() {
  for (int i = 0; i < 6; i++) {
    calibration_[i].scale = DEFAULT_CAL_SCALE[i];
    calibration_[i].offset = DEFAULT_CAL_OFFSET[i];
  }
  recomputeAnglesFromSteps();
}

bool StepperController::loadCalibration() {
  CalibrationData stored;
  EEPROM.get(CALIBRATION_EEPROM_ADDR, stored);
  if (stored.magic != CALIBRATION_MAGIC || stored.version != CALIBRATION_VERSION) {
    return false;
  }

  for (int i = 0; i < 6; i++) {
    calibration_[i] = stored.cal[i];
    if (calibration_[i].scale < MIN_CAL_SCALE) {
      calibration_[i].scale = MIN_CAL_SCALE;
    }
  }
  recomputeAnglesFromSteps();
  return true;
}

bool StepperController::saveCalibration() const {
  CalibrationData stored;
  stored.magic = CALIBRATION_MAGIC;
  stored.version = CALIBRATION_VERSION;
  for (int i = 0; i < 6; i++) {
    stored.cal[i] = calibration_[i];
  }
  EEPROM.put(CALIBRATION_EEPROM_ADDR, stored);
  return true;
}

void StepperController::degreesToSteps(const JointAngles& angles, long steps[6]) {
  for (int i = 0; i < 6; i++) {
    float logicalDeg = angles[i];
    float rawDeg = (logicalDeg - calibration_[i].offset) / calibration_[i].scale;
    steps[i] = (long)(rawDeg * USTEPS_PER_DEG[i]);
  }
}

void StepperController::stepsToDegrees(const long steps[6], JointAngles& angles) {
  for (int i = 0; i < 6; i++) {
    float rawDeg = (float)steps[i] / USTEPS_PER_DEG[i];
    angles[i] = rawDeg * calibration_[i].scale + calibration_[i].offset;
  }
}

void StepperController::calculateStepIntervals() {
  long currentStepsSnapshot[6];
  long targetStepsSnapshot[6];
  snapshotStepState(currentStepsSnapshot, targetStepsSnapshot, nullptr);

  // Velocity-based streaming mode: compute step intervals directly from the per-joint
  // Hermite tangent velocity instead of from remaining integer step distance.
  //
  // Why: with distance-based intervals, a joint moving at 0.533 steps/ms (e.g. J5 at
  // 30 deg/s) alternates between 0 and 1 remaining steps every tick.  When delta=0 the
  // interval becomes ∞ (no step scheduled); when delta=1 it becomes 1875 µs.  The ISR
  // then fires an irregular burst/skip pattern at ~111 Hz — audible roughness.
  // Velocity-based intervals are continuous and independent of integer step rounding.
  if (motionProfile_.mode == STEPPER_MOTION_STREAMING && streamingActive_) {
    for (int i = 0; i < 6; i++) {
      float logicalVelDegS = fabs(motionProfile_.streamingVelocity[i]);
      long delta = labs(targetStepsSnapshot[i] - currentStepsSnapshot[i]);

      if (logicalVelDegS < 0.001f) {
        if (delta > 0) {
          // Safety fallback: residual steps at near-zero velocity.
          // Slow finite interval prevents ISR runaway; normal trajectory execution
          // should never produce nonzero delta with zero commanded velocity.
          stepInterval_[i] = 500000UL;  // 0.5 s/step
        } else {
          // Velocity ≈ 0 and at target: set interval to zero so this joint is idle.
          stepInterval_[i] = 0;
        }
      } else {
        // Convert logical deg/s → raw deg/s → steps/s → µs/step.
        float rawVelDegS = logicalVelDegS / calibration_[i].scale;
        unsigned long intervalUs = (unsigned long)(1000000.0f / (rawVelDegS * USTEPS_PER_DEG[i]));
        if (intervalUs < MIN_STEP_INTERVAL_US) intervalUs = MIN_STEP_INTERVAL_US;
        stepInterval_[i] = intervalUs;
        // Note: when delta=0 (joint between integer steps) the interval is still set
        // nonzero so the ISR can tick the countdown and fire the next step on time.
      }
    }
    return;
  }

  // Point-to-point mode (or non-velocity streaming): distance-based interval distribution.
  float maxDegrees = 0.0f;
  for (int i = 0; i < 6; i++) {
    float rawDeg = fabs((float)(targetStepsSnapshot[i] - currentStepsSnapshot[i]) / USTEPS_PER_DEG[i]);
    float logicalDeg = rawDeg * calibration_[i].scale;
    if (logicalDeg > maxDegrees) {
      maxDegrees = logicalDeg;
    }
  }

  if (maxDegrees < 0.001f || motionProfile_.appliedSpeedDegS <= 0.0f) {
    for (int i = 0; i < 6; i++) {
      stepInterval_[i] = 0;
    }
    return;
  }

  float totalTime = maxDegrees / motionProfile_.appliedSpeedDegS;
  for (int i = 0; i < 6; i++) {
    long stepCount = labs(targetStepsSnapshot[i] - currentStepsSnapshot[i]);
    if (stepCount == 0) {
      stepInterval_[i] = 0;
      continue;
    }

    unsigned long intervalUs = (unsigned long)((totalTime * 1000000.0f) / (float)stepCount);
    if (intervalUs < MIN_STEP_INTERVAL_US) {
      intervalUs = MIN_STEP_INTERVAL_US;
    }
    stepInterval_[i] = intervalUs;
  }
}

void StepperController::refreshTimerStepIntervals(bool preserveCountdown) {
  noInterrupts();
  for (int i = 0; i < 6; i++) {
    if (stepInterval_[i] == 0) {
      stepIntervalTicks_[i] = 0;
      stepCountdownTicks_[i] = 0;
      dirSetupTicksRemaining_[i] = 0;
      stepPulseTicksRemaining_[i] = 0;
      if (stepPulseHigh_[i]) {
        digitalWrite(JOINT_PINS[i].step, LOW);
      }
      stepPulseHigh_[i] = false;
      pendingStepDir_[i] = 0;
      continue;
    }

    uint32_t nextIntervalTicks = (stepInterval_[i] + STEPPER_TIMER_PERIOD_US - 1) / STEPPER_TIMER_PERIOD_US;
    if (nextIntervalTicks < 1) nextIntervalTicks = 1;

    uint32_t previousIntervalTicks = stepIntervalTicks_[i];
    uint32_t previousCountdownTicks = stepCountdownTicks_[i];
    stepIntervalTicks_[i] = nextIntervalTicks;

    if (!preserveCountdown || previousIntervalTicks == 0 || previousCountdownTicks == 0) {
      stepCountdownTicks_[i] = nextIntervalTicks;
      continue;
    }

    float remainingRatio = (float)previousCountdownTicks / (float)previousIntervalTicks;
    if (remainingRatio < 0.0f) remainingRatio = 0.0f;
    if (remainingRatio > 1.0f) remainingRatio = 1.0f;

    uint32_t preservedCountdownTicks = (uint32_t)roundf((float)nextIntervalTicks * remainingRatio);
    if (preservedCountdownTicks < 1) preservedCountdownTicks = 1;
    stepCountdownTicks_[i] = preservedCountdownTicks;
  }
  interrupts();
}

void StepperController::snapshotStepState(long currentSteps[6], long targetSteps[6], bool* motionActive) const {
  noInterrupts();
  if (motionActive) {
    *motionActive = isMoving_;
  }
  for (int i = 0; i < 6; i++) {
    currentSteps[i] = currentSteps_[i];
    targetSteps[i] = targetSteps_[i];
  }
  interrupts();
}

void StepperController::recomputeAnglesFromSteps() {
  long currentStepsSnapshot[6];
  long targetStepsSnapshot[6];
  snapshotStepState(currentStepsSnapshot, targetStepsSnapshot, nullptr);
  stepsToDegrees(currentStepsSnapshot, currentAngles_);
  stepsToDegrees(targetStepsSnapshot, targetAngles_);
}

float StepperController::rawDegreesFromSteps(long steps, int jointIndex) const {
  return (float)steps / USTEPS_PER_DEG[jointIndex];
}

void StepperController::onStepTimerISR() {
  if (timerInstance_) {
    timerInstance_->processStepTimerISR();
  }
}

void StepperController::processStepTimerISR() {
  if (!isEnabled_ || !isMoving_) {
    return;
  }

  bool anyMoving = false;

  for (int i = 0; i < 6; i++) {
    long delta = targetSteps_[i] - currentSteps_[i];
    if (delta == 0) {
      if (streamingActive_ && stepIntervalTicks_[i] > 0) {
        // Velocity-based streaming: joint is momentarily at its integer-rounded
        // target but the commanded velocity is non-zero.  Tick the countdown without
        // firing a step so the rhythm is intact when the next target arrives.
        anyMoving = true;
        if (stepCountdownTicks_[i] > 0) {
          stepCountdownTicks_[i]--;
        } else {
          stepCountdownTicks_[i] = stepIntervalTicks_[i];
        }
        continue;
      }
      // PTP or zero-velocity streaming: joint is done, reset state.
      if (stepPulseHigh_[i]) {
        digitalWrite(JOINT_PINS[i].step, LOW);
      }
      stepCountdownTicks_[i] = 0;
      dirSetupTicksRemaining_[i] = 0;
      stepPulseTicksRemaining_[i] = 0;
      stepPulseHigh_[i] = false;
      pendingStepDir_[i] = 0;
      continue;
    }

    anyMoving = true;

    if (stepPulseHigh_[i]) {
      if (stepPulseTicksRemaining_[i] > 0) {
        stepPulseTicksRemaining_[i]--;
      }
      if (stepPulseTicksRemaining_[i] == 0) {
        digitalWrite(JOINT_PINS[i].step, LOW);
        stepPulseHigh_[i] = false;
      }
      continue;
    }

    if (dirSetupTicksRemaining_[i] > 0) {
      dirSetupTicksRemaining_[i]--;
      if (dirSetupTicksRemaining_[i] > 0) {
        continue;
      }
    } else {
      if (stepCountdownTicks_[i] > 0) {
        stepCountdownTicks_[i]--;
        if (stepCountdownTicks_[i] > 0) {
          continue;
        }
      }

      int8_t stepDirection = (delta > 0) ? 1 : -1;
      pendingStepDir_[i] = stepDirection;
      bool dir = stepDirection > 0;
      if (INVERT_DIR[i]) dir = !dir;
      digitalWrite(JOINT_PINS[i].dir, dir ? HIGH : LOW);
      dirSetupTicksRemaining_[i] = dirSetupTicks_;
      stepCountdownTicks_[i] = stepIntervalTicks_[i];

      if (dirSetupTicks_ > 0) {
        continue;
      }
    }

    digitalWrite(JOINT_PINS[i].step, HIGH);
    stepPulseHigh_[i] = true;
    stepPulseTicksRemaining_[i] = pulseWidthTicks_;
    currentSteps_[i] += pendingStepDir_[i];
  }

  if (!anyMoving) {
    isMoving_ = false;
  }
}
