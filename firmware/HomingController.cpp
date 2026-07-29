#include "HomingController.h"

HomingController::HomingController(StepperController& stepper)
    : stepper_(stepper),
      sequenceLength_(0),
      sequenceIndex_(0),
      phase_(PHASE_IDLE),
      failed_(false),
      completed_(false),
      homedAxis_(-1),
      error_(""),
      phaseStartedMs_(0) {
  for (int i = 0; i < NUM_AXES; i++) {
    sequence_[i] = 0;
    debounce_[i] = 0;
    triggered_[i] = false;
  }
}

void HomingController::begin() {
  for (int i = 0; i < NUM_AXES; i++) {
    if (HAS_ENDSTOP[i] && ENDSTOP_PINS[i] != ENDSTOP_NONE) {
      pinMode(ENDSTOP_PINS[i], INPUT_PULLUP);
    }
  }
}

// ---------------------------------------------------------------------------
// Endstops
// ---------------------------------------------------------------------------

void HomingController::pollEndstops() {
  for (int i = 0; i < NUM_AXES; i++) {
    if (!HAS_ENDSTOP[i] || ENDSTOP_PINS[i] == ENDSTOP_NONE) {
      triggered_[i] = false;
      continue;
    }

    const bool raw = (digitalRead(ENDSTOP_PINS[i]) == LOW);  // active LOW

    if (raw) {
      if (debounce_[i] < ENDSTOP_DEBOUNCE_COUNT) {
        debounce_[i]++;
        if (debounce_[i] >= ENDSTOP_DEBOUNCE_COUNT) triggered_[i] = true;
      }
    } else {
      debounce_[i] = 0;
      triggered_[i] = false;
    }
  }
}

bool HomingController::isEndstopTriggered(int axis) const {
  if (axis < 0 || axis >= NUM_AXES) return false;
  return triggered_[axis];
}

EndstopState HomingController::endstopState() const {
  EndstopState state;
  for (int i = 0; i < NUM_AXES; i++) state.triggered[i] = triggered_[i];
  return state;
}

// ---------------------------------------------------------------------------
// Run control
// ---------------------------------------------------------------------------

int HomingController::currentAxis() const {
  if (phase_ == PHASE_IDLE || sequenceIndex_ >= sequenceLength_) return -1;
  return (int)sequence_[sequenceIndex_];
}

bool HomingController::start(const uint8_t* axes, uint8_t count) {
  if (isBusy()) return false;
  if (!stepper_.isEnabled() || stepper_.isEmergencyStopped()) return false;

  sequenceLength_ = 0;
  for (uint8_t i = 0; i < count && sequenceLength_ < NUM_AXES; i++) {
    const uint8_t axis = axes[i];
    if (axis >= NUM_AXES) return false;
    if (!HAS_ENDSTOP[axis]) return false;
    sequence_[sequenceLength_++] = axis;
  }

  if (sequenceLength_ == 0) return false;

  sequenceIndex_ = 0;
  failed_ = false;
  completed_ = false;
  homedAxis_ = -1;
  error_ = "";
  enterPhase(PHASE_BEGIN_AXIS);
  return true;
}

bool HomingController::startAll() {
  uint8_t axes[NUM_AXES];
  uint8_t count = 0;
  for (uint8_t i = 0; i < NUM_AXES; i++) {
    if (HAS_ENDSTOP[i]) axes[count++] = i;
  }
  return start(axes, count);
}

void HomingController::abort() {
  if (!isBusy()) return;
  stepper_.decelerateToStop();
  phase_ = PHASE_IDLE;
  sequenceLength_ = 0;
  sequenceIndex_ = 0;
}

bool HomingController::consumeCompletion() {
  const bool value = completed_;
  completed_ = false;
  return value;
}

int HomingController::consumeHomedAxis() {
  const int value = homedAxis_;
  homedAxis_ = -1;
  return value;
}

bool HomingController::consumeFailure() {
  const bool value = failed_;
  failed_ = false;
  return value;
}

void HomingController::enterPhase(Phase phase) {
  phase_ = phase;
  phaseStartedMs_ = millis();
}

void HomingController::fail(const char* message) {
  error_ = message;
  failed_ = true;
  phase_ = PHASE_IDLE;
  sequenceLength_ = 0;
  sequenceIndex_ = 0;
  stepper_.decelerateToStop();
}

bool HomingController::phaseTimedOut() const {
  return (millis() - phaseStartedMs_) > PHASE_TIMEOUT_MS;
}

int HomingController::seekDirection(int axis) const {
  return HOME_TOWARD_MIN[axis] ? -1 : 1;
}

void HomingController::finishAxis() {
  sequenceIndex_++;
  if (sequenceIndex_ >= sequenceLength_) {
    phase_ = PHASE_IDLE;
    sequenceLength_ = 0;
    sequenceIndex_ = 0;
    completed_ = true;
    return;
  }
  enterPhase(PHASE_BEGIN_AXIS);
}

// ---------------------------------------------------------------------------
// State machine
// ---------------------------------------------------------------------------

void HomingController::update() {
  if (phase_ == PHASE_IDLE) return;

  if (stepper_.isEmergencyStopped()) {
    fail("Emergency stop during homing");
    return;
  }

  const int axis = currentAxis();
  if (axis < 0) {
    phase_ = PHASE_IDLE;
    return;
  }

  if (phaseTimedOut()) {
    fail("Homing phase timed out");
    return;
  }

  switch (phase_) {
    case PHASE_BEGIN_AXIS: {
      // If the switch is already made, skip straight to backing off it. Seeking
      // into a switch that is already closed would drive into the hard stop.
      if (isEndstopTriggered(axis)) {
        enterPhase(PHASE_BACKOFF);
        return;
      }
      // A long move toward the switch. It is deliberately not clamped to the
      // joint limits, because before homing the recorded position is arbitrary;
      // the switch is the limit here.
      if (stepper_.queueAxisMove(axis, seekDirection(axis) * HOMING_MAX_TRAVEL,
                                 HOMING_SPEED)) {
        enterPhase(PHASE_SEEK);
      }
      return;
    }

    case PHASE_SEEK: {
      if (isEndstopTriggered(axis)) {
        // Decelerate rather than cut the pulses: at HOMING_SPEED the overtravel
        // is well under a degree, and BACKOFF_DISTANCE covers it.
        stepper_.decelerateToStop();
        enterPhase(PHASE_SEEK_SETTLE);
        return;
      }
      if (stepper_.isIdle()) {
        fail("Endstop not found within travel limit");
      }
      return;
    }

    case PHASE_SEEK_SETTLE: {
      if (stepper_.isIdle()) enterPhase(PHASE_BACKOFF);
      return;
    }

    case PHASE_BACKOFF: {
      if (stepper_.queueAxisMove(axis, -seekDirection(axis) * BACKOFF_DISTANCE,
                                 HOMING_SPEED)) {
        enterPhase(PHASE_BACKOFF_SETTLE);
      }
      return;
    }

    case PHASE_BACKOFF_SETTLE: {
      if (!stepper_.isIdle()) return;
      if (isEndstopTriggered(axis)) {
        // Still closed after retracting: the switch is stuck or miswired.
        // Continuing would drive the joint into its hard stop.
        fail("Endstop still closed after back-off");
        return;
      }
      enterPhase(PHASE_FINE);
      return;
    }

    case PHASE_FINE: {
      // Second, slow approach. This is what sets the repeatability of the datum,
      // so it runs at HOMING_FINE_SPEED and travels only a little further than
      // the back-off distance.
      if (stepper_.queueAxisMove(axis, seekDirection(axis) * BACKOFF_DISTANCE * 2.0f,
                                 HOMING_FINE_SPEED)) {
        enterPhase(PHASE_FINE_SETTLE);
      }
      return;
    }

    case PHASE_FINE_SETTLE: {
      if (isEndstopTriggered(axis)) {
        stepper_.decelerateToStop();
        enterPhase(PHASE_SET_ZERO);
        return;
      }
      if (stepper_.isIdle()) {
        fail("Endstop not found on fine approach");
      }
      return;
    }

    case PHASE_SET_ZERO: {
      if (!stepper_.isIdle()) return;

      // The datum. Only THIS axis is touched: the old code assigned a whole
      // JointAngles of zeros here, which discarded the calibration of every
      // other axis and left the arm driving past its limits on the next move.
      stepper_.setJointAngle(axis, HOME_POSITION[axis]);
      stepper_.setHomed(axis, true);

      // Every axis homed so far now has a datum, so the recorded position is
      // meaningful again.
      bool allHomed = true;
      for (int i = 0; i < NUM_AXES; i++) {
        if (HAS_ENDSTOP[i] && !stepper_.isHomed(i)) allHomed = false;
      }
      if (allHomed) stepper_.setPositionTrusted(true);

      enterPhase(PHASE_PARK);
      return;
    }

    case PHASE_PARK: {
      const float delta = POST_HOME_ANGLES[axis] - HOME_POSITION[axis];
      if (delta == 0.0f) {
        finishAxis();
        return;
      }
      if (stepper_.queueAxisMove(axis, delta, HOMING_SPEED)) {
        enterPhase(PHASE_PARK_SETTLE);
      }
      return;
    }

    case PHASE_PARK_SETTLE: {
      if (stepper_.isIdle()) {
        homedAxis_ = axis;
        finishAxis();
      }
      return;
    }

    case PHASE_IDLE:
    default:
      return;
  }
}
