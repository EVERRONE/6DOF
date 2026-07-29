#include "StepperController.h"

// digitalWriteFast exists on Teensy; fall back on other cores so this still
// compiles. At 100 kHz the ISR has a 10 us budget and needs at most twelve pin
// writes, so even the slow path fits comfortably.
#ifndef digitalWriteFast
#define digitalWriteFast digitalWrite
#endif

// The timer needs a plain function pointer, so the single controller instance
// registers itself here.
static StepperController* s_instance = 0;

static void stepTimerISR() {
  if (s_instance) s_instance->onTick();
}

StepperController::StepperController()
    : enabled_(false),
      estopped_(false),
      positionTrusted_(false),
      running_(false),
      activeBlock_(0),
      eventCount_(0),
      stopAtStep_(0),
      stepsDone_(0),
      entryRate_(0.0f),
      cruiseRate_(0.0f),
      accel_(1.0f),
      rate_(0.0f),
      accumulator_(0.0f),
      decelStop_(false),
      pulseMask_(0),
      abortComplete_(false),
      pendingSince_(0) {
  for (int i = 0; i < NUM_AXES; i++) {
    currentSteps_[i] = 0;
    absDelta_[i] = 0;
    dirSign_[i] = 0;
    counter_[i] = 0;
    homed_[i] = false;
  }
}

void StepperController::begin() {
  for (int i = 0; i < NUM_AXES; i++) {
    pinMode(JOINT_PINS[i].step, OUTPUT);
    pinMode(JOINT_PINS[i].dir, OUTPUT);
    digitalWriteFast(JOINT_PINS[i].step, LOW);
    digitalWriteFast(JOINT_PINS[i].dir, LOW);
  }

  pinMode(ENABLE_PIN_1, OUTPUT);
  pinMode(ENABLE_PIN_2, OUTPUT);
  disable();  // start with the drivers off for safety

  int32_t zero[NUM_AXES];
  for (int i = 0; i < NUM_AXES; i++) zero[i] = 0;
  planner_.reset(zero);

  s_instance = this;

  // Priority above the USB and serial handlers, so step timing is not pushed
  // around by host traffic. Lower number is higher priority; the default is 128.
  stepTimer_.priority(32);
  stepTimer_.begin(stepTimerISR, 1000000.0f / (float)STEP_ISR_HZ);
}

// ---------------------------------------------------------------------------
// Driver power
// ---------------------------------------------------------------------------

void StepperController::enable() {
  digitalWriteFast(ENABLE_PIN_1, LOW);  // active LOW
  digitalWriteFast(ENABLE_PIN_2, LOW);
  enabled_ = true;
  estopped_ = false;
}

void StepperController::disable() {
  emergencyStop();
  digitalWriteFast(ENABLE_PIN_1, HIGH);
  digitalWriteFast(ENABLE_PIN_2, HIGH);
  enabled_ = false;
}

// ---------------------------------------------------------------------------
// Queueing
// ---------------------------------------------------------------------------

bool StepperController::queueMove(const JointAngles& target, float speed) {
  if (estopped_) return false;

  int32_t steps[NUM_AXES];
  for (int i = 0; i < NUM_AXES; i++) {
    float a = target[i];
    if (a < JOINT_MIN[i]) a = JOINT_MIN[i];
    if (a > JOINT_MAX[i]) a = JOINT_MAX[i];
    steps[i] = (int32_t)lroundf(a * USTEPS_PER_DEG[i]);
  }

  // enqueue() re-plans the queue, which reads the head/tail indices the ISR
  // advances. Masking interrupts for the few microseconds it takes removes the
  // whole interleaving question; it happens once per command, so the added step
  // jitter is a rare few microseconds against a 10 us quantisation.
  noInterrupts();
  const bool ok = planner_.enqueue(steps, speed);
  interrupts();

  return ok;
}

bool StepperController::queueAxisMove(int axis, float deltaDegrees, float speed) {
  if (estopped_) return false;
  if (axis < 0 || axis >= NUM_AXES) return false;

  int32_t target[NUM_AXES];

  noInterrupts();
  const int32_t* base = planner_.plannedPosition();
  for (int i = 0; i < NUM_AXES; i++) target[i] = base[i];
  target[axis] = base[axis] + (int32_t)lroundf(deltaDegrees * USTEPS_PER_DEG[axis]);
  const bool ok = planner_.enqueue(target, speed);
  interrupts();

  return ok;
}

// ---------------------------------------------------------------------------
// Service
// ---------------------------------------------------------------------------

void StepperController::service() {
  // An aborted move leaves the queue to be dropped. The ISR cannot do it
  // itself without racing the producer, so it hands the job over here.
  if (abortComplete_) {
    abortComplete_ = false;
    resyncPlanner();
  }

  if (running_ || estopped_ || !enabled_) return;

  if (planner_.empty()) {
    pendingSince_ = 0;
    return;
  }

  // Give the host a moment to fill the queue, so a streamed trajectory has
  // look-ahead from the very first move and does not have to decelerate to a
  // stop at the end of move one. A single jog command still starts within
  // START_DELAY_MS, which is imperceptible.
  if (pendingSince_ == 0) pendingSince_ = millis();

  const bool deepEnough = planner_.queued() >= START_QUEUE_DEPTH;
  const bool waitedEnough = (millis() - pendingSince_) >= START_DELAY_MS;
  if (!deepEnough && !waitedEnough) return;

  pendingSince_ = 0;

  noInterrupts();
  loadNextBlock();
  interrupts();
}

// ---------------------------------------------------------------------------
// Stopping
// ---------------------------------------------------------------------------

void StepperController::emergencyStop() {
  noInterrupts();

  running_ = false;
  decelStop_ = false;
  activeBlock_ = 0;
  stepsDone_ = 0;
  stopAtStep_ = 0;
  rate_ = 0.0f;
  accumulator_ = 0.0f;

  for (int i = 0; i < NUM_AXES; i++) {
    if (pulseMask_ & (1u << i)) digitalWriteFast(JOINT_PINS[i].step, LOW);
  }
  pulseMask_ = 0;

  int32_t snapshot[NUM_AXES];
  for (int i = 0; i < NUM_AXES; i++) snapshot[i] = currentSteps_[i];
  planner_.clear();
  planner_.syncPosition(snapshot);

  interrupts();

  estopped_ = true;
  pendingSince_ = 0;

  // Cutting the pulses at speed can leave the motor behind its step count.
  positionTrusted_ = false;
}

void StepperController::decelerateToStop() {
  noInterrupts();

  if (running_) {
    // Stop distance at the current rate: s = v^2 / (2a). Two extra events of
    // slack so the profile has somewhere to land.
    const float v = rate_;
    const float a = accel_ > 1.0f ? accel_ : 1.0f;
    uint32_t stopSteps = (uint32_t)(v * v / (2.0f * a)) + 2;

    uint32_t target = stepsDone_ + stopSteps;
    // Never travel past the commanded target of the block: overshooting a
    // position is worse than a slightly firmer stop. The block's own profile
    // already decelerates within its acceleration limit, so the firmest case
    // this can produce is the block's normal deceleration.
    if (target > eventCount_) target = eventCount_;

    stopAtStep_ = target;
    decelStop_ = true;
    rate_ = MotionPlanner::rateAt(entryRate_, 0.0f, cruiseRate_, accel_,
                                  stepsDone_, stopAtStep_);
    interrupts();
    return;
  }

  interrupts();

  // Already stopped: just drop anything queued.
  resyncPlanner();
  pendingSince_ = 0;
}

// ---------------------------------------------------------------------------
// Position
// ---------------------------------------------------------------------------

void StepperController::snapshotSteps(int32_t out[NUM_AXES]) const {
  noInterrupts();
  for (int i = 0; i < NUM_AXES; i++) out[i] = currentSteps_[i];
  interrupts();
}

JointAngles StepperController::currentAngles() const {
  int32_t steps[NUM_AXES];
  snapshotSteps(steps);

  JointAngles angles;
  for (int i = 0; i < NUM_AXES; i++) {
    angles[i] = (float)steps[i] / USTEPS_PER_DEG[i];
  }
  return angles;
}

void StepperController::setJointAngle(int axis, float degrees) {
  if (axis < 0 || axis >= NUM_AXES) return;
  if (running_) return;  // would corrupt the block in flight

  noInterrupts();
  currentSteps_[axis] = (int32_t)lroundf(degrees * USTEPS_PER_DEG[axis]);

  int32_t snapshot[NUM_AXES];
  for (int i = 0; i < NUM_AXES; i++) snapshot[i] = currentSteps_[i];
  planner_.syncPosition(snapshot);
  interrupts();
}

bool StepperController::isHomed(int axis) const {
  if (axis < 0 || axis >= NUM_AXES) return false;
  return homed_[axis];
}

void StepperController::setHomed(int axis, bool homed) {
  if (axis < 0 || axis >= NUM_AXES) return;
  homed_[axis] = homed;
}

void StepperController::resyncPlanner() {
  int32_t snapshot[NUM_AXES];
  snapshotSteps(snapshot);

  noInterrupts();
  planner_.clear();
  planner_.syncPosition(snapshot);
  interrupts();
}

// ---------------------------------------------------------------------------
// Block loading
// ---------------------------------------------------------------------------

float StepperController::liveExitRate() const {
  if (activeBlock_ == 0) return 0.0f;
  // Re-read every step: the planner raises this when it finds more look-ahead.
  return *(const volatile float*)&activeBlock_->exitRate;
}

/** Caller must have interrupts masked, or be the ISR itself. */
bool StepperController::loadNextBlock() {
  MotionBlock* b = planner_.currentBlock();
  if (b == 0) {
    activeBlock_ = 0;
    running_ = false;
    return false;
  }

  activeBlock_ = b;
  eventCount_ = b->eventCount;
  stopAtStep_ = b->eventCount;
  stepsDone_ = 0;
  accumulator_ = 0.0f;
  decelStop_ = false;

  entryRate_ = b->entryRate;
  cruiseRate_ = b->cruiseRate;
  accel_ = b->accel > 1.0f ? b->accel : 1.0f;

  for (int i = 0; i < NUM_AXES; i++) {
    absDelta_[i] = b->absDelta[i];
    dirSign_[i] = b->dirSign[i];
    // Centring the Bresenham phase spreads each axis' steps evenly through the
    // block instead of bunching them at the end. Total counts stay exact.
    counter_[i] = b->eventCount / 2;

    if (b->dirSign[i] != 0) {
      bool forward = (b->dirSign[i] > 0);
      if (INVERT_DIR[i]) forward = !forward;
      // Direction is constant for the whole block and is set here, at least one
      // full timer tick before the first pulse can be raised. That is 10 us of
      // setup time, far beyond the driver's requirement, and it means the ISR
      // never touches a direction pin.
      digitalWriteFast(JOINT_PINS[i].dir, forward ? HIGH : LOW);
    }
  }

  planner_.markExecuting();

  rate_ = MotionPlanner::rateAt(entryRate_, b->exitRate, cruiseRate_, accel_, 0,
                                stopAtStep_);
  running_ = true;
  return true;
}

/** Called from the ISR when the active block reaches its end. */
void StepperController::finishActiveBlock() {
  planner_.completeBlock();

  if (decelStop_) {
    // A graceful abort ends motion here; the main loop drops the rest of the
    // queue, because clearing it from the ISR would race the producer.
    running_ = false;
    decelStop_ = false;
    activeBlock_ = 0;
    rate_ = 0.0f;
    abortComplete_ = true;
    return;
  }

  if (!loadNextBlock()) {
    running_ = false;
    rate_ = 0.0f;
  }
}

// ---------------------------------------------------------------------------
// Timer interrupt
// ---------------------------------------------------------------------------

void StepperController::onTick() {
  // Close out pulses raised on the previous tick. A step therefore spans two
  // ticks, one high and one low, which satisfies the driver's minimum pulse and
  // low times without a single blocking delay, and caps the step rate at
  // STEP_ISR_HZ / 2.
  bool loweredThisTick = false;
  if (pulseMask_ != 0) {
    for (int i = 0; i < NUM_AXES; i++) {
      if (pulseMask_ & (1u << i)) digitalWriteFast(JOINT_PINS[i].step, LOW);
    }
    pulseMask_ = 0;
    loweredThisTick = true;
  }

  if (!running_) return;

  accumulator_ += rate_ * TICK_SECONDS;
  if (accumulator_ < 1.0f) return;

  // Guarantee at least one low tick between pulses. Because the rate is capped
  // at STEP_ISR_HZ / 2 the accumulator cannot reach 1.0 on the tick straight
  // after a step, so in practice this never fires; it is here so that a bad
  // rate can only cost a step, never produce a pulse the driver cannot see.
  if (loweredThisTick) return;

  accumulator_ -= 1.0f;

  // Distribute one step event across the axes. The axis with the most travel
  // steps on every event; the others step in proportion, which is what keeps a
  // multi-axis move on a straight line in joint space.
  uint32_t mask = 0;
  for (int i = 0; i < NUM_AXES; i++) {
    if (absDelta_[i] == 0) continue;

    counter_[i] += absDelta_[i];
    if (counter_[i] >= eventCount_) {
      counter_[i] -= eventCount_;
      digitalWriteFast(JOINT_PINS[i].step, HIGH);
      mask |= (1u << i);
      currentSteps_[i] += dirSign_[i];
    }
  }
  pulseMask_ = mask;

  stepsDone_++;

  if (stepsDone_ >= stopAtStep_) {
    finishActiveBlock();
    return;
  }

  rate_ = MotionPlanner::rateAt(entryRate_, decelStop_ ? 0.0f : liveExitRate(),
                                cruiseRate_, accel_, stepsDone_, stopAtStep_);
}
