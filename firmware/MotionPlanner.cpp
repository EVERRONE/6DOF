#include "MotionPlanner.h"

MotionPlanner::MotionPlanner() : head_(0), tail_(0), executing_(false) {
  for (int i = 0; i < NUM_AXES; i++) plannerPos_[i] = 0;
  resetLimits();
}

void MotionPlanner::resetLimits() {
  for (int i = 0; i < NUM_AXES; i++) {
    speedLimit_[i] = MAX_JOINT_SPEED[i];
    accelLimit_[i] = MAX_JOINT_ACCEL[i];
  }
}

// Bounded by TUNING_MAX_*, not by the working value: the point of tuning is to
// find out whether the working value is too low, so it cannot be the ceiling.
// A mistyped figure is held to what the hardware could conceivably do.
void MotionPlanner::setSpeedLimit(int axis, float degPerSec) {
  if (axis < 0 || axis >= NUM_AXES) return;
  if (!(degPerSec > 0.0f)) return;
  speedLimit_[axis] =
      degPerSec < TUNING_MAX_SPEED[axis] ? degPerSec : TUNING_MAX_SPEED[axis];
}

void MotionPlanner::setAccelLimit(int axis, float degPerSec2) {
  if (axis < 0 || axis >= NUM_AXES) return;
  if (!(degPerSec2 > 0.0f)) return;
  accelLimit_[axis] =
      degPerSec2 < TUNING_MAX_ACCEL[axis] ? degPerSec2 : TUNING_MAX_ACCEL[axis];
}

float MotionPlanner::speedLimit(int axis) const {
  return (axis >= 0 && axis < NUM_AXES) ? speedLimit_[axis] : 0.0f;
}

float MotionPlanner::accelLimit(int axis) const {
  return (axis >= 0 && axis < NUM_AXES) ? accelLimit_[axis] : 0.0f;
}

void MotionPlanner::reset(const int32_t currentSteps[NUM_AXES]) {
  clear();
  syncPosition(currentSteps);
}

void MotionPlanner::clear() {
  head_ = 0;
  tail_ = 0;
  executing_ = false;
}

void MotionPlanner::syncPosition(const int32_t currentSteps[NUM_AXES]) {
  for (int i = 0; i < NUM_AXES; i++) plannerPos_[i] = currentSteps[i];
}

uint8_t MotionPlanner::queued() const {
  const uint8_t h = head_;
  const uint8_t t = tail_;
  return (uint8_t)((h + MOTION_QUEUE_LENGTH - t) % MOTION_QUEUE_LENGTH);
}

uint8_t MotionPlanner::freeSlots() const {
  return (uint8_t)(MOTION_QUEUE_LENGTH - 1 - queued());
}

MotionBlock* MotionPlanner::currentBlock() {
  if (queued() == 0) return 0;
  return &buffer_[tail_];
}

void MotionPlanner::markExecuting() {
  executing_ = true;
}

void MotionPlanner::completeBlock() {
  if (queued() == 0) {
    executing_ = false;
    return;
  }
  tail_ = nextIndex(tail_);
  executing_ = false;
}

bool MotionPlanner::enqueue(const int32_t targetSteps[NUM_AXES],
                            float requestedSpeedDegPerSec) {
  if (freeSlots() == 0) return false;

  const uint8_t alreadyQueued = queued();
  MotionBlock& b = buffer_[head_];

  uint32_t eventCount = 0;
  float degDelta[NUM_AXES];
  float maxDeg = 0.0f;
  float sumSq = 0.0f;

  for (int i = 0; i < NUM_AXES; i++) {
    const int32_t d = targetSteps[i] - plannerPos_[i];

    b.targetSteps[i] = targetSteps[i];
    b.absDelta[i] = (uint32_t)(d < 0 ? -(int64_t)d : (int64_t)d);
    b.dirSign[i] = (d > 0) ? 1 : ((d < 0) ? -1 : 0);

    if (b.absDelta[i] > eventCount) eventCount = b.absDelta[i];

    degDelta[i] = (float)d / USTEPS_PER_DEG[i];
    const float ad = degDelta[i] < 0.0f ? -degDelta[i] : degDelta[i];
    if (ad > maxDeg) maxDeg = ad;
    sumSq += degDelta[i] * degDelta[i];
  }

  // Already there. Accept the command so the host is not left waiting for an
  // acknowledgement, but do not queue a zero-length block: eventCount == 0
  // would divide by zero in the Bresenham distribution.
  if (eventCount == 0) return true;

  b.eventCount = eventCount;

  const float len = sqrtf(sumSq);
  for (int i = 0; i < NUM_AXES; i++) {
    b.unit[i] = (len > 0.0f) ? (degDelta[i] / len) : 0.0f;
  }

  // Per-axis speed and acceleration caps, mapped into step-event rates.
  //
  // Axis i runs at  eventRate * absDelta[i] / eventCount,  so the event rate
  // that puts axis i exactly at its limit is  limit_i * eventCount / absDelta[i].
  // The binding axis is whichever gives the smallest value.
  float rateCap = 0.0f;
  float accelCap = 0.0f;
  bool haveCap = false;

  for (int i = 0; i < NUM_AXES; i++) {
    if (b.absDelta[i] == 0) continue;

    const float scale = (float)eventCount / (float)b.absDelta[i];
    const float r = speedLimit_[i] * USTEPS_PER_DEG[i] * scale;
    const float a = accelLimit_[i] * USTEPS_PER_DEG[i] * scale;

    if (!haveCap) {
      rateCap = r;
      accelCap = a;
      haveCap = true;
    } else {
      if (r < rateCap) rateCap = r;
      if (a < accelCap) accelCap = a;
    }
  }

  // The requested speed applies to the joint with the largest angular
  // displacement, which is how the J command's speed argument has always been
  // interpreted.
  float requestedRate = rateCap;
  if (requestedSpeedDegPerSec > 0.0f && maxDeg > 0.0f) {
    const float duration = maxDeg / requestedSpeedDegPerSec;
    if (duration > 0.0f) requestedRate = (float)eventCount / duration;
  }

  b.cruiseRate = (requestedRate < rateCap) ? requestedRate : rateCap;

  const float ceilRate = maxEventRate();
  if (b.cruiseRate > ceilRate) b.cruiseRate = ceilRate;
  if (b.cruiseRate < MIN_EVENT_RATE) b.cruiseRate = MIN_EVENT_RATE;

  b.accel = accelCap;
  if (b.accel < 1.0f) b.accel = 1.0f;

  // Junction limit against the previous queued move.
  //
  // Two moves in the same direction can run through their junction at full
  // speed; a reversal has to come to a stop. Scaling by the cosine of the angle
  // between the two direction vectors interpolates between those, is monotone
  // in the turn angle, and never exceeds either move's own cruise rate.
  if (alreadyQueued == 0) {
    // Nothing ahead of this move, so the arm is at rest.
    b.maxEntryRate = 0.0f;
  } else {
    const MotionBlock& p = buffer_[prevIndex(head_)];

    float dot = 0.0f;
    for (int i = 0; i < NUM_AXES; i++) dot += p.unit[i] * b.unit[i];

    const float cap = (p.cruiseRate < b.cruiseRate) ? p.cruiseRate : b.cruiseRate;
    b.maxEntryRate = (dot <= 0.0f) ? MIN_EVENT_RATE : (cap * dot);
    if (b.maxEntryRate < MIN_EVENT_RATE) b.maxEntryRate = MIN_EVENT_RATE;
  }

  b.entryRate = 0.0f;
  b.exitRate = 0.0f;

  for (int i = 0; i < NUM_AXES; i++) plannerPos_[i] = targetSteps[i];

  head_ = nextIndex(head_);

  recalculate();
  return true;
}

/**
 * Re-plan entry and exit rates across the queued moves.
 *
 * Two passes, as in any look-ahead planner:
 *
 *   Reverse - every move must be able to decelerate from its entry rate to its
 *   exit rate within its own length, and the last queued move must be able to
 *   stop. This is the safety-critical direction.
 *
 *   Forward - an entry rate is only real if the preceding move can accelerate up
 *   to it. Without this pass a move could be planned to begin at a speed the arm
 *   never actually reached, which shows up as a step change in velocity.
 */
void MotionPlanner::recalculate() {
  if (queued() == 0) return;

  // The executing block's entry rate is frozen: the ISR is already using it.
  const uint8_t first = executing_ ? nextIndex(tail_) : tail_;
  if (first == head_) return;  // nothing but the running block

  // --- Reverse pass ---------------------------------------------------------
  {
    float nextEntry = 0.0f;  // the last queued move must come to a stop
    uint8_t i = prevIndex(head_);

    for (;;) {
      MotionBlock& b = buffer_[i];
      const float reachable =
          sqrtf(nextEntry * nextEntry + 2.0f * b.accel * (float)b.eventCount);

      b.entryRate = (b.maxEntryRate < reachable) ? b.maxEntryRate : reachable;
      nextEntry = b.entryRate;

      if (i == first) break;
      i = prevIndex(i);
    }
  }

  // --- Forward pass ---------------------------------------------------------
  {
    float available;
    if (executing_) {
      // What the running move can hand over at its end.
      const MotionBlock& r = buffer_[tail_];
      const float reachable =
          sqrtf(r.entryRate * r.entryRate + 2.0f * r.accel * (float)r.eventCount);
      available = (r.cruiseRate < reachable) ? r.cruiseRate : reachable;
    } else {
      available = 0.0f;  // starting from rest
    }

    uint8_t i = first;
    for (;;) {
      MotionBlock& b = buffer_[i];

      if (b.entryRate > available) b.entryRate = available;

      const float reachable =
          sqrtf(b.entryRate * b.entryRate + 2.0f * b.accel * (float)b.eventCount);
      available = (b.cruiseRate < reachable) ? b.cruiseRate : reachable;

      i = nextIndex(i);
      if (i == head_) break;
    }
  }

  // --- Exit rates follow from each successor's entry rate -------------------
  {
    uint8_t i = first;
    for (;;) {
      MotionBlock& b = buffer_[i];
      const uint8_t nx = nextIndex(i);
      b.exitRate = (nx == head_) ? 0.0f : buffer_[nx].entryRate;
      i = nx;
      if (i == head_) break;
    }
  }

  // --- Hand the running move a higher exit rate if one is now available -----
  //
  // Raising it is safe mid-move: a higher exit rate only lifts the deceleration
  // branch of the profile, so the arm decelerates less, and the successor has
  // just been validated to accept exactly this entry rate. Lowering it would not
  // be safe, because the ISR may already be past the point from which a lower
  // exit rate could still be reached, so this only ever increases it.
  if (executing_) {
    const float want = buffer_[first].entryRate;
    MotionBlock& running = buffer_[tail_];
    if (want > running.exitRate) running.exitRate = want;
  }
}
