#ifndef MOTION_PLANNER_H
#define MOTION_PLANNER_H

// Look-ahead motion planner: a queue of coordinated moves, each with a
// trapezoidal velocity profile, planned so that consecutive moves carry speed
// through their junction instead of stopping at every one.
//
// Why this exists: the host streams a trajectory as a series of joint targets.
// The previous firmware held a single target and recomputed a constant-rate
// move on every command, so a 10 Hz stream produced ten full start/stop cycles
// per second on every moving joint. With a queue and junction planning the arm
// runs the whole trajectory as one continuous motion.
//
// Units. Everything internal is in *step events*. One step event advances the
// axis with the largest travel by exactly one microstep; the other axes are
// distributed against it by Bresenham, so the move stays coordinated by
// construction. Rates are step events per second, accelerations step events per
// second squared. Per-joint limits in deg/s and deg/s^2 are converted to event
// rates when a move is queued.
//
// This file is deliberately free of Arduino dependencies so the planning maths
// can be unit-tested on a host. See firmware/test/.

#include <stdint.h>
#include <math.h>
#include "config.h"

/**
 * One queued move.
 *
 * `entryRate` and `exitRate` are filled in by the planner and may change while
 * the block sits in the queue. Once the block starts executing only `exitRate`
 * may still be modified, and only upwards - see MotionPlanner::recalculate.
 */
struct MotionBlock {
  /** Absolute target position, in microsteps. */
  int32_t targetSteps[NUM_AXES];
  /** Travel per axis, in microsteps, unsigned. */
  uint32_t absDelta[NUM_AXES];
  /** Direction per axis: +1, -1, or 0 when the axis does not move. */
  int8_t dirSign[NUM_AXES];
  /** Step events in this block: max(absDelta). Never zero for a queued block. */
  uint32_t eventCount;

  /** Unit vector of the move in joint-degree space, for junction angles. */
  float unit[NUM_AXES];

  /** Top speed for this move, step events/s. */
  float cruiseRate;
  /** Acceleration, step events/s^2. Constant within a block. */
  float accel;
  /** Highest entry rate this junction physically allows. */
  float maxEntryRate;

  /** Planned rate at the first step event. */
  float entryRate;
  /** Planned rate at the last step event. */
  float exitRate;
};

class MotionPlanner {
public:
  MotionPlanner();

  /** Discard the queue and adopt `currentSteps` as the planning origin. */
  void reset(const int32_t currentSteps[NUM_AXES]);

  /**
   * Discard every queued move. The caller must have stopped step generation
   * first - this does not synchronise with the ISR.
   */
  void clear();

  /** Adopt `currentSteps` as the origin for the next queued move. */
  void syncPosition(const int32_t currentSteps[NUM_AXES]);

  /**
   * Queue a move to an absolute target.
   *
   * @param targetSteps absolute target position in microsteps
   * @param requestedSpeedDegPerSec speed of the joint with the largest angular
   *        displacement; clamped to the per-axis limits in config.h. Pass 0 to
   *        run as fast as the limits allow.
   * @return false only if the queue is full. A zero-length move is accepted and
   *         discarded.
   */
  bool enqueue(const int32_t targetSteps[NUM_AXES], float requestedSpeedDegPerSec);

  uint8_t queued() const;
  uint8_t freeSlots() const;
  bool empty() const { return queued() == 0; }

  /** The block at the head of the queue, or null when the queue is empty. */
  MotionBlock* currentBlock();

  /** Mark the head block as under execution, freezing its entry rate. */
  void markExecuting();

  /** Retire the head block and advance to the next. */
  void completeBlock();

  bool isExecuting() const { return executing_; }

  /** Target of the last queued move, i.e. where the arm will end up. */
  const int32_t* plannedPosition() const { return plannerPos_; }

  /**
   * Commanded rate at step `step`, in step events per second.
   *
   * The profile is expressed as velocity against *position* rather than time:
   *
   *   v(s) = min( cruise,
   *               sqrt(entry^2 + 2*a*s),            // still accelerating
   *               sqrt(exit^2  + 2*a*(end - s)) )   // must reach exit by `end`
   *
   * Written this way it needs no phase bookkeeping, is self-correcting, always
   * arrives at the target at no more than the exit rate, and degrades to a
   * triangular profile on a short move without any special case.
   *
   * `endStep` is normally block.eventCount. A decelerate-to-stop abort passes a
   * smaller value to bring the arm to rest early.
   *
   * Takes scalars rather than a block so the step ISR can call it straight from
   * the few values it caches, without touching the queue.
   */
  static float rateAt(float entryRate, float exitRate, float cruiseRate,
                      float accel, uint32_t step, uint32_t endStep) {
    const float remaining = (endStep > step) ? (float)(endStep - step) : 0.0f;
    const float va = sqrtf(entryRate * entryRate + 2.0f * accel * (float)step);
    const float vd = sqrtf(exitRate * exitRate + 2.0f * accel * remaining);

    float v = cruiseRate;
    if (va < v) v = va;
    if (vd < v) v = vd;
    if (v < MIN_EVENT_RATE) v = MIN_EVENT_RATE;
    return v;
  }

  /** Convenience overload for whole blocks, used by tests and diagnostics. */
  static float rateAt(const MotionBlock& b, uint32_t step) {
    return rateAt(b.entryRate, b.exitRate, b.cruiseRate, b.accel, step, b.eventCount);
  }

  /** Highest step-event rate the timer can generate: one high tick, one low. */
  static float maxEventRate() { return (float)STEP_ISR_HZ * 0.5f; }

private:
  MotionBlock buffer_[MOTION_QUEUE_LENGTH];

  // Single-producer (main loop) / single-consumer (step ISR) ring buffer.
  // One slot is always left free so that full and empty stay distinguishable.
  volatile uint8_t head_;  // next free slot, written by the producer
  volatile uint8_t tail_;  // block being executed, advanced by the consumer
  volatile bool executing_;

  /** Target of the last queued move; the origin for the next one. */
  int32_t plannerPos_[NUM_AXES];

  static uint8_t nextIndex(uint8_t i) {
    return (uint8_t)((i + 1) % MOTION_QUEUE_LENGTH);
  }
  static uint8_t prevIndex(uint8_t i) {
    return (uint8_t)((i + MOTION_QUEUE_LENGTH - 1) % MOTION_QUEUE_LENGTH);
  }

  void recalculate();
};

#endif
