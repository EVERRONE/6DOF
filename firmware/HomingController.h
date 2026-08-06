#ifndef HOMING_CONTROLLER_H
#define HOMING_CONTROLLER_H

#include <Arduino.h>
#include "config.h"
#include "types.h"
#include "StepperController.h"

/**
 * Homing, as a non-blocking state machine.
 *
 * update() is called once per main-loop pass and returns immediately, so serial
 * commands keep being processed and position reports keep flowing for the whole
 * homing run. The previous implementation ran homing inside the command handler
 * with busy-wait loops, which froze the entire controller for tens of seconds
 * and needed a hand-rolled serial peek to remain interruptible at all.
 *
 * Each axis is homed in four moves: a fast seek onto the switch, a back-off, a
 * slow second approach for repeatability, and a move to the resting pose. All
 * four go through the normal queued motion path, so they are acceleration
 * limited like any other move.
 */
class HomingController {
public:
  explicit HomingController(StepperController& stepper);

  void begin();

  // --- Endstops -----------------------------------------------------------

  /**
   * Sample the endstop switches. Call every loop pass.
   *
   * Debounced by requiring ENDSTOP_DEBOUNCE_COUNT consecutive agreeing samples.
   * Polling here rather than in the step ISR keeps the interrupt lean; the main
   * loop now runs without any blocking delay, so the latency is a small fraction
   * of a microstep at homing speed.
   */
  void pollEndstops();

  bool isEndstopTriggered(int axis) const;
  EndstopState endstopState() const;

  // --- Homing runs --------------------------------------------------------

  /** Home the listed axes in order. False if already busy or nothing valid. */
  bool start(const uint8_t* axes, uint8_t count);

  /** Home every axis that has an endstop, in ascending order. */
  bool startAll();

  /** Advance the state machine. Call every loop pass. */
  void update();

  /** Stop a run in progress, decelerating first. */
  void abort();

  bool isBusy() const { return phase_ != PHASE_IDLE; }
  bool hasFailed() const { return failed_; }
  const char* lastError() const { return error_; }

  /** True once after a failure, so the caller reports it exactly one time. */
  bool consumeFailure();

  /** Axis being homed right now, or -1 when idle. */
  int currentAxis() const;

  /** True once when a run finishes successfully. */
  bool consumeCompletion();

  /** Axis that just finished homing, or -1. Cleared by reading it. */
  int consumeHomedAxis();

private:
  enum Phase {
    PHASE_IDLE,
    PHASE_BEGIN_AXIS,
    PHASE_SEEK,
    PHASE_SEEK_SETTLE,
    PHASE_BACKOFF,
    PHASE_BACKOFF_SETTLE,
    PHASE_FINE,
    PHASE_FINE_SETTLE,
    PHASE_SET_ZERO,
    PHASE_PARK,
    PHASE_PARK_SETTLE
  };

  StepperController& stepper_;

  uint8_t sequence_[NUM_AXES];
  uint8_t sequenceLength_;
  uint8_t sequenceIndex_;

  Phase phase_;
  bool failed_;
  bool completed_;
  int homedAxis_;
  const char* error_;
  uint32_t phaseStartedMs_;

  uint8_t debounce_[NUM_AXES];
  bool triggered_[NUM_AXES];

  // Back-off retries for the current axis. The switch has to re-open before the
  // slow approach can set a repeatable datum, and how far that takes is
  // mechanical, so the retreat is repeated up to BACKOFF_MAX_DISTANCE.
  uint8_t backoffAttempts_;

  void enterPhase(Phase phase);
  void fail(const char* message);
  void finishAxis();
  bool phaseTimedOut() const;
  int seekDirection(int axis) const;

  /** Longest any single homing phase may take before it is called a failure. */
  static const uint32_t PHASE_TIMEOUT_MS = 90000;
};

#endif
