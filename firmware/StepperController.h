#ifndef STEPPER_CONTROLLER_H
#define STEPPER_CONTROLLER_H

#include <Arduino.h>
#include "config.h"
#include "types.h"
#include "MotionPlanner.h"

/**
 * Step pulse generation and position tracking.
 *
 * Steps are produced from a fixed-frequency timer interrupt (STEP_ISR_HZ), not
 * from the main loop. The interrupt advances a phase accumulator at the rate the
 * velocity profile asks for, and distributes the step across axes by Bresenham
 * against the axis with the most travel, so a multi-axis move stays coordinated
 * to within one microstep.
 *
 * The main loop only queues moves and calls service(); it never generates a
 * pulse and never blocks on one. That is deliberate: the previous version
 * stepped from loop() with blocking delayMicroseconds() calls, which put
 * +/-27-53% of a step period of jitter on every pulse.
 */
class StepperController {
public:
  StepperController();

  void begin();

  // --- Driver power --------------------------------------------------------

  /** Energise the drivers. Also clears a latched emergency stop. */
  void enable();

  /**
   * De-energise the drivers. The arm is then back-driveable, so on a geared arm
   * this will let it sag under its own weight.
   */
  void disable();

  bool isEnabled() const { return enabled_; }

  // --- Motion -------------------------------------------------------------

  /**
   * Queue an absolute move. Angles are clamped to JOINT_MIN/JOINT_MAX.
   *
   * @param speed speed of the joint with the largest angular displacement, in
   *              deg/s; clamped to the per-axis limits. Pass 0 for "as fast as
   *              allowed".
   * @return false if the queue is full, so the caller can apply back-pressure.
   */
  bool queueMove(const JointAngles& target, float speed);

  /**
   * Queue a single-axis relative move, measured from the end of whatever is
   * already queued.
   *
   * Deliberately NOT clamped to the joint limits: this is what homing uses, and
   * before the arm is homed its position is unknown, so clamping would stop it
   * ever reaching the endstop. The endstop itself is the limit during homing.
   */
  bool queueAxisMove(int axis, float deltaDegrees, float speed);

  /** Start queued moves, and finish off any pending abort. Call from loop(). */
  void service();

  bool isMoving() const { return running_; }
  /**
   * Motion limits, adjustable at runtime for tuning by ear.
   *
   * Exposed through the controller rather than handing the planner out, so the
   * queue stays its own business.
   */
  void setSpeedLimit(int axis, float degPerSec) { planner_.setSpeedLimit(axis, degPerSec); }
  void setAccelLimit(int axis, float degPerSec2) { planner_.setAccelLimit(axis, degPerSec2); }
  float speedLimit(int axis) const { return planner_.speedLimit(axis); }
  float accelLimit(int axis) const { return planner_.accelLimit(axis); }
  void resetMotionLimits() { planner_.resetLimits(); }

  uint8_t queueFree() const { return planner_.freeSlots(); }
  uint8_t queueDepth() const { return planner_.queued(); }

  /** True once every queued move has been executed and the arm is stopped. */
  bool isIdle() const { return !running_ && planner_.empty(); }

  // --- Stopping -----------------------------------------------------------

  /**
   * Emergency stop: cut step generation immediately and drop the queue.
   *
   * The drivers stay energised, because de-energising them would let a geared
   * arm fall. Stopping from speed without a ramp can lose steps, so this marks
   * the recorded position as untrusted and the arm should be re-homed.
   */
  void emergencyStop();

  /**
   * Graceful stop: decelerate to a halt using the normal acceleration limit,
   * then drop the queue. Position stays trusted. This is what homing and a
   * user-requested cancel should use.
   */
  void decelerateToStop();

  bool isEmergencyStopped() const { return estopped_; }

  // --- Position -----------------------------------------------------------

  /**
   * Live joint angles, valid while moving. Reads a consistent snapshot of the
   * step counters with interrupts briefly masked.
   */
  JointAngles currentAngles() const;

  void snapshotSteps(int32_t out[NUM_AXES]) const;

  /**
   * Define the current angle of ONE axis, leaving the other five untouched.
   *
   * Homing uses this to set its zero. The previous firmware assigned a whole
   * JointAngles of zeros instead, which wiped the calibration of every other
   * axis - so homing four joints in sequence left three of them recorded as 0
   * while physically parked at 5, 55 and 129 degrees, and the next ordinary move
   * drove them straight past their limits.
   *
   * Ignored while a move is in progress, which would corrupt the running block.
   */
  void setJointAngle(int axis, float degrees);

  /**
   * False when a hard stop may have lost steps, so the recorded position cannot
   * be relied on until the arm is re-homed.
   */
  bool positionTrusted() const { return positionTrusted_; }
  void setPositionTrusted(bool trusted) { positionTrusted_ = trusted; }

  /** Per-axis: has this axis been homed since power-up? */
  bool isHomed(int axis) const;
  void setHomed(int axis, bool homed);

  // --- Timer interrupt ----------------------------------------------------

  /** Called from the timer ISR. Not part of the public API. */
  void onTick();

private:
  MotionPlanner planner_;
  IntervalTimer stepTimer_;

  // Cached pin state so the ISR does not recompute it.
  bool enabled_;
  volatile bool estopped_;
  volatile bool positionTrusted_;
  bool homed_[NUM_AXES];

  // --- State shared with the ISR ------------------------------------------

  volatile bool running_;
  volatile int32_t currentSteps_[NUM_AXES];

  // Active block, copied out at load time so the ISR does not chase pointers.
  // exitRate is the exception: the planner may raise it mid-move, so the ISR
  // re-reads it from the block every step.
  MotionBlock* activeBlock_;
  volatile uint32_t absDelta_[NUM_AXES];
  volatile int8_t dirSign_[NUM_AXES];
  volatile uint32_t counter_[NUM_AXES];
  volatile uint32_t eventCount_;
  volatile uint32_t stopAtStep_;
  volatile uint32_t stepsDone_;
  volatile float entryRate_;
  volatile float cruiseRate_;
  volatile float accel_;
  volatile float rate_;
  volatile float accumulator_;
  volatile bool decelStop_;
  volatile uint32_t pulseMask_;
  volatile bool abortComplete_;

  // Look-ahead start delay.
  uint32_t pendingSince_;

  bool loadNextBlock();
  void finishActiveBlock();
  void resyncPlanner();
  float liveExitRate() const;

  static constexpr float TICK_SECONDS = 1.0f / (float)STEP_ISR_HZ;
};

#endif
