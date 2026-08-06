#ifndef STEPPER_CONTROLLER_H
#define STEPPER_CONTROLLER_H

#include "config.h"
#include "types.h"
#include <IntervalTimer.h>

enum StepperMotionMode {
  STEPPER_MOTION_POINT_TO_POINT,
  STEPPER_MOTION_STREAMING
};

struct StreamingMotionCommand {
  JointAngles targetAngles;
  JointAngles targetVelocityDegS;

  StreamingMotionCommand() {}
  StreamingMotionCommand(const JointAngles& target, const JointAngles& velocity)
    : targetAngles(target), targetVelocityDegS(velocity) {}
};

class StepperController {
public:
  StepperController();

  void begin();
  void enable();
  void disable();
  void emergencyStop();
  bool isEnabled() const;

  // Motion control
  void setTargetAngles(const JointAngles& target, float speed);
  void setStreamingCommand(const StreamingMotionCommand& command);
  void setStreamingTargetAngles(const JointAngles& target, float speed);
  void update();  // Call in loop() for step generation
  bool isMoving() const;

  // Position tracking
  JointAngles getCurrentAngles() const;
  JointAngles getTargetAngles() const;
  void setCurrentAngles(const JointAngles& angles);
  float getCurrentRawDegrees(int jointIndex) const;

  // Direct step control (for homing)
  void stepJoint(int jointIndex, int steps, float speed);

  // Stop flag for interrupting blocking operations (homing)
  void requestStop();
  void clearStop();
  bool isStopRequested() const;

  // Calibration management
  void setCalibration(int jointIndex, float scale, float offset);
  JointCalibration getCalibration(int jointIndex) const;
  void setCalibrationAll(const float scale[6], const float offset[6]);
  void resetCalibration();
  bool loadCalibration();
  bool saveCalibration() const;

private:
  JointAngles currentAngles_;
  JointAngles targetAngles_;

  long currentSteps_[6];
  long targetSteps_[6];

  struct MotionProfileState {
    StepperMotionMode mode;
    float requestedSpeedDegS;
    float appliedSpeedDegS;
    unsigned long lastUpdateMicros;
    JointAngles streamingVelocity;  // per-joint velocity for velocity-based step intervals

    MotionProfileState()
      : mode(STEPPER_MOTION_POINT_TO_POINT),
        requestedSpeedDegS(DEFAULT_SPEED),
        appliedSpeedDegS(DEFAULT_SPEED),
        lastUpdateMicros(0) {}
    // streamingVelocity is default-initialised to zeros by JointAngles()
  };

  struct MotionCommandContext {
    StepperMotionMode mode;
    float requestedSpeedDegS;
    bool preserveCountdown;
    bool resetProfileFromFloor;
    bool slewImmediately;
  };

  MotionProfileState motionProfile_;
  bool isMoving_;
  bool isEnabled_;
  volatile bool stopRequested_;
  volatile bool streamingActive_;  // true when streaming with non-zero per-joint velocity

  unsigned long stepInterval_[6];  // microseconds between steps

  JointCalibration calibration_[6];

  IntervalTimer stepTimer_;
  volatile bool stepPulseHigh_[6];
  volatile uint16_t stepPulseTicksRemaining_[6];
  volatile uint16_t dirSetupTicksRemaining_[6];
  volatile uint32_t stepCountdownTicks_[6];
  volatile uint32_t stepIntervalTicks_[6];
  volatile int8_t pendingStepDir_[6];
  uint16_t pulseWidthTicks_;
  uint16_t dirSetupTicks_;
  static StepperController* timerInstance_;
  static void onStepTimerISR();
  void processStepTimerISR();

  void degreesToSteps(const JointAngles& angles, long steps[6]);
  void stepsToDegrees(const long steps[6], JointAngles& angles);
  void calculateStepIntervals();
  void refreshTimerStepIntervals(bool preserveCountdown = false);
  void applyMotionCommand(const JointAngles& target, const MotionCommandContext& context);
  bool captureWasMoving() const;
  float clampCommandSpeed(float speed) const;
  float speedFromVelocityVector(const JointAngles& velocityDegS) const;
  JointAngles clampTargetAngles(const JointAngles& target) const;
  void storeTargetSteps(const long steps[6]);
  bool syncMotionExecutionState(bool preserveCountdown);
  bool slewMotionProfile(unsigned long nowMicros);
  void snapshotStepState(long currentSteps[6], long targetSteps[6], bool* motionActive) const;
  void recomputeAnglesFromSteps();
  float rawDegreesFromSteps(long steps, int jointIndex) const;
};

#endif
