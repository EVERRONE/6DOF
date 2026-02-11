#ifndef STEPPER_CONTROLLER_H
#define STEPPER_CONTROLLER_H

#include "config.h"
#include "types.h"

class StepperController {
public:
  StepperController();

  void begin();
  void enable();
  void disable();
  void emergencyStop();

  // Motion control
  void setTargetAngles(const JointAngles& target, float speed);
  void update();  // Call in loop() for step generation
  bool isMoving() const;

  // Position tracking
  JointAngles getCurrentAngles() const;
  void setCurrentAngles(const JointAngles& angles);

  // Direct step control (for homing)
  void stepJoint(int jointIndex, int steps, float speed);

  // Stop flag for interrupting blocking operations (homing)
  void requestStop();
  void clearStop();
  bool isStopRequested() const;

private:
  JointAngles currentAngles_;
  JointAngles targetAngles_;

  long currentSteps_[6];
  long targetSteps_[6];

  float speed_;
  bool isMoving_;
  bool isEnabled_;
  volatile bool stopRequested_;

  unsigned long lastStepTime_[6];
  unsigned long stepInterval_[6];  // microseconds between steps

  void degreesToSteps(const JointAngles& angles, long steps[6]);
  void stepsToDegrees(const long steps[6], JointAngles& angles);
  void calculateStepIntervals();
  void generateSteps();
};

#endif
