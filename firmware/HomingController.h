#ifndef HOMING_CONTROLLER_H
#define HOMING_CONTROLLER_H

#include "config.h"
#include "types.h"
#include "StepperController.h"

class HomingController {
public:
  HomingController(StepperController& stepper);

  void begin();
  bool homeJoint(int jointIndex);
  bool homeAll();

  bool isEndstopTriggered(int jointIndex);
  EndstopState getEndstopState();

private:
  StepperController& stepper_;
  bool lastRawState_[6];
  bool debouncedState_[6];
  unsigned long lastChangeMs_[6];

  bool findEndstop(int jointIndex);
  void backOff(int jointIndex);
  void fineApproach(int jointIndex);
  void checkSerialForStop();
  void updateEndstopDebounce(int jointIndex);
};

#endif
