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

  bool findEndstop(int jointIndex);
  void backOff(int jointIndex);
  void fineApproach(int jointIndex);
  void checkSerialForStop();
};

#endif
