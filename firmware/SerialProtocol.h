#ifndef SERIAL_PROTOCOL_H
#define SERIAL_PROTOCOL_H

#include "config.h"
#include "types.h"
#include "StepperController.h"
#include "HomingController.h"

class SerialProtocol {
public:
  SerialProtocol(StepperController& stepper, HomingController& homing);

  void begin(unsigned long baudRate);
  void update();  // Call in loop()

private:
  StepperController& stepper_;
  HomingController& homing_;

  String inputBuffer_;
  RobotState currentState_;

  void processCommand(String cmd);
  void handleMoveCommand(String cmd);
  void handleHomeCommand(String cmd);
  void handleQueryCommand();
  void handleEnableCommand(String cmd);
  void handleStopCommand();

  void sendPosition();
  void sendEndstopState();
  void sendError(String message);
};

#endif
