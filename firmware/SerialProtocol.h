#ifndef SERIAL_PROTOCOL_H
#define SERIAL_PROTOCOL_H

#include "config.h"
#include "types.h"
#include "StepperController.h"
#include "HomingController.h"
#include "TrajectoryExecutor.h"

class SerialProtocol {
public:
  SerialProtocol(StepperController& stepper, HomingController& homing, TrajectoryExecutor& trajectory);

  void begin(unsigned long baudRate);
  void update();  // Call in loop()

private:
  StepperController& stepper_;
  HomingController& homing_;
  TrajectoryExecutor& trajectory_;

  String inputBuffer_;
  RobotState currentState_;
  HomePoseRuntimeConfig homePoseConfig_;
  uint32_t ackSequence_;

  void processCommand(String cmd);
  void handleMoveCommand(String cmd);
  void handleHomeCommand(String cmd);
  void handleQueryCommand();
  void handleEnableCommand(String cmd);
  void handleStopCommand();
  void handleConfigCommand(String cmd);
  void handleCalCommand(String cmd);
  void handleJogRelativeCommand(String cmd);
  void handleHomePoseCommand(String cmd);
  void handleTrajectoryQueueCommand(String cmd);
  void handleMotionKernelCommand(String cmd);

  void sendPosition();
  void sendEndstopState();
  void sendConfig();
  void sendHomePoseConfig();
  void sendTrajectoryQueueStatus();
  void sendMotionKernelStatus();
  void sendError(String message);
  void sendAck(const String& scope, const String& detail = "");
  void sendTrajectoryQueueError(const String& code, const String& detail);
  bool executeOperationalHomePose(String& errorMessage);
  bool moveSingleJointBlocking(int jointIndex, float targetDeg, float speedDegS, String& errorMessage);
  bool pollSerialForEmergencyStop();
  bool isHomePoseJointAllowed(int jointIndex) const;
  float clampHomePoseSpeed(float speedDegS) const;
  void resetHomePoseConfig();
  bool loadHomePoseConfig();
  bool saveHomePoseConfig() const;
  bool parseJointIndex(String token, int& jointIndex);
  bool parseCalibrationJson(String json, float scaleOut[6], float offsetOut[6]);
  bool parseFloatAfterKey(String json, int keyPos, float& out);
  unsigned long lastMotionKernelStatusMs_;
};

#endif
