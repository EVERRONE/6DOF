#include "SerialProtocol.h"

SerialProtocol::SerialProtocol(StepperController& stepper, HomingController& homing)
  : stepper_(stepper), homing_(homing), currentState_(STATE_IDLE) {}

void SerialProtocol::begin(unsigned long baudRate) {
  Serial.begin(baudRate);
  while (!Serial && millis() < 3000);  // Wait up to 3s for serial
  Serial.println("OK Robot arm ready");
}

void SerialProtocol::update() {
  // Read incoming serial data
  while (Serial.available()) {
    char c = Serial.read();

    if (c == '\n') {
      processCommand(inputBuffer_);
      inputBuffer_ = "";
    } else if (c != '\r') {
      inputBuffer_ += c;
    }
  }

  // Send periodic position updates (every 100ms)
  static unsigned long lastUpdate = 0;
  if (millis() - lastUpdate > 100) {
    sendPosition();
    sendEndstopState();
    lastUpdate = millis();
  }
}

void SerialProtocol::processCommand(String cmd) {
  cmd.trim();
  if (cmd.length() == 0) return;

  char commandType = toupper(cmd.charAt(0));

  switch (commandType) {
    case 'J':  // Joint move: J <j1> <j2> <j3> <j4> <j5> <j6> <speed>
      handleMoveCommand(cmd);
      break;

    case 'H':  // Home: H <joints> (e.g., "H 2345" or "H ALL")
      handleHomeCommand(cmd);
      break;

    case 'Q':  // Query position
      handleQueryCommand();
      break;

    case 'E':  // Enable: E 1 (enable) or E 0 (disable)
      handleEnableCommand(cmd);
      break;

    case 'S':  // Emergency stop
      handleStopCommand();
      break;

    default:
      sendError("Unknown command");
  }
}

void SerialProtocol::handleMoveCommand(String cmd) {
  // Parse: J <j1> <j2> <j3> <j4> <j5> <j6> <speed>
  float values[7];
  int index = 0;
  int startPos = 2;  // Skip "J "

  while (startPos < cmd.length() && index < 7) {
    int endPos = cmd.indexOf(' ', startPos);
    if (endPos == -1) endPos = cmd.length();

    String valueStr = cmd.substring(startPos, endPos);
    values[index++] = valueStr.toFloat();

    startPos = endPos + 1;
  }

  if (index != 7) {
    sendError("Invalid move command format");
    return;
  }

  JointAngles target;
  for (int i = 0; i < 6; i++) {
    target[i] = values[i];
  }
  float speed = values[6];

  stepper_.setTargetAngles(target, speed);
  currentState_ = STATE_MOVING;

  Serial.println("OK Moving");
}

void SerialProtocol::handleHomeCommand(String cmd) {
  String joints = cmd.substring(2);
  joints.trim();

  currentState_ = STATE_HOMING;

  if (joints == "ALL") {
    if (homing_.homeAll()) {
      currentState_ = STATE_IDLE;
    } else {
      currentState_ = STATE_ERROR;
    }
  } else {
    // Home specific joints: "H 2345"
    for (unsigned int i = 0; i < joints.length(); i++) {
      int joint = joints.charAt(i) - '1';  // Convert '2' -> 1 (0-indexed)
      if (joint >= 0 && joint < 6) {
        if (!homing_.homeJoint(joint)) {
          currentState_ = STATE_ERROR;
          return;
        }
      }
    }
    currentState_ = STATE_IDLE;
  }
}

void SerialProtocol::handleQueryCommand() {
  sendPosition();
}

void SerialProtocol::handleEnableCommand(String cmd) {
  String value = cmd.substring(2);
  value.trim();

  if (value == "1") {
    stepper_.enable();
    Serial.println("OK Motors enabled");
  } else {
    stepper_.disable();
    Serial.println("OK Motors disabled");
  }
}

void SerialProtocol::handleStopCommand() {
  stepper_.emergencyStop();
  currentState_ = STATE_ESTOPPED;
  Serial.println("OK Emergency stop");
}

void SerialProtocol::sendPosition() {
  JointAngles current = stepper_.getCurrentAngles();

  Serial.print("POS ");
  for (int i = 0; i < 6; i++) {
    Serial.print(current[i], 2);
    if (i < 5) Serial.print(" ");
  }
  Serial.println();
}

void SerialProtocol::sendEndstopState() {
  EndstopState state = homing_.getEndstopState();

  Serial.print("ENDSTOP ");
  for (int i = 0; i < 6; i++) {
    Serial.print(state.triggered[i] ? "1" : "0");
    if (i < 5) Serial.print(" ");
  }
  Serial.println();
}

void SerialProtocol::sendError(String message) {
  Serial.print("ERROR ");
  Serial.println(message);
}
