#include "SerialProtocol.h"
#include <EEPROM.h>
#include <math.h>

SerialProtocol::SerialProtocol(
  StepperController& stepper,
  HomingController& homing,
  TrajectoryExecutor& trajectory
)
  : stepper_(stepper),
    homing_(homing),
    trajectory_(trajectory),
    currentState_(STATE_IDLE),
    ackSequence_(0),
    lastMotionKernelStatusMs_(0) {
  resetHomePoseConfig();
}

void SerialProtocol::begin(unsigned long baudRate) {
  Serial.begin(baudRate);
  while (!Serial && millis() < 3000);  // Wait up to 3s for serial

  if (!loadHomePoseConfig()) {
    resetHomePoseConfig();
  }

  Serial.println("OK Robot arm ready");
}

void SerialProtocol::update() {
  trajectory_.update();

  uint16_t progressIndex = 0;
  uint32_t elapsedMs = 0;
  if (trajectory_.consumeProgressEvent(progressIndex, elapsedMs)) {
    Serial.print("TQ PROG ");
    Serial.print(progressIndex);
    Serial.print(" ");
    Serial.println(elapsedMs);
  }

  if (trajectory_.consumeDoneEvent()) {
    Serial.println("TQ DONE");
    currentState_ = STATE_IDLE;
  }

  if (millis() - lastMotionKernelStatusMs_ > 500) {
    sendMotionKernelStatus();
    lastMotionKernelStatusMs_ = millis();
  }

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

  if (cmd == "CFG?") {
    sendConfig();
    return;
  }

  if (cmd == "HP?") {
    sendHomePoseConfig();
    return;
  }

  if (cmd == "TQ?") {
    sendTrajectoryQueueStatus();
    return;
  }

  if (cmd == "MQ?") {
    sendMotionKernelStatus();
    return;
  }

  int firstSpace = cmd.indexOf(' ');
  String token = (firstSpace == -1) ? cmd : cmd.substring(0, firstSpace);
  String args = (firstSpace == -1) ? "" : cmd.substring(firstSpace + 1);
  token.toUpperCase();

  if (token == "CFG") {
    handleConfigCommand(args);
    return;
  }

  if (token == "CAL") {
    handleCalCommand(args);
    return;
  }

  if (token == "JR") {
    handleJogRelativeCommand(args);
    return;
  }

  if (token == "HP") {
    handleHomePoseCommand(args);
    return;
  }

  if (token == "TQ") {
    handleTrajectoryQueueCommand(args);
    return;
  }

  if (token == "MQ") {
    handleMotionKernelCommand(args);
    return;
  }

  char commandType = toupper(token.charAt(0));

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
  trajectory_.stop();

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
  trajectory_.stop();

  String joints = cmd.substring(2);
  joints.trim();
  joints.toUpperCase();

  currentState_ = STATE_HOMING;

  if (joints == "ALL") {
    if (!homing_.homeAll()) {
      currentState_ = STATE_ERROR;
      return;
    }

    if (homePoseConfig_.enabled && HOMEPOSE_APPLY_AFTER_HALL) {
      String errorMessage;
      if (!executeOperationalHomePose(errorMessage)) {
        currentState_ = STATE_ERROR;
        sendError(String("HOMEPOSE ") + errorMessage);
        return;
      }
      Serial.println("HOMEPOSE_REACHED");
    }

    currentState_ = STATE_IDLE;
    Serial.println("OK All joints homed");
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
    trajectory_.stop();
    stepper_.disable();
    Serial.println("OK Motors disabled");
  }
}

void SerialProtocol::handleStopCommand() {
  trajectory_.emergencyStop();
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

void SerialProtocol::sendAck(const String& scope, const String& detail) {
  if (!CAP_COMMAND_ACK_V1) return;
  ackSequence_++;
  Serial.print("ACK ");
  Serial.print(ackSequence_);
  Serial.print(" ");
  Serial.print(scope);
  if (detail.length() > 0) {
    Serial.print(" ");
    Serial.print(detail);
  }
  Serial.println();
}

void SerialProtocol::sendTrajectoryQueueError(const String& code, const String& detail) {
  if (CAP_TRAJECTORY_ERROR_CODES_V1) {
    Serial.print("TQ ERR ");
    Serial.print(code);
    if (detail.length() > 0) {
      Serial.print(" ");
      Serial.print(detail);
    }
    Serial.println();
  }
  sendError(String("TQ ") + detail);
}

void SerialProtocol::handleConfigCommand(String cmd) {
  cmd.trim();
  if (cmd.length() == 0) {
    sendError("CFG missing payload");
    return;
  }

  float scale[6];
  float offset[6];
  for (int i = 0; i < 6; i++) {
    scale[i] = 1.0f;
    offset[i] = 0.0f;
  }

  if (!parseCalibrationJson(cmd, scale, offset)) {
    sendError("CFG parse failed");
    return;
  }

  stepper_.setCalibrationAll(scale, offset);
  Serial.println("OK CFG updated");
}

void SerialProtocol::handleCalCommand(String cmd) {
  cmd.trim();
  if (cmd.length() == 0) {
    sendError("CAL missing subcommand");
    return;
  }

  int space = cmd.indexOf(' ');
  String sub = (space == -1) ? cmd : cmd.substring(0, space);
  String args = (space == -1) ? "" : cmd.substring(space + 1);
  sub.toUpperCase();

  if (sub == "SAVE") {
    stepper_.saveCalibration();
    Serial.println("OK CAL saved");
    return;
  }

  if (sub == "LOAD") {
    if (stepper_.loadCalibration()) {
      Serial.println("OK CAL loaded");
    } else {
      stepper_.resetCalibration();
      Serial.println("OK CAL defaults");
    }
    return;
  }

  if (sub == "RESET") {
    stepper_.resetCalibration();
    Serial.println("OK CAL reset");
    return;
  }

  if (sub == "SET") {
    args.trim();
    int first = args.indexOf(' ');
    int second = args.indexOf(' ', first + 1);
    if (first == -1 || second == -1) {
      sendError("CAL SET format");
      return;
    }

    String jointToken = args.substring(0, first);
    String scaleStr = args.substring(first + 1, second);
    String offsetStr = args.substring(second + 1);
    int jointIndex = -1;
    if (!parseJointIndex(jointToken, jointIndex)) {
      sendError("CAL SET joint");
      return;
    }

    float scale = scaleStr.toFloat();
    float offset = offsetStr.toFloat();
    stepper_.setCalibration(jointIndex, scale, offset);
    Serial.println("OK CAL set");
    return;
  }

  if (sub == "ZERO") {
    args.trim();
    int first = args.indexOf(' ');
    if (first == -1) {
      sendError("CAL ZERO format");
      return;
    }

    String jointToken = args.substring(0, first);
    String logicalStr = args.substring(first + 1);
    int jointIndex = -1;
    if (!parseJointIndex(jointToken, jointIndex)) {
      sendError("CAL ZERO joint");
      return;
    }

    float logicalDeg = logicalStr.toFloat();
    JointCalibration cal = stepper_.getCalibration(jointIndex);
    float rawDeg = stepper_.getCurrentRawDegrees(jointIndex);
    float offset = logicalDeg - (rawDeg * cal.scale);
    stepper_.setCalibration(jointIndex, cal.scale, offset);
    Serial.println("OK CAL zero");
    return;
  }

  sendError("CAL unknown");
}

void SerialProtocol::handleJogRelativeCommand(String cmd) {
  cmd.trim();
  if (cmd.length() == 0) {
    sendError("JR format");
    return;
  }

  int first = cmd.indexOf(' ');
  int second = cmd.indexOf(' ', first + 1);
  if (first == -1 || second == -1) {
    sendError("JR format");
    return;
  }

  String jointToken = cmd.substring(0, first);
  String deltaStr = cmd.substring(first + 1, second);
  String speedStr = cmd.substring(second + 1);

  int jointIndex = -1;
  if (!parseJointIndex(jointToken, jointIndex)) {
    sendError("JR joint");
    return;
  }

  float deltaDeg = deltaStr.toFloat();
  float speed = speedStr.toFloat();

  JointAngles current = stepper_.getCurrentAngles();
  JointAngles target = current;
  target[jointIndex] = current[jointIndex] + deltaDeg;
  stepper_.setTargetAngles(target, speed);
  currentState_ = STATE_MOVING;

  Serial.println("OK Jog");
}

void SerialProtocol::handleHomePoseCommand(String cmd) {
  cmd.trim();
  if (cmd.length() == 0) {
    sendError("HP missing subcommand");
    return;
  }

  int space = cmd.indexOf(' ');
  String sub = (space == -1) ? cmd : cmd.substring(0, space);
  String args = (space == -1) ? "" : cmd.substring(space + 1);
  sub.toUpperCase();

  if (sub == "EN") {
    args.trim();
    if (args != "0" && args != "1") {
      sendError("HP EN expects 0 or 1");
      return;
    }
    homePoseConfig_.enabled = (args == "1");
    Serial.print("OK HP ");
    Serial.println(homePoseConfig_.enabled ? "enabled" : "disabled");
    return;
  }

  if (sub == "SPD") {
    args.trim();
    if (args.length() == 0) {
      sendError("HP SPD missing speed");
      return;
    }

    float speed = clampHomePoseSpeed(args.toFloat());
    homePoseConfig_.speedDegS = speed;
    Serial.println("OK HP speed");
    return;
  }

  if (sub == "SET") {
    args.trim();
    int first = args.indexOf(' ');
    if (first == -1) {
      sendError("HP SET format");
      return;
    }

    String jointToken = args.substring(0, first);
    String angleStr = args.substring(first + 1);
    int jointIndex = -1;
    if (!parseJointIndex(jointToken, jointIndex)) {
      sendError("HP SET joint");
      return;
    }
    if (!isHomePoseJointAllowed(jointIndex)) {
      sendError("HP SET joint must be J2..J5");
      return;
    }

    float target = angleStr.toFloat();
    if (target < JOINT_MIN[jointIndex] || target > JOINT_MAX[jointIndex]) {
      sendError("HP SET out of range");
      return;
    }

    homePoseConfig_.jointsDeg[jointIndex] = target;
    Serial.println("OK HP set");
    return;
  }

  if (sub == "SETALL") {
    args.trim();
    float values[4];
    int idx = 0;
    int start = 0;

    while (start < args.length() && idx < 4) {
      int end = args.indexOf(' ', start);
      if (end == -1) end = args.length();
      values[idx++] = args.substring(start, end).toFloat();
      start = end + 1;
      while (start < args.length() && args.charAt(start) == ' ') start++;
    }

    if (idx != 4) {
      sendError("HP SETALL format");
      return;
    }

    const int homeIndices[4] = {1, 2, 3, 4};
    for (int i = 0; i < 4; i++) {
      int jointIndex = homeIndices[i];
      float target = values[i];
      if (target < JOINT_MIN[jointIndex] || target > JOINT_MAX[jointIndex]) {
        sendError("HP SETALL out of range");
        return;
      }
    }

    for (int i = 0; i < 4; i++) {
      homePoseConfig_.jointsDeg[homeIndices[i]] = values[i];
    }

    Serial.println("OK HP setall");
    return;
  }

  if (sub == "SAVE") {
    saveHomePoseConfig();
    Serial.println("OK HP saved");
    return;
  }

  if (sub == "LOAD") {
    if (loadHomePoseConfig()) {
      Serial.println("OK HP loaded");
    } else {
      resetHomePoseConfig();
      Serial.println("OK HP defaults");
    }
    return;
  }

  if (sub == "RESET") {
    resetHomePoseConfig();
    Serial.println("OK HP reset");
    return;
  }

  sendError("HP unknown");
}

void SerialProtocol::handleTrajectoryQueueCommand(String cmd) {
  auto mapTqErrorCode = [](const String& raw) -> String {
    if (raw == "running") return "RUNNING";
    if (raw == "queue full") return "QUEUE_FULL";
    if (raw == "time not monotonic") return "TIME_NON_MONOTONIC";
    if (raw == "need >=2 points") return "NEED_AT_LEAST_TWO_POINTS";
    if (raw == "motors disabled") return "MOTORS_DISABLED";
    if (raw == "already running") return "ALREADY_RUNNING";
    if (raw.startsWith("joint range")) return "JOINT_RANGE";
    if (raw.startsWith("velocity range")) return "VELOCITY_RANGE";
    if (raw == "non-finite") return "NON_FINITE";
    return "UNKNOWN";
  };

  cmd.trim();
  if (cmd.length() == 0) {
    sendTrajectoryQueueError("MISSING_SUBCOMMAND", "missing subcommand");
    return;
  }

  int space = cmd.indexOf(' ');
  String sub = (space == -1) ? cmd : cmd.substring(0, space);
  String args = (space == -1) ? "" : cmd.substring(space + 1);
  sub.toUpperCase();

  if (sub == "CLEAR") {
    trajectory_.clear();
    Serial.println("TQ READY 0");
    sendAck("TQ", "CLEAR");
    return;
  }

  if (sub == "STOP") {
    trajectory_.stop();
    Serial.println("OK TQ stopped");
    sendAck("TQ", "STOP");
    return;
  }

  if (sub == "RUN") {
    String errorMessage;
    if (!trajectory_.run(errorMessage)) {
      sendTrajectoryQueueError(mapTqErrorCode(errorMessage), errorMessage);
      return;
    }

    currentState_ = STATE_MOVING;
    Serial.print("TQ READY ");
    Serial.println(trajectory_.getPointCount());
    if (CAP_TRAJECTORY_STATUS_V2) {
      Serial.print("TQ STAT ");
      Serial.print(trajectory_.getPointCount());
      Serial.print(" 1 ");
      Serial.print(trajectory_.getCurrentPointIndex());
      Serial.print(" ");
      Serial.print(trajectory_.getElapsedMs());
      Serial.print(" ");
      Serial.println(ackSequence_);
    }
    Serial.println("OK TQ running");
    sendAck("TQ", "RUN");
    return;
  }

  if (sub == "PT") {
    float values[13];
    int index = 0;
    int start = 0;
    args.trim();

    while (start < args.length() && index < 13) {
      int end = args.indexOf(' ', start);
      if (end == -1) end = args.length();
      String token = args.substring(start, end);
      values[index++] = token.toFloat();
      start = end + 1;
      while (start < args.length() && args.charAt(start) == ' ') start++;
    }

    if (index != 13) {
      sendTrajectoryQueueError("PT_FORMAT", "PT format");
      return;
    }

    uint32_t tMs = (uint32_t)values[0];
    JointAngles q;
    JointAngles qd;
    for (int i = 0; i < 6; i++) {
      q[i] = values[1 + i];
      qd[i] = values[7 + i];
    }

    String errorMessage;
    if (!trajectory_.addPoint(tMs, q, qd, errorMessage)) {
      sendTrajectoryQueueError(mapTqErrorCode(errorMessage), errorMessage);
      return;
    }

    Serial.println("OK TQ pt");
    sendAck("TQ", "PT");
    return;
  }

  sendTrajectoryQueueError("UNKNOWN_SUBCOMMAND", "unknown");
}

void SerialProtocol::handleMotionKernelCommand(String cmd) {
  cmd.trim();
  if (cmd.length() == 0) {
    sendError("MQ missing subcommand");
    return;
  }

  String sub = cmd;
  sub.toUpperCase();
  if (sub == "RESET") {
    trajectory_.resetDiagnostics();
    Serial.println("OK MQ reset");
    return;
  }

  sendError("MQ unknown");
}

bool SerialProtocol::executeOperationalHomePose(String& errorMessage) {
  const int homeSequence[4] = {1, 2, 3, 4};  // J2, J3, J4, J5 (0-indexed)

  for (int i = 0; i < 4; i++) {
    int jointIndex = homeSequence[i];
    float target = homePoseConfig_.jointsDeg[jointIndex];
    if (target < JOINT_MIN[jointIndex] || target > JOINT_MAX[jointIndex]) {
      errorMessage = String("target range J") + String(jointIndex + 1);
      return false;
    }

    if (!moveSingleJointBlocking(jointIndex, target, homePoseConfig_.speedDegS, errorMessage)) {
      return false;
    }
  }

  return true;
}

bool SerialProtocol::moveSingleJointBlocking(
  int jointIndex,
  float targetDeg,
  float speedDegS,
  String& errorMessage
) {
  JointAngles current = stepper_.getCurrentAngles();
  float delta = targetDeg - current[jointIndex];
  const float epsilon = 0.001f;
  if (fabs(delta) <= epsilon) {
    return true;
  }

  if (homing_.isEndstopTriggered(jointIndex)) {
    bool movingTowardEndstop = HOME_TOWARD_MIN[jointIndex] ? (delta < 0.0f) : (delta > 0.0f);
    if (movingTowardEndstop) {
      errorMessage = String("toward endstop J") + String(jointIndex + 1);
      return false;
    }
  }

  JointAngles target = current;
  target[jointIndex] = targetDeg;

  stepper_.clearStop();
  stepper_.setTargetAngles(target, clampHomePoseSpeed(speedDegS));

  unsigned long startMs = millis();
  const unsigned long timeoutMs = 30000;

  while (stepper_.isMoving()) {
    stepper_.update();
    if (pollSerialForEmergencyStop()) {
      errorMessage = "aborted";
      return false;
    }

    if (stepper_.isStopRequested()) {
      errorMessage = "aborted";
      return false;
    }

    if (millis() - startMs > timeoutMs) {
      stepper_.emergencyStop();
      errorMessage = String("timeout J") + String(jointIndex + 1);
      return false;
    }

    delayMicroseconds(50);
  }

  return true;
}

bool SerialProtocol::pollSerialForEmergencyStop() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == 'S' || c == 's') {
      handleStopCommand();
      return true;
    }
  }

  return false;
}

bool SerialProtocol::isHomePoseJointAllowed(int jointIndex) const {
  return jointIndex >= 1 && jointIndex <= 4;  // J2..J5
}

float SerialProtocol::clampHomePoseSpeed(float speedDegS) const {
  float speed = speedDegS;
  if (speed < HOMEPOSE_MIN_SPEED_DEG_S) speed = HOMEPOSE_MIN_SPEED_DEG_S;
  if (speed > HOMEPOSE_MAX_SPEED_DEG_S) speed = HOMEPOSE_MAX_SPEED_DEG_S;
  return speed;
}

void SerialProtocol::resetHomePoseConfig() {
  homePoseConfig_.enabled = HOMEPOSE_DEFAULT_ENABLED;
  homePoseConfig_.speedDegS = HOMEPOSE_DEFAULT_SPEED_DEG_S;
  for (int i = 0; i < 6; i++) {
    homePoseConfig_.jointsDeg[i] = HOMEPOSE_DEFAULT_JOINTS_DEG[i];
  }
}

bool SerialProtocol::loadHomePoseConfig() {
  HomePosePersistentData stored;
  EEPROM.get(HOMEPOSE_EEPROM_ADDR, stored);
  if (stored.magic != HOMEPOSE_MAGIC || stored.version != HOMEPOSE_VERSION) {
    return false;
  }

  homePoseConfig_.enabled = stored.enabled != 0;
  homePoseConfig_.speedDegS = clampHomePoseSpeed(stored.speedDegS);
  for (int i = 0; i < 6; i++) {
    float target = stored.jointsDeg[i];
    if (target < JOINT_MIN[i]) target = JOINT_MIN[i];
    if (target > JOINT_MAX[i]) target = JOINT_MAX[i];
    homePoseConfig_.jointsDeg[i] = target;
  }

  return true;
}

bool SerialProtocol::saveHomePoseConfig() const {
  HomePosePersistentData stored;
  stored.magic = HOMEPOSE_MAGIC;
  stored.version = HOMEPOSE_VERSION;
  stored.enabled = homePoseConfig_.enabled ? 1 : 0;
  stored.speedDegS = homePoseConfig_.speedDegS;
  for (int i = 0; i < 6; i++) {
    stored.jointsDeg[i] = homePoseConfig_.jointsDeg[i];
  }

  EEPROM.put(HOMEPOSE_EEPROM_ADDR, stored);
  return true;
}

void SerialProtocol::sendHomePoseConfig() {
  Serial.print("HP {\"enabled\":");
  Serial.print(homePoseConfig_.enabled ? "true" : "false");
  Serial.print(",\"speedDegS\":");
  Serial.print(homePoseConfig_.speedDegS, 3);
  Serial.print(",\"applyAfterHAll\":");
  Serial.print(HOMEPOSE_APPLY_AFTER_HALL ? "true" : "false");
  Serial.print(",\"jointsDeg\":[");
  for (int i = 0; i < 6; i++) {
    Serial.print(homePoseConfig_.jointsDeg[i], 3);
    if (i < 5) Serial.print(",");
  }
  Serial.println("]}");
}

void SerialProtocol::sendTrajectoryQueueStatus() {
  Serial.print("TQ READY ");
  Serial.println(trajectory_.getPointCount());
  if (CAP_TRAJECTORY_STATUS_V2) {
    Serial.print("TQ STAT ");
    Serial.print(trajectory_.getPointCount());
    Serial.print(" ");
    Serial.print(trajectory_.isRunning() ? 1 : 0);
    Serial.print(" ");
    Serial.print(trajectory_.getCurrentPointIndex());
    Serial.print(" ");
    Serial.print(trajectory_.getElapsedMs());
    Serial.print(" ");
    Serial.println(ackSequence_);
  }
}

void SerialProtocol::sendMotionKernelStatus() {
  MotionKernelDiagnostics diag = trajectory_.getDiagnostics();
  Serial.print("MQ STAT ");
  Serial.print(diag.tickJitterUs);
  Serial.print(" ");
  Serial.print(diag.queueUnderrun);
  Serial.print(" ");
  Serial.println(diag.stepOverrun);
}

void SerialProtocol::sendConfig() {
  Serial.print("CFG {\"version\":1,\"protocolVersion\":");
  Serial.print(SERIAL_PROTOCOL_VERSION);
  Serial.print(",\"capabilities\":{\"trajectoryQueue\":");
  Serial.print(CAP_TRAJECTORY_QUEUE ? "true" : "false");
  Serial.print(",\"trajectoryMaxPoints\":");
  Serial.print(TRAJECTORY_MAX_POINTS);
  Serial.print(",\"trajectoryPointFormat\":\"");
  Serial.print(TRAJECTORY_POINT_FORMAT);
  Serial.print("\",\"motionKernelV2\":");
  Serial.print(CAP_MOTION_KERNEL_V2 ? "true" : "false");
  Serial.print(",\"motionKernelDiag\":");
  Serial.print(CAP_MOTION_KERNEL_DIAG ? "true" : "false");
  Serial.print(",\"commandAckV1\":");
  Serial.print(CAP_COMMAND_ACK_V1 ? "true" : "false");
  Serial.print(",\"trajectoryErrorCodesV1\":");
  Serial.print(CAP_TRAJECTORY_ERROR_CODES_V1 ? "true" : "false");
  Serial.print(",\"trajectoryStatusV2\":");
  Serial.print(CAP_TRAJECTORY_STATUS_V2 ? "true" : "false");
  Serial.print("},\"joints\":[");
  for (int i = 0; i < 6; i++) {
    JointCalibration cal = stepper_.getCalibration(i);
    Serial.print("{\"min\":");
    Serial.print(JOINT_MIN[i], 3);
    Serial.print(",\"max\":");
    Serial.print(JOINT_MAX[i], 3);
    Serial.print(",\"stepsPerDeg\":");
    Serial.print(USTEPS_PER_DEG[i], 3);
    Serial.print(",\"invertDir\":");
    Serial.print(INVERT_DIR[i] ? "true" : "false");
    Serial.print(",\"hasEndstop\":");
    Serial.print(HAS_ENDSTOP[i] ? "true" : "false");
    Serial.print(",\"homeTowardMin\":");
    Serial.print(HOME_TOWARD_MIN[i] ? "true" : "false");
    Serial.print(",\"homeLogicalDeg\":");
    Serial.print(HOME_LOGICAL_DEG[i], 3);
    Serial.print(",\"postHomeOffsetDeg\":");
    Serial.print(POST_HOME_OFFSET_DEG[i], 3);
    Serial.print(",\"urdfOffsetDeg\":");
    Serial.print(URDF_OFFSET_DEG[i], 3);
    Serial.print(",\"cal\":{\"scale\":");
    Serial.print(cal.scale, 6);
    Serial.print(",\"offset\":");
    Serial.print(cal.offset, 6);
    Serial.print("}}");
    if (i < 5) Serial.print(",");
  }
  Serial.print("],\"pulseWidthUs\":");
  Serial.print(PULSE_WIDTH_US);
  Serial.print(",\"dirSetupUs\":");
  Serial.print(DIR_SETUP_US);
  Serial.print(",\"endstopDebounceMs\":");
  Serial.print(ENDSTOP_DEBOUNCE_MS);
  Serial.print(",\"homePose\":{\"enabled\":");
  Serial.print(homePoseConfig_.enabled ? "true" : "false");
  Serial.print(",\"speedDegS\":");
  Serial.print(homePoseConfig_.speedDegS, 3);
  Serial.print(",\"applyAfterHAll\":");
  Serial.print(HOMEPOSE_APPLY_AFTER_HALL ? "true" : "false");
  Serial.print(",\"jointsDeg\":[");
  for (int i = 0; i < 6; i++) {
    Serial.print(homePoseConfig_.jointsDeg[i], 3);
    if (i < 5) Serial.print(",");
  }
  Serial.print("]}");
  Serial.println("}");
}

bool SerialProtocol::parseJointIndex(String token, int& jointIndex) {
  token.trim();
  token.toUpperCase();
  if (token.startsWith("J")) {
    token = token.substring(1);
  }
  if (token.length() == 0) return false;
  int idx = token.toInt() - 1;
  if (idx < 0 || idx >= 6) return false;
  jointIndex = idx;
  return true;
}

bool SerialProtocol::parseCalibrationJson(String json, float scaleOut[6], float offsetOut[6]) {
  int calPos = json.indexOf("\"cal\"");
  if (calPos == -1) return false;

  int pos = calPos;
  int found = 0;
  while (found < 6) {
    int scalePos = json.indexOf("\"scale\"", pos);
    if (scalePos == -1) break;
    float scaleValue = 1.0f;
    if (!parseFloatAfterKey(json, scalePos, scaleValue)) break;

    int offsetPos = json.indexOf("\"offset\"", scalePos);
    if (offsetPos == -1) break;
    float offsetValue = 0.0f;
    if (!parseFloatAfterKey(json, offsetPos, offsetValue)) break;

    scaleOut[found] = scaleValue;
    offsetOut[found] = offsetValue;
    found++;
    pos = offsetPos + 1;
  }

  return found == 6;
}

bool SerialProtocol::parseFloatAfterKey(String json, int keyPos, float& out) {
  int colon = json.indexOf(':', keyPos);
  if (colon == -1) return false;

  int start = colon + 1;
  while (start < json.length() && (json.charAt(start) == ' ' || json.charAt(start) == '\t')) {
    start++;
  }

  int end = start;
  while (end < json.length()) {
    char c = json.charAt(end);
    if (c == ',' || c == '}' || c == ']') break;
    end++;
  }

  String value = json.substring(start, end);
  out = value.toFloat();
  return true;
}
