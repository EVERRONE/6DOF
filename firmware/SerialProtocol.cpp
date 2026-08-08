#include "SerialProtocol.h"

#include <ctype.h>
#include <stdlib.h>
#include <string.h>

namespace {

/** Skip leading spaces and tabs. */
char* skipSpace(char* p) {
  while (*p == ' ' || *p == '\t') p++;
  return p;
}

/**
 * Case-insensitive prefix match. Hand-rolled rather than using strncasecmp,
 * which is not available on every Arduino core.
 */
bool matchesPrefix(const char* text, const char* prefix) {
  for (; *prefix != '\0'; text++, prefix++) {
    if (toupper((unsigned char)*text) != toupper((unsigned char)*prefix)) return false;
  }
  return true;
}

/**
 * Read one float from `*cursor`, advancing past it.
 * Returns false when there is no number to read.
 */
bool nextFloat(char** cursor, float* out) {
  char* p = skipSpace(*cursor);
  if (*p == '\0') return false;

  char* end = p;
  const float value = strtof(p, &end);
  if (end == p) return false;

  *out = value;
  *cursor = end;
  return true;
}

}  // namespace

SerialProtocol::SerialProtocol(StepperController& stepper, HomingController& homing, IOController& io)
    : stepper_(stepper),
      homing_(homing),
      io_(io),
      length_(0),
      discardLine_(false),
      lastReportMs_(0) {
  buffer_[0] = '\0';
}

void SerialProtocol::begin(unsigned long baudRate) {
  Serial.begin(baudRate);
  // Teensy presents a USB CDC port, so the baud rate is nominal and there is no
  // reason to wait for a DTR that a headless host may never assert.
  Serial.println("OK Robot arm ready");
}

void SerialProtocol::update() {
  readSerial();
  reportHomingEvents();
  sendPeriodicReports();
}

bool SerialProtocol::canWrite(int bytes) const {
  return Serial.availableForWrite() >= bytes;
}

// ---------------------------------------------------------------------------
// Receive
// ---------------------------------------------------------------------------

void SerialProtocol::readSerial() {
  // Bounded per pass so a host that floods the link cannot starve endstop
  // polling or the homing state machine.
  int budget = 128;

  while (budget-- > 0 && Serial.available()) {
    const char c = (char)Serial.read();

    if (c == '\n' || c == '\r') {
      if (discardLine_) {
        // The line was too long to hold; report once and drop it.
        discardLine_ = false;
        length_ = 0;
        sendError("Command too long");
        continue;
      }
      if (length_ == 0) continue;

      buffer_[length_] = '\0';
      processLine(buffer_);
      length_ = 0;
      continue;
    }

    if (discardLine_) continue;

    if (length_ >= BUFFER_SIZE - 1) {
      discardLine_ = true;
      length_ = 0;
      continue;
    }

    buffer_[length_++] = c;
  }
}

void SerialProtocol::processLine(char* line) {
  char* p = skipSpace(line);
  if (*p == '\0') return;

  const char command = (char)toupper((unsigned char)*p);
  char* args = p + 1;

  switch (command) {
    case 'J': handleMove(args); break;
    case 'H': handleHome(args); break;
    case 'Q': handleQuery(); break;
    case 'E': handleEnable(args); break;
    case 'S': handleStop(); break;
    case 'A': handleAbort(); break;
    case 'V': handleLimits(args); break;
    case 'O': handleIO(args); break;
    default: sendError("Unknown command"); break;
  }
}

// ---------------------------------------------------------------------------
// Commands
// ---------------------------------------------------------------------------

void SerialProtocol::handleMove(char* args) {
  JointAngles target;
  char* cursor = args;

  for (int i = 0; i < NUM_AXES; i++) {
    float value = 0.0f;
    if (!nextFloat(&cursor, &value)) {
      sendError("Expected 6 joint angles");
      return;
    }
    target[i] = value;
  }

  // Speed is optional; without it the move runs at DEFAULT_SPEED.
  float speed = DEFAULT_SPEED;
  nextFloat(&cursor, &speed);

  if (stepper_.isEmergencyStopped()) {
    sendError("Emergency stop latched, send E 1 to clear");
    return;
  }
  if (!stepper_.isEnabled()) {
    sendError("Motors disabled");
    return;
  }
  if (homing_.isBusy()) {
    sendError("Homing in progress");
    return;
  }

  if (!stepper_.queueMove(target, speed)) {
    // Not an error: the host is streaming faster than the arm consumes. Tell it
    // how much room there is so it can resend when space appears.
    Serial.print("BUSY ");
    Serial.println((int)stepper_.queueFree());
    return;
  }

  Serial.print("OK J ");
  Serial.println((int)stepper_.queueFree());
}

void SerialProtocol::handleHome(char* args) {
  char* p = skipSpace(args);

  if (stepper_.isEmergencyStopped()) {
    sendError("Emergency stop latched, send E 1 to clear");
    return;
  }
  if (!stepper_.isEnabled()) {
    sendError("Motors disabled");
    return;
  }
  if (homing_.isBusy()) {
    sendError("Homing already in progress");
    return;
  }

  bool started = false;

  if (matchesPrefix(p, "ALL")) {
    started = homing_.startAll();
  } else {
    uint8_t axes[NUM_AXES];
    uint8_t count = 0;

    for (; *p != '\0' && count < NUM_AXES; p++) {
      if (*p == ' ' || *p == '\t') continue;
      if (*p < '1' || *p > '6') {
        sendError("Joint numbers must be 1-6");
        return;
      }
      axes[count++] = (uint8_t)(*p - '1');
    }

    if (count == 0) {
      sendError("Expected joint numbers or ALL");
      return;
    }
    started = homing_.start(axes, count);
  }

  if (!started) {
    sendError("Cannot home those joints");
    return;
  }

  Serial.println("OK Homing");
}

void SerialProtocol::handleEnable(char* args) {
  char* p = skipSpace(args);

  if (*p == '0') {
    stepper_.disable();
    // The cell is being put down, so nothing should be left energised that
    // nobody is watching. An emergency stop deliberately does not do this: a
    // gripper that opens mid-stop drops whatever it is holding.
    io_.allSafe();
    Serial.println("OK Motors disabled");
    return;
  }

  stepper_.enable();
  Serial.println("OK Motors enabled");
}

void SerialProtocol::handleStop() {
  homing_.abort();
  stepper_.emergencyStop();
  Serial.println("OK Emergency stop");
}

void SerialProtocol::handleAbort() {
  homing_.abort();
  stepper_.decelerateToStop();
  Serial.println("OK Aborted");
}

/**
 * V - motion limits, for tuning by ear.
 *
 *   V                 report every axis
 *   V <n> <spd> <acc> set one axis, 1-based
 *   V RESET           back to what config.h says
 *
 * Acceleration is the parameter that decides whether the arm is quiet, and it
 * can only be set by ear: run a move, listen, adjust, run again. Held in
 * config.h alone that costs a re-flash per attempt, which makes the job
 * impractical rather than merely slow.
 *
 * Nothing is persisted. A tuning value that survives a reboot is one somebody
 * forgets they left in, and then config.h no longer describes the machine.
 */
void SerialProtocol::handleLimits(char* args) {
  char* cursor = args;
  while (*cursor == ' ') cursor++;

  if (*cursor == '\0') {
    for (int i = 0; i < NUM_AXES; i++) {
      Serial.print("LIMIT ");
      Serial.print(i + 1);
      Serial.print(' ');
      Serial.print(stepper_.speedLimit(i), 2);
      Serial.print(' ');
      Serial.print(stepper_.accelLimit(i), 2);
      Serial.print(' ');
      Serial.print(TUNING_MAX_SPEED[i], 2);
      Serial.print(' ');
      Serial.println(TUNING_MAX_ACCEL[i], 2);
    }
    return;
  }

  if (cursor[0] == 'R' || cursor[0] == 'r') {
    stepper_.resetMotionLimits();
    Serial.println("OK Limits reset");
    return;
  }

  float axis = 0.0f, speed = 0.0f, accel = 0.0f;
  if (!nextFloat(&cursor, &axis) || !nextFloat(&cursor, &speed) ||
      !nextFloat(&cursor, &accel)) {
    sendError("V wants: axis speed accel, or RESET");
    return;
  }

  const int index = (int)axis - 1;
  if (index < 0 || index >= NUM_AXES) {
    sendError("V axis out of range");
    return;
  }

  // Changing a limit under a move in progress would alter the block the ISR is
  // executing halfway through it.
  if (!stepper_.isIdle()) {
    sendError("V refused: the arm is moving");
    return;
  }

  stepper_.setSpeedLimit(index, speed);
  stepper_.setAccelLimit(index, accel);

  // Report what was actually taken, which is not what was asked for when the
  // request went past the ceiling.
  Serial.print("OK V ");
  Serial.print(index + 1);
  Serial.print(' ');
  Serial.print(stepper_.speedLimit(index), 2);
  Serial.print(' ');
  Serial.println(stepper_.accelLimit(index), 2);
}

void SerialProtocol::handleQuery() {
  sendPosition();
  sendEndstops();
  sendStatus();
}

// ---------------------------------------------------------------------------
// Reports
// ---------------------------------------------------------------------------

void SerialProtocol::reportHomingEvents() {
  // Check for room *before* consuming: these events are one-shot, so consuming
  // one and then finding no space to send it would lose it silently.
  if (!canWrite(96)) return;

  const int axis = homing_.consumeHomedAxis();
  if (axis >= 0) {
    Serial.print("HOMED ");
    Serial.println(axis + 1);
  }

  if (homing_.consumeCompletion()) {
    Serial.println("OK Homed");
  }

  if (homing_.consumeFailure()) {
    sendError(homing_.lastError());
  }
}

void SerialProtocol::sendPeriodicReports() {
  const uint32_t now = millis();
  if ((now - lastReportMs_) < REPORT_INTERVAL_MS) return;

  // Three lines of roughly 60, 20 and 30 bytes. If the host is not draining the
  // port, skip this round rather than block: the main loop drives endstop
  // polling and the homing state machine.
  if (!canWrite(128)) return;

  lastReportMs_ = now;
  sendPosition();
  sendEndstops();
  sendIO();
  sendStatus();
}

void SerialProtocol::sendPosition() {
  const JointAngles angles = stepper_.currentAngles();

  Serial.print("POS");
  for (int i = 0; i < NUM_AXES; i++) {
    Serial.print(' ');
    Serial.print(angles[i], 2);
  }
  Serial.println();
}

void SerialProtocol::sendEndstops() {
  const EndstopState state = homing_.endstopState();

  Serial.print("ENDSTOP");
  for (int i = 0; i < NUM_AXES; i++) {
    Serial.print(' ');
    Serial.print(state.triggered[i] ? '1' : '0');
  }
  Serial.println();
}

void SerialProtocol::sendIO() {
  // IO <o1..oN> <i1..iN>, digits like ENDSTOP. Outputs first, inputs after.
  Serial.print("IO");
  for (int i = 0; i < NUM_OUTPUTS; i++) {
    Serial.print(' ');
    Serial.print(io_.outputState(i) ? '1' : '0');
  }
  for (int i = 0; i < NUM_INPUTS; i++) {
    Serial.print(' ');
    Serial.print(io_.inputState(i) ? '1' : '0');
  }
  Serial.println();
}

/**
 * O                 report the names, so the host does not hardcode them
 * O <n> <0|1>       drive one output, 1-based
 * O SAFE            every output to its configured safe state
 */
void SerialProtocol::handleIO(char* args) {
  char* p = skipSpace(args);

  if (*p == '\0') {
    for (int i = 0; i < NUM_OUTPUTS; i++) {
      Serial.print("IONAME OUT ");
      Serial.print(i + 1);
      Serial.print(' ');
      Serial.println(OUTPUT_POINTS[i].name);
    }
    for (int i = 0; i < NUM_INPUTS; i++) {
      Serial.print("IONAME IN ");
      Serial.print(i + 1);
      Serial.print(' ');
      Serial.println(INPUT_POINTS[i].name);
    }
    sendIO();
    return;
  }

  if (toupper((unsigned char)*p) == 'S') {
    io_.allSafe();
    Serial.println("OK O safe");
    sendIO();
    return;
  }

  const long index = strtol(p, &p, 10);
  if (index < 1 || index > NUM_OUTPUTS) {
    sendError("Output out of range");
    return;
  }

  p = skipSpace(p);
  if (*p != '0' && *p != '1') {
    sendError("Output value must be 0 or 1");
    return;
  }

  io_.setOutput((uint8_t)(index - 1), *p == '1');

  Serial.print("OK O ");
  Serial.print(index);
  Serial.print(' ');
  Serial.println(*p == '1' ? '1' : '0');
  sendIO();
}

void SerialProtocol::sendStatus() {
  // STATUS <state> <queueFree> <moving> <positionTrusted> <homedMask> <enabled>
  const char* name = "IDLE";
  switch (currentState()) {
    case STATE_MOVING:   name = "MOVING"; break;
    case STATE_HOMING:   name = "HOMING"; break;
    case STATE_ERROR:    name = "ERROR"; break;
    case STATE_ESTOPPED: name = "ESTOP"; break;
    case STATE_IDLE:
    default:             name = "IDLE"; break;
  }

  uint8_t homedMask = 0;
  for (int i = 0; i < NUM_AXES; i++) {
    if (stepper_.isHomed(i)) homedMask |= (uint8_t)(1u << i);
  }

  Serial.print("STATUS ");
  Serial.print(name);
  Serial.print(' ');
  Serial.print((int)stepper_.queueFree());
  Serial.print(' ');
  Serial.print(stepper_.isMoving() ? '1' : '0');
  Serial.print(' ');
  Serial.print(stepper_.positionTrusted() ? '1' : '0');
  Serial.print(' ');
  Serial.print((int)homedMask);
  Serial.print(' ');
  // Whether the drivers are energised. Reported so the host does not have to
  // assume its own E command succeeded.
  Serial.println(stepper_.isEnabled() ? '1' : '0');
}

void SerialProtocol::sendError(const char* message) {
  Serial.print("ERROR ");
  Serial.println(message);
}

RobotState SerialProtocol::currentState() const {
  if (stepper_.isEmergencyStopped()) return STATE_ESTOPPED;
  if (homing_.hasFailed()) return STATE_ERROR;
  if (homing_.isBusy()) return STATE_HOMING;
  if (!stepper_.isIdle()) return STATE_MOVING;
  return STATE_IDLE;
}
