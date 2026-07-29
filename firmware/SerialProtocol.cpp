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

SerialProtocol::SerialProtocol(StepperController& stepper, HomingController& homing)
    : stepper_(stepper),
      homing_(homing),
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

void SerialProtocol::sendStatus() {
  // STATUS <state> <queueFree> <moving> <positionTrusted> <homedMask>
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
  Serial.println((int)homedMask);
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
