#ifndef SERIAL_PROTOCOL_H
#define SERIAL_PROTOCOL_H

#include <Arduino.h>
#include "config.h"
#include "types.h"
#include "StepperController.h"
#include "HomingController.h"

/**
 * ASCII line protocol with the host.
 *
 * See docs/SERIAL_PROTOCOL.md for the wire format. Two things differ from the
 * original design and matter to callers:
 *
 *  - A move command reports how much queue space is left, so the host can keep
 *    the queue full without overrunning it. Streaming a trajectory depends on
 *    this: queue depth is what lets the planner carry speed from one point to
 *    the next instead of stopping at each one.
 *
 *  - Homing returns immediately and reports progress asynchronously, because it
 *    is now a state machine rather than a blocking routine.
 *
 * Input is parsed out of a fixed char buffer. The original used Arduino String
 * and appended one character at a time, which reallocates on the receive path
 * and fragments the heap over a long session.
 */
class SerialProtocol {
public:
  SerialProtocol(StepperController& stepper, HomingController& homing);

  void begin(unsigned long baudRate);

  /** Drain the receive buffer, emit periodic reports. Call every loop pass. */
  void update();

private:
  static const uint8_t BUFFER_SIZE = 128;

  StepperController& stepper_;
  HomingController& homing_;

  char buffer_[BUFFER_SIZE];
  uint8_t length_;
  bool discardLine_;

  uint32_t lastReportMs_;

  void readSerial();
  void reportHomingEvents();
  void sendPeriodicReports();

  void processLine(char* line);
  void handleMove(char* args);
  void handleHome(char* args);
  void handleEnable(char* args);
  void handleStop();
  void handleAbort();
  void handleQuery();

  void sendPosition();
  void sendEndstops();
  void sendStatus();
  void sendError(const char* message);

  RobotState currentState() const;

  /** True when there is room to send a line without blocking the main loop. */
  bool canWrite(int bytes) const;
};

#endif
