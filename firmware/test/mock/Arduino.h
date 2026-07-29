// Minimal Arduino mock, only for host-side unit tests of the firmware.
//
// This exists so that the real StepperController.cpp, HomingController.cpp and
// SerialProtocol.cpp can be compiled and driven on a development machine,
// instead of the tests re-implementing the logic they are meant to check. The
// step ISR under test is the same code that runs on the Teensy.
//
// It models the parts of the hardware that matter:
//   - pin levels, with STEP pulses counted into a virtual motor position so a
//     test can compare where the motor actually is against what the firmware
//     believes,
//   - a settable millisecond clock,
//   - a serial port backed by string buffers.

#ifndef ARDUINO_MOCK_H
#define ARDUINO_MOCK_H

#include <cstdint>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <string>
#include <vector>

#define HIGH 1
#define LOW 0
#define OUTPUT 1
#define INPUT 0
#define INPUT_PULLUP 2

#define DEC 10

namespace mock {

const int MAX_PINS = 64;

struct Hardware {
  int level[MAX_PINS];
  int mode[MAX_PINS];

  /** Virtual motor position per axis, in microsteps, advanced by STEP pulses. */
  long motorSteps[8];

  /** Driven by the test to model endstop switches. Active LOW, like the real ones. */
  bool endstopClosed[8];

  uint32_t millisValue;

  std::string txLog;
  std::string rxQueue;
  size_t rxCursor;

  Hardware() { reset(); }

  void reset() {
    for (int i = 0; i < MAX_PINS; i++) {
      level[i] = LOW;
      mode[i] = INPUT;
    }
    for (int i = 0; i < 8; i++) {
      motorSteps[i] = 0;
      endstopClosed[i] = false;
    }
    millisValue = 0;
    txLog.clear();
    rxQueue.clear();
    rxCursor = 0;
  }
};

extern Hardware hw;

/** Registered by the test so STEP pulses can be attributed to an axis. */
struct AxisPins {
  uint8_t step;
  uint8_t dir;
  bool invertDir;
};

extern std::vector<AxisPins> axisPins;
extern std::vector<uint8_t> endstopPins;

void registerAxis(int index, uint8_t stepPin, uint8_t dirPin, bool invertDir);
void registerEndstop(int index, uint8_t pin);

}  // namespace mock

// ---------------------------------------------------------------------------
// Arduino API
// ---------------------------------------------------------------------------

inline void pinMode(uint8_t pin, uint8_t mode) {
  if (pin < mock::MAX_PINS) mock::hw.mode[pin] = mode;
}

inline void digitalWrite(uint8_t pin, uint8_t value) {
  if (pin >= mock::MAX_PINS) return;

  const int previous = mock::hw.level[pin];
  mock::hw.level[pin] = value ? HIGH : LOW;

  // A rising edge on a STEP pin advances the virtual motor, in the direction the
  // DIR pin currently selects. Recovering the logical direction through
  // invertDir also exercises the firmware's direction handling.
  if (previous == LOW && mock::hw.level[pin] == HIGH) {
    for (size_t axis = 0; axis < mock::axisPins.size(); axis++) {
      const mock::AxisPins& ap = mock::axisPins[axis];
      if (ap.step != pin) continue;

      const bool pinHigh = (mock::hw.level[ap.dir] == HIGH);
      const bool forward = ap.invertDir ? !pinHigh : pinHigh;
      mock::hw.motorSteps[axis] += forward ? 1 : -1;
      break;
    }
  }
}

#define digitalWriteFast digitalWrite

inline int digitalRead(uint8_t pin) {
  for (size_t axis = 0; axis < mock::endstopPins.size(); axis++) {
    if (mock::endstopPins[axis] == pin) {
      return mock::hw.endstopClosed[axis] ? LOW : HIGH;  // active LOW
    }
  }
  return (pin < mock::MAX_PINS) ? mock::hw.level[pin] : HIGH;
}

inline uint32_t millis() { return mock::hw.millisValue; }
inline uint32_t micros() { return mock::hw.millisValue * 1000u; }

inline void noInterrupts() {}
inline void interrupts() {}

/** Stand-in for the Teensy timer. Tests call StepperController::onTick directly. */
class IntervalTimer {
public:
  bool begin(void (*)(), float) { return true; }
  void priority(uint8_t) {}
  void end() {}
};

class MockSerial {
public:
  void begin(unsigned long) {}

  int available() const {
    return (int)(mock::hw.rxQueue.size() - mock::hw.rxCursor);
  }

  int read() {
    if (mock::hw.rxCursor >= mock::hw.rxQueue.size()) return -1;
    return (unsigned char)mock::hw.rxQueue[mock::hw.rxCursor++];
  }

  int availableForWrite() const { return 1024; }

  void print(const char* s) { mock::hw.txLog += s; }
  void print(char c) { mock::hw.txLog += c; }
  void print(int v) { appendFormatted("%d", v); }
  void print(unsigned int v) { appendFormatted("%u", v); }
  void print(long v) { appendFormatted("%ld", v); }

  void print(float v, int digits = 2) {
    char tmp[32];
    snprintf(tmp, sizeof(tmp), "%.*f", digits, (double)v);
    mock::hw.txLog += tmp;
  }

  void println() { mock::hw.txLog += '\n'; }
  void println(const char* s) { mock::hw.txLog += s; mock::hw.txLog += '\n'; }
  void println(char c) { mock::hw.txLog += c; mock::hw.txLog += '\n'; }
  void println(int v) { print(v); println(); }
  void println(float v, int digits = 2) { print(v, digits); println(); }

private:
  template <typename T>
  void appendFormatted(const char* fmt, T v) {
    char tmp[32];
    snprintf(tmp, sizeof(tmp), fmt, v);
    mock::hw.txLog += tmp;
  }
};

extern MockSerial Serial;

#endif
