#include "Arduino.h"

namespace mock {

Hardware hw;
std::vector<AxisPins> axisPins;
std::vector<uint8_t> endstopPins;

void registerAxis(int index, uint8_t stepPin, uint8_t dirPin, bool invertDir) {
  if ((int)axisPins.size() <= index) axisPins.resize(index + 1);
  AxisPins ap;
  ap.step = stepPin;
  ap.dir = dirPin;
  ap.invertDir = invertDir;
  axisPins[index] = ap;
}

void registerEndstop(int index, uint8_t pin) {
  if ((int)endstopPins.size() <= index) endstopPins.resize(index + 1, 255);
  endstopPins[index] = pin;
}

}  // namespace mock

MockSerial Serial;
