#include "IOController.h"

IOController::IOController() {
  for (uint8_t i = 0; i < NUM_OUTPUTS; i++) {
    outputs_[i] = OUTPUT_SAFE_STATE[i];
  }
  for (uint8_t i = 0; i < NUM_INPUTS; i++) {
    inputs_[i] = false;
    rawLast_[i] = false;
    stable_[i] = 0;
  }
}

void IOController::begin() {
  for (uint8_t i = 0; i < NUM_OUTPUTS; i++) {
    pinMode(OUTPUT_POINTS[i].pin, OUTPUT);
  }

  // Drive the safe state before anything else can ask for something different.
  // A board that resets while a gripper is closed comes back with the gripper
  // closed unless this happens first, and nothing else in the system knows the
  // difference between "closed because a program wanted it" and "closed because
  // the pin happened to power up that way".
  allSafe();

  for (uint8_t i = 0; i < NUM_INPUTS; i++) {
    // Pull-ups, because a sensor that is unplugged should read the same as one
    // that is not triggered rather than floating between the two.
    pinMode(INPUT_POINTS[i].pin, INPUT_PULLUP);

    const bool raw = digitalRead(INPUT_POINTS[i].pin) == LOW;
    rawLast_[i] = raw;
    inputs_[i] = INPUT_ACTIVE_LOW[i] ? raw : !raw;
    stable_[i] = ENDSTOP_DEBOUNCE_COUNT;
  }
}

void IOController::update() {
  for (uint8_t i = 0; i < NUM_INPUTS; i++) {
    const bool raw = digitalRead(INPUT_POINTS[i].pin) == LOW;

    if (raw != rawLast_[i]) {
      // Changed: start counting again. A reading only becomes the state after it
      // has held still for the whole count.
      rawLast_[i] = raw;
      stable_[i] = 0;
      continue;
    }

    if (stable_[i] < ENDSTOP_DEBOUNCE_COUNT) {
      stable_[i]++;
      if (stable_[i] >= ENDSTOP_DEBOUNCE_COUNT) {
        inputs_[i] = INPUT_ACTIVE_LOW[i] ? raw : !raw;
      }
    }
  }
}

bool IOController::setOutput(uint8_t index, bool high) {
  if (index >= NUM_OUTPUTS) return false;

  outputs_[index] = high;
  digitalWrite(OUTPUT_POINTS[index].pin, high ? HIGH : LOW);
  return true;
}

bool IOController::outputState(uint8_t index) const {
  if (index >= NUM_OUTPUTS) return false;
  return outputs_[index];
}

bool IOController::inputState(uint8_t index) const {
  if (index >= NUM_INPUTS) return false;
  return inputs_[index];
}

void IOController::allSafe() {
  for (uint8_t i = 0; i < NUM_OUTPUTS; i++) {
    setOutput(i, OUTPUT_SAFE_STATE[i]);
  }
}
