#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// Pin definitions
struct PinConfig {
  uint8_t step;
  uint8_t dir;
};

const PinConfig JOINT_PINS[6] = {
  {2, 5},    // J1
  {3, 6},    // J2
  {4, 7},    // J3
  {22, 23},  // J4
  {24, 25},  // J5
  {26, 27}   // J6
};

const uint8_t ENABLE_PIN_1 = 8;  // Active LOW - CNC shield 1 (J1-J3)
const uint8_t ENABLE_PIN_2 = 9;  // Active LOW - CNC shield 2 (J4-J6)

const uint8_t ENDSTOP_PINS[6] = {
  255,  // J1 - no endstop
  30,   // J2
  31,   // J3
  32,   // J4
  33,   // J5
  255   // J6 - no endstop
};

// Joint limits (degrees)
const float JOINT_MIN[6] = {-40, 0, 0, 0, 0, -360};
const float JOINT_MAX[6] = {30, 60, 70, 274, 280, 360};

// Microsteps per degree (1/16 microstepping)
const float USTEPS_PER_DEG[6] = {
  55.556,   // J1
  222.222,  // J2
  55.556,   // J3
  33.333,   // J4
  17.778,   // J5
  8.889     // J6
};

// Direction inversion
const bool INVERT_DIR[6] = {false, true, true, false, false, false};

// Homing configuration
const bool HAS_ENDSTOP[6] = {false, true, true, true, true, false};
const bool HOME_TOWARD_MIN[6] = {false, false, true, false, false, false};
const float POST_HOME_ANGLES[6] = {0, 5, 55, 129, 220, 0};

// Motion parameters
const float DEFAULT_SPEED = 30.0;  // degrees/second
const float HOMING_SPEED = 10.0;   // degrees/second
const float BACKOFF_DISTANCE = 2.0; // degrees
const unsigned long PULSE_WIDTH_US = 5;  // Step pulse width

#endif
