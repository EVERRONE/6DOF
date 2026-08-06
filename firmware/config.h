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
// J2/J4/J5 home toward max endstops, so zero is at the high mechanical side.
// Their valid travel away from endstop is negative.
const float JOINT_MIN[6] = {-60, -120, 0, -355, -355, -360};
const float JOINT_MAX[6] = {60, 0, 120, 0, 0, 360};

// Calibration defaults (logical degrees = raw degrees by default)
const float DEFAULT_CAL_SCALE[6] = {1, 1, 1, 1, 1, 1};
const float DEFAULT_CAL_OFFSET[6] = {0, 0, 0, 0, 0, 0};
const float MIN_CAL_SCALE = 0.0001;

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
// Logical angle assigned at the endstop (degrees), per joint
const float HOME_LOGICAL_DEG[6] = {
  0.0,  // J1 - unused (no endstop)
  0.0,  // J2
  0.0,  // J3
  0.0,  // J4
  0.0,  // J5
  0.0   // J6 - unused (no endstop)
};
// Post-home offsets (degrees away from endstop), per joint
const float POST_HOME_OFFSET_DEG[6] = {
  0.0,  // J1
  0.0,  // J2
  0.0,  // J3
  0.0,  // J4
  0.0,  // J5
  0.0   // J6
};

// URDF joint zero offsets (degrees) to align logical angles with URDF space
const float URDF_OFFSET_DEG[6] = {
  0.0,   // J1
  60.0,  // J2
  0.0,   // J3
  274.0, // J4
  280.0, // J5
  0.0    // J6
};

// Motion parameters
const float DEFAULT_SPEED = 30.0;  // degrees/second
const float HOMING_SPEED = 10.0;   // degrees/second
const float HOMING_SPEED_FACTOR[6] = {
  1.0,  // J1
  1.0,  // J2
  1.15, // J3
  2.0,  // J4
  2.0,  // J5
  1.0   // J6
};
const float BACKOFF_DISTANCE = 2.0; // degrees
const unsigned long PULSE_WIDTH_US = 5;  // Step pulse width
const unsigned long DIR_SETUP_US = 2;    // Dir setup/hold time
const unsigned long ENDSTOP_DEBOUNCE_MS = 5;
const unsigned long MIN_STEP_INTERVAL_US = PULSE_WIDTH_US + DIR_SETUP_US + 2;
const unsigned long STEPPER_TIMER_PERIOD_US = 10;              // deterministic stepping tick

// Streaming/trajectory execution parameters
const unsigned long TRAJECTORY_TICK_US = 1000;                 // 1 kHz update target
const unsigned long TRAJECTORY_PROGRESS_INTERVAL_MS = 100;     // Progress event cadence
const float STREAM_MIN_SPEED_DEG_S = 0.05f;                 // Allow queued motion to track very slow spline segments
const float STREAM_MAX_SPEED_DEG_S = 120.0f;
const float STREAM_SPEED_ACCEL_DEG_S2 = 300.0f;               // Speed slew clamp
const uint16_t TRAJECTORY_MAX_POINTS = 256;
const uint16_t TRAJECTORY_DECELERATION_TICKS = 400;           // Max ticks for end-of-trajectory decel phase
const uint8_t SERIAL_PROTOCOL_VERSION = 3;
const bool CAP_TRAJECTORY_QUEUE = true;
const bool CAP_MOTION_KERNEL_V2 = true;
const bool CAP_MOTION_KERNEL_DIAG = true;
const bool CAP_COMMAND_ACK_V1 = true;
const bool CAP_TRAJECTORY_ERROR_CODES_V1 = true;
const bool CAP_TRAJECTORY_STATUS_V2 = true;
const char TRAJECTORY_POINT_FORMAT[] = "hermite_v1";

struct JointCalibration {
  float scale;
  float offset;
};

struct CalibrationData {
  uint32_t magic;
  uint8_t version;
  JointCalibration cal[6];
};

const uint32_t CALIBRATION_MAGIC = 0x43414C31; // "CAL1"
const uint8_t CALIBRATION_VERSION = 1;
const int CALIBRATION_EEPROM_ADDR = 0;

struct HomePoseRuntimeConfig {
  bool enabled;
  float speedDegS;
  float jointsDeg[6];
};

struct HomePosePersistentData {
  uint32_t magic;
  uint8_t version;
  uint8_t enabled;
  float speedDegS;
  float jointsDeg[6];
};

const bool HOMEPOSE_APPLY_AFTER_HALL = true;
const bool HOMEPOSE_DEFAULT_ENABLED = false;
const float HOMEPOSE_DEFAULT_SPEED_DEG_S = 10.0f;
const float HOMEPOSE_MIN_SPEED_DEG_S = 5.0f;
const float HOMEPOSE_MAX_SPEED_DEG_S = 40.0f;
const float HOMEPOSE_DEFAULT_JOINTS_DEG[6] = {
  0.0f,  // J1
  0.0f,  // J2
  0.0f,  // J3
  0.0f,  // J4
  0.0f,  // J5
  0.0f   // J6
};

const uint32_t HOMEPOSE_MAGIC = 0x48504F31; // "HPO1"
const uint8_t HOMEPOSE_VERSION = 1;
const int HOMEPOSE_EEPROM_ADDR = CALIBRATION_EEPROM_ADDR + sizeof(CalibrationData);

#endif
