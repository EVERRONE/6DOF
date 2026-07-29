#ifndef CONFIG_H
#define CONFIG_H

// Hardware constants and tuning parameters.
//
// This header deliberately depends only on <stdint.h> and not on <Arduino.h>,
// so that the motion planner can be compiled and unit-tested on a host
// machine. See firmware/test/.

#include <stdint.h>

// ---------------------------------------------------------------------------
// Axes
// ---------------------------------------------------------------------------

#define NUM_AXES 6

// ---------------------------------------------------------------------------
// Pin assignment (Teensy 4.1)
// ---------------------------------------------------------------------------

struct PinConfig {
  uint8_t step;
  uint8_t dir;
};

const PinConfig JOINT_PINS[NUM_AXES] = {
  {2, 5},    // J1
  {3, 6},    // J2
  {4, 7},    // J3
  {22, 23},  // J4
  {24, 25},  // J5
  {26, 27}   // J6
};

const uint8_t ENABLE_PIN_1 = 8;  // Active LOW - CNC shield 1 (J1-J3)
const uint8_t ENABLE_PIN_2 = 9;  // Active LOW - CNC shield 2 (J4-J6)

const uint8_t ENDSTOP_NONE = 255;

const uint8_t ENDSTOP_PINS[NUM_AXES] = {
  ENDSTOP_NONE,  // J1 - no endstop
  30,            // J2
  31,            // J3
  32,            // J4
  33,            // J5
  ENDSTOP_NONE   // J6 - no endstop
};

// Direction inversion, per axis.
const bool INVERT_DIR[NUM_AXES] = {false, true, true, false, false, false};

// ---------------------------------------------------------------------------
// Travel limits (degrees)
// ---------------------------------------------------------------------------
//
// These are the mechanical hard stops. They are mirrored in the web app at
// robot-arm-control/src/kinematics/robotModel.ts, and a test there asserts the
// exact values. Change both together, or the solver will hand over angles the
// firmware silently clamps.

const float JOINT_MIN[NUM_AXES] = {-40, 0, 0, 0, 0, -360};
const float JOINT_MAX[NUM_AXES] = {30, 60, 70, 274, 280, 360};

// ---------------------------------------------------------------------------
// Calibration
// ---------------------------------------------------------------------------

// Microsteps per degree of joint travel (1/16 microstepping, including the
// gear reduction). Implied reductions: J1 6.3:1, J2 25:1, J3 6.3:1,
// J4 3.7:1, J5 2:1, J6 1:1.
const float USTEPS_PER_DEG[NUM_AXES] = {
  55.556,   // J1
  222.222,  // J2
  55.556,   // J3
  33.333,   // J4
  17.778,   // J5
  8.889     // J6
};

// ---------------------------------------------------------------------------
// Motion limits
// ---------------------------------------------------------------------------
//
// MAX_JOINT_SPEED and MAX_JOINT_ACCEL are the physical capability of each
// axis. Every queued move is scaled so that no axis exceeds either, whatever
// speed the host asks for.
//
// Acceleration is the parameter that stops the arm from grinding. Without a
// ramp, J2 (25:1) was asked to jump from standstill to 125-250 RPM at the
// motor, far above the pull-in torque of a loaded NEMA17, which loses steps and
// makes the noise. These values ramp J2 to 40 deg/s in 0.4 s.
//
// Tune upward only while listening to the arm. If a joint knocks on starting or
// stopping, its acceleration is too high for the load.

const float MAX_JOINT_SPEED[NUM_AXES] = {
  60.0f,   // J1  deg/s  ->  63 RPM at the motor
  40.0f,   // J2         -> 167 RPM
  60.0f,   // J3         ->  63 RPM
  90.0f,   // J4         ->  56 RPM
  120.0f,  // J5         ->  40 RPM
  180.0f   // J6         ->  30 RPM
};

const float MAX_JOINT_ACCEL[NUM_AXES] = {
  150.0f,  // J1  deg/s^2
  100.0f,  // J2  carries the whole arm, so the gentlest ramp
  150.0f,  // J3
  250.0f,  // J4
  300.0f,  // J5
  400.0f   // J6
};

// Default speed used when the host does not specify one.
const float DEFAULT_SPEED = 30.0f;  // deg/s

// ---------------------------------------------------------------------------
// Step generation
// ---------------------------------------------------------------------------

// Step pulses are generated from a fixed-frequency timer interrupt. Step times
// are quantised to this tick, so the tick rate sets the timing jitter: at
// 100 kHz a step lands within 10 us of its ideal time. The previous
// implementation generated steps from the main loop with blocking delays, which
// gave +/-27-53% of a step period of jitter and an audible growl.
//
// A step occupies two ticks (one high, one low), so the ceiling on the step
// rate is STEP_ISR_HZ / 2. The fastest axis needs 8889 steps/s, so there is
// plenty of headroom.
const uint32_t STEP_ISR_HZ = 100000;

// Slowest rate the profile is allowed to command, in step events per second.
// Without a floor, a move that starts and ends at rest would sit at zero
// velocity forever.
const float MIN_EVENT_RATE = 20.0f;

// ---------------------------------------------------------------------------
// Motion queue
// ---------------------------------------------------------------------------

// Number of queued moves. Depth is what lets the planner carry speed through a
// streamed trajectory instead of stopping at every point, so keep it generous.
#define MOTION_QUEUE_LENGTH 24

// Wait for this many queued moves before starting to execute, so that a
// streamed trajectory has look-ahead from the very first move.
const uint8_t START_QUEUE_DEPTH = 4;

// ...but never wait longer than this, so a single jog command still runs
// immediately.
const uint32_t START_DELAY_MS = 40;

// ---------------------------------------------------------------------------
// Homing
// ---------------------------------------------------------------------------

const bool HAS_ENDSTOP[NUM_AXES] = {false, true, true, true, true, false};

// Direction of travel to find the endstop.
const bool HOME_TOWARD_MIN[NUM_AXES] = {false, false, true, false, false, false};

// Joint angle assigned once the endstop is found, and the resting pose the
// joint is moved to afterwards.
const float HOME_POSITION[NUM_AXES] = {0, 0, 0, 0, 0, 0};
const float POST_HOME_ANGLES[NUM_AXES] = {0, 5, 55, 129, 220, 0};

const float HOMING_SPEED = 10.0f;       // deg/s, fast seek
const float HOMING_FINE_SPEED = 2.0f;   // deg/s, second approach
const float BACKOFF_DISTANCE = 2.0f;    // degrees to retract after triggering

// Give up if the endstop has not triggered within this much travel.
const float HOMING_MAX_TRAVEL = 360.0f; // degrees

// Endstops are read in the main loop; a switch must read triggered this many
// consecutive polls to count. Cheap debounce against contact bounce and noise.
const uint8_t ENDSTOP_DEBOUNCE_COUNT = 12;

// ---------------------------------------------------------------------------
// Host reporting
// ---------------------------------------------------------------------------

// How often position, endstop and status lines are pushed to the host. The
// position report is now live during a move, so this also sets how smoothly the
// 3D view tracks the arm.
const uint32_t REPORT_INTERVAL_MS = 50;

#endif
