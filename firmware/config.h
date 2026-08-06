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
//
// Measured on the arm, not derived: each joint was jogged in the positive
// direction and watched against its endstop, which sits at the minimum end of
// travel on every switched axis. A positive command must move a joint away from
// its switch. J2, J4 and J5 moved toward it and are inverted here; J3 already
// moved correctly and keeps the value it had.
//
// This contradicts the old project notes on J3, which call for false. The notes
// were right about J2, J4, J5 and wrong about J3, so the measurement stands.
const bool INVERT_DIR[NUM_AXES] = {false, false, true, true, true, false};

// ---------------------------------------------------------------------------
// Travel limits (degrees)
// ---------------------------------------------------------------------------
//
// These are the mechanical hard stops. They are mirrored in the web app at
// robot-arm-control/src/kinematics/robotModel.ts, and a test there asserts the
// exact values. Change both together, or the solver will hand over angles the
// firmware silently clamps.

// Taken from the mechanical design in URDF.md, mapped into firmware angles
// through the same relation the 3D view uses:
//
//     urdf = URDF_DIRECTION * (logical - POST_HOME_ANGLES)
//
// This is where limits are supposed to come from. The previous values came from
// project notes, which this bring-up caught being wrong about direction on four
// of six axes and about homing direction on three of four - not a source to
// take a safety limit from. The URDF was written by hand (per-joint effort and
// velocity figures, none of them round) and never consulted.
//
// A software limit sits INSIDE the mechanical stop, with margin, so that
// reaching a software limit is ordinary and reaching a stop is a fault. Three
// degrees is taken off each upper bound for that. The lower bound is the
// endstop end, physically anchored at zero and verified on the arm; two degrees
// keeps ordinary motion off the switch, which the safety monitor watches.
//
// Verify from the inside, never by probing: jog to a limit and check clearance
// remains. A joint that reaches its limit with room to spare has a conservative
// limit. One that fouls first means the URDF is optimistic - tighten it.
//
// !! J1 is deliberately short of its design range. The URDF allows +/-160 deg,
// !! and nothing mechanical is known to stop it, but the URDF does not model
// !! cables, and 320 degrees of base rotation will wrap a loom that was never
// !! routed for it. +/-90 already gives two and a half times the old range.
// !! Open it further once the cable routing has been looked at.
//
// !! J6's URDF entry reads lower="0" upper="0", the placeholder a continuous
// !! joint gets, so it carries no information. Left at a full turn either way.
//
// Confirmed on the arm by approaching each limit from the inside, and three of
// them moved:
//
//   J3  158 -> 104   the design range is not reachable: the arm hits ITSELF at
//                    about 107 degrees, well before anything mechanical stops
//                    the joint. See the note below - this one is different in
//                    kind from the others.
//   J4  305 -> 332   more travel than the URDF claimed
//   J5  271 -> 222   less
//
// J1 and J2 have not been checked against the arm yet.
//
// !! J3's limit is a SELF-COLLISION limit, not a mechanical one, and a single
// !! number is a poor way to hold it. Where the arm fouls depends on where J2
// !! and the wrist are, so 104 is conservative in some poses and cannot be
// !! trusted to be conservative in all of them. It is a stand-in until the
// !! planner checks collisions properly; until then, treat J3 near its limit
// !! with more suspicion than the other axes.
const float JOINT_MIN[NUM_AXES] = {-90,   2,   2,   2,   2, -360};
const float JOINT_MAX[NUM_AXES] = { 90,  86, 104, 332, 222,  360};

// ---------------------------------------------------------------------------
// Calibration
// ---------------------------------------------------------------------------

// Microsteps per degree of joint travel (1/16 microstepping, including the
// gear reduction).
//
// Measured, not derived. The original values came from documented gear pairings
// and were wrong on three axes; these fold in the scale factors from an earlier
// calibration of this same arm (logical = raw * scale, so the true figure is
// the old one divided by scale).
//
// J5 is the striking one: its gear pairing was never recorded, so 2:1 was a
// guess, and it is really 3.39:1. J3 is a 16:90 pair rather than the assumed
// 16:100 - 5.625:1 against 6.25:1, two pairings that look nearly identical on
// the machine.
//
// Cross-checked independently: applying these scales to the angles this
// firmware reports at the parked pose reproduces the home pose recorded by the
// earlier project to within 0.0 deg on J4, 1.0 on J5 and 1.1 on J3.
const float USTEPS_PER_DEG[NUM_AXES] = {
  55.556,   // J1   6.250:1
  222.222,  // J2  25.000:1
  50.000,   // J3   5.625:1  (was 55.556, assumed 6.25:1)
  25.862,   // J4   2.909:1  (was 33.333, assumed 3.75:1)
  30.132,   // J5   3.390:1  (was 17.778, assumed 2:1 - never recorded)
  8.889     // J6   1.000:1
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

// !! BRING-UP VALUES. These are deliberately far below what the hardware can
// !! do -- roughly a quarter of the estimates they replace -- because the arm
// !! stalled and vibrated on the first run, worst on J3. A stalling motor makes
// !! every other measurement meaningless: it does not reach its endstop, it
// !! loses steps, and the position report becomes fiction.
// !!
// !! At these speeds every axis stays inside the pull-in torque of its motor
// !! even if the drivers turn out to be set to 1/8 microstepping, which would
// !! double the real speed. That makes the direction and calibration checks in
// !! docs/BRINGUP.md possible regardless of how the jumpers are set.
// !!
// !! Raise these afterwards, one axis at a time, listening. See BRINGUP step 6.
const float MAX_JOINT_SPEED[NUM_AXES] = {
  15.0f,   // J1  deg/s  ->  16 RPM at the motor
  10.0f,   // J2         ->  42 RPM
  15.0f,   // J3         ->  16 RPM
  20.0f,   // J4         ->  13 RPM
  30.0f,   // J5         ->  10 RPM
  45.0f    // J6         ->   8 RPM
};

const float MAX_JOINT_ACCEL[NUM_AXES] = {
  40.0f,   // J1  deg/s^2
  25.0f,   // J2  carries the whole arm, so the gentlest ramp
  40.0f,   // J3
  60.0f,   // J4
  75.0f,   // J5
  100.0f   // J6
};

// Ceiling for runtime tuning, per axis.
//
// MAX_JOINT_SPEED above is the working value: what the arm is currently trusted
// to do. These are the hard bound on what tuning may ask for, so a mistyped
// figure cannot send an axis somewhere nothing has ever tested.
//
// Set to the original design estimates. Those turned out to be too fast on a
// cold arm - they are what made J3 stall and vibrate - so reaching them is not
// expected. They are a bound, not a target.
//
// !! Past what an axis can hold, a stepper loses steps silently. Nothing detects
// !! it: the position report keeps counting and stops matching the arm. After a
// !! tuning run that produced grinding, re-home before trusting a position.
const float TUNING_MAX_SPEED[NUM_AXES] = {60.0f, 40.0f, 60.0f, 90.0f, 120.0f, 180.0f};
const float TUNING_MAX_ACCEL[NUM_AXES] = {150.0f, 100.0f, 150.0f, 250.0f, 300.0f, 400.0f};

// Default speed used when the host does not specify one.
const float DEFAULT_SPEED = 8.0f;  // deg/s

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
//
// Resolved on the arm. Every switched axis carries its endstop at the minimum
// end of travel: HOME_POSITION is 0 for all of them, JOINT_MIN is 0, and the
// resting poses in POST_HOME_ANGLES are all positive, so the joint parks by
// moving up and away from the switch. Seeking toward the minimum is therefore
// correct on all four, which is what the old project notes said and what the
// values below now reflect.
const bool HOME_TOWARD_MIN[NUM_AXES] = {false, true, true, true, true, false};

// Joint angle assigned once the endstop is found, and the resting pose the
// joint is moved to afterwards.
const float HOME_POSITION[NUM_AXES] = {0, 0, 0, 0, 0, 0};

// Where the arm parks after homing. Measured on the arm: it was jogged into the
// wanted pose and these are the angles reported there, rescaled by the
// calibration above. They agree with the operational home pose recorded by the
// earlier project on this machine to within 2 degrees on every axis.
const float POST_HOME_ANGLES[NUM_AXES] = {0, 15.0f, 41.1f, 165.0f, 131.0f, 0};

// Also reduced for bring-up: a seek in the wrong direction should crawl, not
// run, so there is time to cut the power before it reaches the hard stop.
const float HOMING_SPEED = 4.0f;        // deg/s, fast seek
const float HOMING_FINE_SPEED = 1.0f;   // deg/s, second approach

// Per-axis multiplier on both homing speeds. The axes do not want the same
// seek rate: J4 and J5 cover 274 and 280 degrees of travel against J2's 60, so
// one speed that suits J2 makes them crawl for minutes. Set from how each axis
// behaved on the arm.
//
// Still scaled by MAX_JOINT_SPEED, so a factor cannot drive an axis past what
// it can actually hold.
const float HOMING_SPEED_FACTOR[NUM_AXES] = {
  1.0f,  // J1  no endstop
  1.0f,  // J2
  2.0f,  // J3
  3.0f,  // J4
  3.0f,  // J5
  1.0f   // J6  no endstop
};
const float BACKOFF_DISTANCE = 2.0f;    // degrees to retract after triggering

// The retreat is repeated in BACKOFF_DISTANCE steps until the switch re-opens,
// up to this total. One fixed retraction is not enough in practice: J5 reported
// "Endstop still closed after back-off" because a joint that starts out pressed
// deep into its switch needs more than 2 degrees to clear it. Retreating moves
// away from the hard stop, so a generous bound costs nothing, while a switch
// that never opens still fails.
const float BACKOFF_MAX_DISTANCE = 15.0f;  // degrees

// Give up if the endstop has not triggered within this much travel, per joint.
//
// A joint cannot need more than its own range of travel to reach its switch,
// whatever arbitrary position it powers up in. Sized as
// (JOINT_MAX - JOINT_MIN) + 10 degrees of margin.
//
// This used to be a flat 360 degrees for every joint. J2 only travels 60
// degrees in total, so a seek in the wrong direction - which is easy to
// configure by accident, see HOME_TOWARD_MIN above - ground the joint against
// its hard stop for 300 degrees before the firmware gave up. Bounding the seek
// per joint turns that from destructive into a clean error.
const float HOMING_MAX_TRAVEL[NUM_AXES] = {
  190.0f,  // J1  range 180 deg (no endstop)
  95.0f,   // J2  range  84 deg
  115.0f,  // J3  range 102 deg
  340.0f,  // J4  range 330 deg
  230.0f,  // J5  range 220 deg
  730.0f   // J6  range 720 deg (no endstop)
};

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
