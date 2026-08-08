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

// ---------------------------------------------------------------------------
// Digital I/O
// ---------------------------------------------------------------------------
//
// Everything the arm has to do besides move: a gripper, a vacuum valve, a light,
// a part-present sensor, a signal to a conveyor. Without these the arm can go to
// a place and nothing else, which is the difference between a moving arm and a
// robot.
//
// !! THESE PIN NUMBERS HAVE NOT BEEN CHECKED AGAINST THE WIRING ON THIS ARM.
// !! They are the ones this firmware provably does not use for anything else -
// !! step, dir, enable and endstops account for 2-9, 22-27 and 30-33 - and they
// !! sit above the Arduino shield footprint, so a CNC shield cannot be holding
// !! them. That is an argument, not a measurement. Check with a meter before
// !! connecting anything, and change them here if they clash.
//
// Named, not numbered, because "gripper" is what an operator is thinking about
// and pin 34 is not. The protocol addresses them by index; the names travel to
// the app so the panel and the path steps can use them.

#define NUM_OUTPUTS 4
#define NUM_INPUTS 4

struct IOPoint {
  uint8_t pin;
  const char* name;
};

const IOPoint OUTPUT_POINTS[NUM_OUTPUTS] = {
  {34, "gripper"},
  {35, "out2"},
  {36, "out3"},
  {37, "out4"}
};

const IOPoint INPUT_POINTS[NUM_INPUTS] = {
  {38, "part"},
  {39, "in2"},
  {40, "in3"},
  {41, "in4"}
};

// What each output is driven to at start-up and when the drivers are switched
// off with E 0.
//
// E 0 means the cell is being put down, so an output left energised is one
// nobody is watching. An emergency stop deliberately does NOT touch these: a
// gripper that opens on a stop drops whatever it is holding, which is usually
// worse than the stop itself, and there is no default that is right for every
// tool. Choose per output and know which you chose.
const bool OUTPUT_SAFE_STATE[NUM_OUTPUTS] = {false, false, false, false};

// Inputs read through the same debounce as the endstops, for the same reason: a
// mechanical sensor bounces for milliseconds, and a path step waiting on one
// would otherwise fire on a contact whisker rather than on the part arriving.
const bool INPUT_ACTIVE_LOW[NUM_INPUTS] = {true, true, true, true};

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
// makes the noise. These values ramp J2 to its full 90 deg/s in 0.3 s
// (90 / 300) - nine times the speed the bring-up values reached, in three
// quarters of the time.
//
// Tune upward only while listening to the arm. If a joint knocks on starting or
// stopping, its acceleration is too high for the load.

// Tuned by ear on the arm over two rounds, one axis at a time, raising speed
// and acceleration together until a joint made a noise. Eight times the
// bring-up values they replace, which were set low after the arm stalled on its
// first run and were never meant to stand.
//
// !! Every axis reached the TUNING_MAX ceiling in both rounds without ever
// !! complaining, so these are NOT the limits of the arm -- they are the limits
// !! of what has been tried, twice. The usual 20% back-off from a found limit
// !! was never taken, because no limit was ever found.
// !!
// !! Held here on purpose rather than pushed further: this is fast enough for
// !! the work, and an axis that has never been heard to fail is one whose margin
// !! nobody knows. The ceilings below stay above these so a third round needs no
// !! re-flash.
// !!
// !! What is not covered: a warm motor, a sagging supply and a payload, none of
// !! which were present while tuning. J4 and J5 in particular were tuned cold
// !! and already run hot -- see docs/HARDWARE.md.
const float MAX_JOINT_SPEED[NUM_AXES] = {
  120.0f,  // J1  deg/s  -> 125 RPM at the motor,  6667 steps/s
  90.0f,   // J2         -> 375 RPM,              20000 steps/s  (the fastest)
  120.0f,  // J3         -> 113 RPM,               6000 steps/s
  180.0f,  // J4         ->  87 RPM,               4655 steps/s
  200.0f,  // J5         -> 113 RPM,               6026 steps/s
  360.0f   // J6         ->  60 RPM,               3200 steps/s
};

const float MAX_JOINT_ACCEL[NUM_AXES] = {
  400.0f,  // J1  deg/s^2
  300.0f,  // J2  carries the whole arm, so the gentlest ramp
  400.0f,  // J3
  600.0f,  // J4
  700.0f,  // J5
  1000.0f  // J6
};

// Ceiling for runtime tuning, per axis.
//
// MAX_JOINT_SPEED above is the working value: what the arm is currently trusted
// to do. These are the hard bound on what tuning may ask for, so a mistyped
// figure cannot send an axis somewhere nothing has ever tested.
//
// Held at 1.5x the working values. Two tuning rounds have now ended with every
// axis sitting on this ceiling rather than on a noise, so its only real job is
// to leave a third round somewhere to go without a re-flash. Kept deliberately
// narrow for that reason: a wide ceiling above values nobody has heard fail is
// an invitation, not a safeguard.
//
// Sanity-checked two ways, because a ceiling is only useful if asking for it
// cannot break something:
//
//   step rate   the fastest is J2 at 30000 steps/s, against the 50 kHz the ISR
//               can emit per axis (STEP_ISR_HZ / 2). Clear, with the margin
//               narrowing - a fourth round would want this checked again.
//   motor rate  the fastest is J2 at 563 RPM through its 25:1 reduction. That
//               is past where a NEMA17 holds much torque, so J2 is the axis
//               most likely to find its limit first, which is the point.
//
// !! Past what an axis can hold, a stepper loses steps silently. Nothing detects
// !! it: the position report keeps counting and stops matching the arm. After a
// !! tuning run that produced grinding, re-home before trusting a position.
const float TUNING_MAX_SPEED[NUM_AXES] = {180.0f, 135.0f, 180.0f, 270.0f, 300.0f, 540.0f};
const float TUNING_MAX_ACCEL[NUM_AXES] = {600.0f, 450.0f, 600.0f, 900.0f, 1000.0f, 1500.0f};

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
// rate is STEP_ISR_HZ / 2, per axis - each axis has its own step pin and the
// Bresenham distribution can fire all six on the same tick. The fastest axis is
// J2 at 20000 steps/s working, 30000 at its tuning ceiling, so there is
// headroom either way - but less than there was, and worth re-checking before
// J2 is raised again.
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
