// Host-side tests for the firmware motion path.
//
// These drive the real StepperController, HomingController and MotionPlanner
// against a mock Arduino layer, so the code under test is the code that runs on
// the Teensy. The mock counts STEP pulses into a virtual motor position, which
// lets a test compare where the motor actually ends up against what the firmware
// believes - the distinction the old homing routine got wrong.

#include <Arduino.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <string>
#include <vector>

#include "../HomingController.h"
#include "../MotionPlanner.h"
#include "../SafetyMonitor.h"
#include "../SerialProtocol.h"
#include "../StepperController.h"
#include "../config.h"
#include "../types.h"

// ---------------------------------------------------------------------------
// Tiny test framework
// ---------------------------------------------------------------------------

static int g_checks = 0;
static int g_failures = 0;
static const char* g_section = "";

static void section(const char* name) {
  g_section = name;
  printf("\n%s\n", name);
}

#define CHECK(cond)                                                     \
  do {                                                                  \
    g_checks++;                                                         \
    if (!(cond)) {                                                      \
      g_failures++;                                                     \
      printf("  FAIL  line %d: %s\n", __LINE__, #cond);                 \
    }                                                                   \
  } while (0)

#define CHECK_NEAR(actual, expected, tol)                                     \
  do {                                                                        \
    g_checks++;                                                               \
    const double a_ = (double)(actual);                                       \
    const double e_ = (double)(expected);                                     \
    if (!(std::fabs(a_ - e_) <= (double)(tol))) {                             \
      g_failures++;                                                           \
      printf("  FAIL  line %d: %s = %.6f, expected %.6f +/- %.6f\n",          \
             __LINE__, #actual, a_, e_, (double)(tol));                       \
    }                                                                         \
  } while (0)

static void pass(const char* what) { printf("  ok    %s\n", what); }

// ---------------------------------------------------------------------------
// Simulator
// ---------------------------------------------------------------------------

/** Ticks per simulated millisecond. */
static const uint32_t TICKS_PER_MS = STEP_ISR_HZ / 1000;

struct Sample {
  double t;
  long steps[NUM_AXES];
};

class Sim {
public:
  Sim(StepperController& stepper, HomingController* homing = 0)
      : stepper_(stepper), homing_(homing), ticks_(0), sampleEveryMs_(0) {}

  void registerHardware() {
    for (int i = 0; i < NUM_AXES; i++) {
      mock::registerAxis(i, JOINT_PINS[i].step, JOINT_PINS[i].dir, INVERT_DIR[i]);
      mock::registerEndstop(i, ENDSTOP_PINS[i]);
    }
  }

  void sampleEvery(uint32_t ms) {
    sampleEveryMs_ = ms;
    samples.clear();
  }

  /** Advance one timer tick, plus per-millisecond housekeeping. */
  void tick() {
    stepper_.onTick();
    ticks_++;

    if (ticks_ % TICKS_PER_MS != 0) return;

    mock::hw.millisValue++;
    if (endstopModel_) endstopModel_();
    if (homing_) homing_->pollEndstops();
    stepper_.service();
    if (homing_) homing_->update();

    if (sampleEveryMs_ && (mock::hw.millisValue % sampleEveryMs_ == 0)) {
      Sample s;
      s.t = mock::hw.millisValue / 1000.0;
      for (int i = 0; i < NUM_AXES; i++) s.steps[i] = mock::hw.motorSteps[i];
      samples.push_back(s);
    }
  }

  void run(double seconds) {
    const uint64_t total = (uint64_t)(seconds * STEP_ISR_HZ);
    for (uint64_t i = 0; i < total; i++) tick();
  }

  /** Run until the arm is idle, or the budget runs out. Returns true if idle. */
  bool runUntilIdle(double maxSeconds) {
    const uint64_t total = (uint64_t)(maxSeconds * STEP_ISR_HZ);
    for (uint64_t i = 0; i < total; i++) {
      tick();
      if (i % TICKS_PER_MS == 0 && stepper_.isIdle() &&
          (homing_ == 0 || !homing_->isBusy())) {
        return true;
      }
    }
    return false;
  }

  void setEndstopModel(void (*fn)()) { endstopModel_ = fn; }

  std::vector<Sample> samples;

private:
  StepperController& stepper_;
  HomingController* homing_;
  uint64_t ticks_;
  uint32_t sampleEveryMs_;
  void (*endstopModel_)() = 0;
};

/** Peak joint speed seen in a trace, deg/s. */
static double peakSpeed(const std::vector<Sample>& s, int axis) {
  double peak = 0.0;
  for (size_t i = 1; i < s.size(); i++) {
    const double dt = s[i].t - s[i - 1].t;
    if (dt <= 0) continue;
    const double dDeg =
        (s[i].steps[axis] - s[i - 1].steps[axis]) / (double)USTEPS_PER_DEG[axis];
    peak = std::max(peak, std::fabs(dDeg / dt));
  }
  return peak;
}

/** Peak joint acceleration seen in a trace, deg/s^2. */
static double peakAccel(const std::vector<Sample>& s, int axis) {
  std::vector<double> v;
  for (size_t i = 1; i < s.size(); i++) {
    const double dt = s[i].t - s[i - 1].t;
    const double dDeg =
        (s[i].steps[axis] - s[i - 1].steps[axis]) / (double)USTEPS_PER_DEG[axis];
    v.push_back(dt > 0 ? dDeg / dt : 0.0);
  }

  double peak = 0.0;
  for (size_t i = 1; i < v.size(); i++) {
    const double dt = s[i + 1].t - s[i].t;
    if (dt <= 0) continue;
    peak = std::max(peak, std::fabs((v[i] - v[i - 1]) / dt));
  }
  return peak;
}

/** Per-window speeds of one axis, deg/s. */
static std::vector<double> windowSpeeds(const std::vector<Sample>& s, int axis) {
  std::vector<double> v;
  for (size_t i = 1; i < s.size(); i++) {
    const double dt = s[i].t - s[i - 1].t;
    const double dDeg =
        (s[i].steps[axis] - s[i - 1].steps[axis]) / (double)USTEPS_PER_DEG[axis];
    v.push_back(dt > 0 ? std::fabs(dDeg / dt) : 0.0);
  }
  return v;
}

static JointAngles makeAngles(float j1, float j2, float j3, float j4, float j5,
                              float j6) {
  JointAngles a;
  a[0] = j1; a[1] = j2; a[2] = j3; a[3] = j4; a[4] = j5; a[5] = j6;
  return a;
}

static void resetWorld() {
  mock::hw.reset();
}

// ---------------------------------------------------------------------------
// Planner unit tests
// ---------------------------------------------------------------------------

static void testPlannerLimits() {
  section("planner: per-axis speed and acceleration limits");

  MotionPlanner planner;
  int32_t origin[NUM_AXES] = {0, 0, 0, 0, 0, 0};
  planner.reset(origin);

  // A move that asks for far more speed than J2 can deliver.
  int32_t target[NUM_AXES] = {0, 0, 0, 0, 0, 0};
  target[1] = (int32_t)lroundf(50.0f * USTEPS_PER_DEG[1]);

  CHECK(planner.enqueue(target, 10000.0f));

  MotionBlock* b = planner.currentBlock();
  CHECK(b != 0);
  CHECK(b->eventCount == (uint32_t)std::abs(target[1]));

  // Map the event rate back to J2's own speed and compare with its limit.
  const double j2Speed = b->cruiseRate / USTEPS_PER_DEG[1];
  const double j2Accel = b->accel / USTEPS_PER_DEG[1];

  CHECK_NEAR(j2Speed, MAX_JOINT_SPEED[1], 0.01);
  CHECK_NEAR(j2Accel, MAX_JOINT_ACCEL[1], 0.01);
  pass("an over-ambitious speed request is clamped to the axis limit");

  // Every axis of a multi-axis move must stay inside its own limits.
  MotionPlanner multi;
  multi.reset(origin);
  int32_t combo[NUM_AXES];
  for (int i = 0; i < NUM_AXES; i++) {
    combo[i] = (int32_t)lroundf(20.0f * USTEPS_PER_DEG[i]);
  }
  CHECK(multi.enqueue(combo, 10000.0f));

  MotionBlock* mb = multi.currentBlock();
  CHECK(mb != 0);
  for (int i = 0; i < NUM_AXES; i++) {
    if (mb->absDelta[i] == 0) continue;
    const double share = (double)mb->absDelta[i] / (double)mb->eventCount;
    const double speed = mb->cruiseRate * share / USTEPS_PER_DEG[i];
    const double accel = mb->accel * share / USTEPS_PER_DEG[i];
    CHECK(speed <= MAX_JOINT_SPEED[i] + 0.01);
    CHECK(accel <= MAX_JOINT_ACCEL[i] + 0.01);
  }
  pass("no axis of a six-axis move exceeds its own speed or acceleration limit");
}

static void testPlannerRequestedSpeed() {
  section("planner: the requested speed applies to the largest displacement");

  MotionPlanner planner;
  int32_t origin[NUM_AXES] = {0, 0, 0, 0, 0, 0};
  planner.reset(origin);

  // J4 travels furthest in degrees, so 20 deg/s should apply to J4.
  int32_t target[NUM_AXES] = {0, 0, 0, 0, 0, 0};
  target[3] = (int32_t)lroundf(60.0f * USTEPS_PER_DEG[3]);
  target[4] = (int32_t)lroundf(10.0f * USTEPS_PER_DEG[4]);

  CHECK(planner.enqueue(target, 20.0f));
  MotionBlock* b = planner.currentBlock();
  CHECK(b != 0);

  const double share = (double)b->absDelta[3] / (double)b->eventCount;
  const double j4Speed = b->cruiseRate * share / USTEPS_PER_DEG[3];
  CHECK_NEAR(j4Speed, 20.0, 0.05);
  pass("20 deg/s is delivered to the joint with the largest travel");
}

static void testPlannerCollinearAndReversal() {
  section("planner: junction velocities");

  int32_t origin[NUM_AXES] = {0, 0, 0, 0, 0, 0};

  // Three moves in a straight line: the junctions should carry full speed.
  {
    MotionPlanner planner;
    planner.reset(origin);

    for (int k = 1; k <= 3; k++) {
      int32_t t[NUM_AXES] = {0, 0, 0, 0, 0, 0};
      t[1] = (int32_t)lroundf(10.0f * k * USTEPS_PER_DEG[1]);
      CHECK(planner.enqueue(t, 40.0f));
    }

    MotionBlock* first = planner.currentBlock();
    CHECK(first != 0);
    CHECK(first->entryRate == 0.0f);          // starts from rest
    CHECK(first->exitRate > 0.5f * first->cruiseRate);
    pass("a collinear junction keeps most of the cruise rate");
  }

  // A reversal has to come to a stop.
  {
    MotionPlanner planner;
    planner.reset(origin);

    int32_t out[NUM_AXES] = {0, 0, 0, 0, 0, 0};
    out[1] = (int32_t)lroundf(30.0f * USTEPS_PER_DEG[1]);
    CHECK(planner.enqueue(out, 40.0f));

    int32_t back[NUM_AXES] = {0, 0, 0, 0, 0, 0};
    back[1] = 0;
    CHECK(planner.enqueue(back, 40.0f));

    MotionBlock* first = planner.currentBlock();
    CHECK(first != 0);
    CHECK_NEAR(first->exitRate, MIN_EVENT_RATE, MIN_EVENT_RATE);
    pass("a reversal decelerates to a stop at the junction");
  }

  // A shallow turn should land in between: slower through the corner, but not a
  // stop. 45 degrees in joint space gives cos = 0.707.
  {
    MotionPlanner planner;
    planner.reset(origin);

    int32_t a[NUM_AXES] = {0, 0, 0, 0, 0, 0};
    a[1] = (int32_t)lroundf(20.0f * USTEPS_PER_DEG[1]);
    CHECK(planner.enqueue(a, 40.0f));

    // Continue on J2 while adding an equal amount of J3.
    int32_t b[NUM_AXES] = {0, 0, 0, 0, 0, 0};
    b[1] = (int32_t)lroundf(40.0f * USTEPS_PER_DEG[1]);
    b[2] = (int32_t)lroundf(20.0f * USTEPS_PER_DEG[2]);
    CHECK(planner.enqueue(b, 40.0f));

    MotionBlock* first = planner.currentBlock();
    CHECK(first != 0);
    CHECK(first->exitRate > MIN_EVENT_RATE * 10.0f);
    CHECK(first->exitRate < first->cruiseRate);
    printf("        45 deg corner: exit %.0f of cruise %.0f events/s\n",
           (double)first->exitRate, (double)first->cruiseRate);
    pass("a shallow corner slows down without stopping");
  }

  // A right-angle turn stops, and that is deliberate. At a sharp corner one axis
  // has to go from full speed to rest while another goes from rest to full speed
  // over zero distance, which no finite acceleration can do. Carrying speed
  // through it would mean allowing a step change in velocity - exactly the jolt
  // this planner exists to remove. A finely sampled trajectory never hits this
  // case, because consecutive points are nearly collinear.
  {
    MotionPlanner planner;
    planner.reset(origin);

    int32_t a[NUM_AXES] = {0, 0, 0, 0, 0, 0};
    a[1] = (int32_t)lroundf(20.0f * USTEPS_PER_DEG[1]);
    CHECK(planner.enqueue(a, 40.0f));

    int32_t b[NUM_AXES] = {0, 0, 0, 0, 0, 0};
    b[1] = a[1];  // J2 holds
    b[2] = (int32_t)lroundf(20.0f * USTEPS_PER_DEG[2]);
    CHECK(planner.enqueue(b, 40.0f));

    MotionBlock* first = planner.currentBlock();
    CHECK(first != 0);
    CHECK_NEAR(first->exitRate, MIN_EVENT_RATE, MIN_EVENT_RATE);
    pass("a 90 degree corner comes to a stop, by design");
  }
}

static void testPlannerQueueAccounting() {
  section("planner: queue accounting");

  MotionPlanner planner;
  int32_t origin[NUM_AXES] = {0, 0, 0, 0, 0, 0};
  planner.reset(origin);

  CHECK(planner.empty());
  CHECK(planner.freeSlots() == MOTION_QUEUE_LENGTH - 1);

  int accepted = 0;
  for (int k = 1; k <= MOTION_QUEUE_LENGTH + 5; k++) {
    int32_t t[NUM_AXES] = {0, 0, 0, 0, 0, 0};
    t[5] = (int32_t)lroundf(2.0f * k * USTEPS_PER_DEG[5]);
    if (planner.enqueue(t, 60.0f)) accepted++;
  }

  CHECK(accepted == MOTION_QUEUE_LENGTH - 1);
  CHECK(planner.freeSlots() == 0);
  pass("the queue reports full instead of overwriting, so the host can wait");

  // A zero-length move is accepted but not queued.
  MotionPlanner idle;
  idle.reset(origin);
  CHECK(idle.enqueue(origin, 30.0f));
  CHECK(idle.empty());
  pass("a move to the current position is accepted and discarded");
}

static void testProfileShape() {
  section("planner: velocity profile");

  MotionPlanner planner;
  int32_t origin[NUM_AXES] = {0, 0, 0, 0, 0, 0};
  planner.reset(origin);

  int32_t target[NUM_AXES] = {0, 0, 0, 0, 0, 0};
  target[1] = (int32_t)lroundf(30.0f * USTEPS_PER_DEG[1]);
  CHECK(planner.enqueue(target, 40.0f));

  MotionBlock* b = planner.currentBlock();
  CHECK(b != 0);

  // Starts and ends at the floor rate, never exceeds cruise, and is symmetric
  // for a move that begins and ends at rest.
  CHECK_NEAR(MotionPlanner::rateAt(*b, 0), MIN_EVENT_RATE, 1.0);
  CHECK_NEAR(MotionPlanner::rateAt(*b, b->eventCount), MIN_EVENT_RATE, 1.0);

  double peak = 0.0;
  for (uint32_t s = 0; s <= b->eventCount; s += 17) {
    const float v = MotionPlanner::rateAt(*b, s);
    CHECK(v <= b->cruiseRate + 0.01f);
    peak = std::max(peak, (double)v);
  }
  CHECK_NEAR(peak, b->cruiseRate, b->cruiseRate * 0.01);
  pass("trapezoid rises to cruise and returns to rest, never overshooting");

  const float mid = MotionPlanner::rateAt(*b, b->eventCount / 2);
  const float quarterUp = MotionPlanner::rateAt(*b, b->eventCount / 4);
  const float quarterDown = MotionPlanner::rateAt(*b, 3 * b->eventCount / 4);
  CHECK_NEAR(quarterUp, quarterDown, mid * 0.02f);
  pass("acceleration and deceleration are symmetric");

  // A move too short to reach cruise degrades to a triangle without any special
  // case in the code.
  MotionPlanner shortMove;
  shortMove.reset(origin);
  int32_t tiny[NUM_AXES] = {0, 0, 0, 0, 0, 0};
  tiny[1] = (int32_t)lroundf(0.5f * USTEPS_PER_DEG[1]);
  CHECK(shortMove.enqueue(tiny, 40.0f));

  MotionBlock* sb = shortMove.currentBlock();
  CHECK(sb != 0);
  const float shortPeak = MotionPlanner::rateAt(*sb, sb->eventCount / 2);
  CHECK(shortPeak < sb->cruiseRate);
  pass("a short move becomes a triangular profile");
}

// ---------------------------------------------------------------------------
// Stepper integration tests
// ---------------------------------------------------------------------------

static void testExactPositioning() {
  section("stepper: step generation lands exactly on target");

  resetWorld();
  StepperController stepper;
  Sim sim(stepper);
  sim.registerHardware();
  stepper.begin();
  stepper.enable();

  // Derived from the limits rather than hardcoded. This test is about arriving
  // exactly on the commanded target, so the target has to be inside the joint
  // range - a hardcoded angle silently turns into a test of the clamp instead
  // the moment a limit changes, which is what happened when J5's range was
  // corrected from 280 to 165 degrees.
  JointAngles target;
  const float fractions[NUM_AXES] = {0.25f, 0.55f, 0.59f, 0.36f, 0.63f, 0.375f};
  for (int i = 0; i < NUM_AXES; i++) {
    target[i] = JOINT_MIN[i] + (JOINT_MAX[i] - JOINT_MIN[i]) * fractions[i];
  }
  CHECK(stepper.queueMove(target, 30.0f));
  CHECK(sim.runUntilIdle(30.0));

  const JointAngles reached = stepper.currentAngles();
  for (int i = 0; i < NUM_AXES; i++) {
    const long expected = lroundf(target[i] * USTEPS_PER_DEG[i]);
    // The firmware's own idea of where it is...
    CHECK_NEAR(reached[i], target[i], 1.0f / USTEPS_PER_DEG[i]);
    // ...and the pulses actually delivered to the driver agree with it.
    CHECK(mock::hw.motorSteps[i] == expected);
  }
  pass("every axis arrives at its target to within one microstep");
  pass("delivered pulses match the recorded position exactly");
}

static void testNoInstantStart() {
  section("stepper: acceleration ramp (the grinding fix)");

  resetWorld();
  StepperController stepper;
  Sim sim(stepper);
  sim.registerHardware();
  stepper.begin();
  stepper.enable();

  sim.sampleEvery(20);
  CHECK(stepper.queueMove(makeAngles(0, 50, 0, 0, 0, 0), MAX_JOINT_SPEED[1]));
  CHECK(sim.runUntilIdle(30.0));

  const double peak = peakSpeed(sim.samples, 1);
  const double accel = peakAccel(sim.samples, 1);

  CHECK(peak <= MAX_JOINT_SPEED[1] * 1.05);
  printf("        J2 peak speed  %.1f deg/s (limit %.1f)\n", peak, MAX_JOINT_SPEED[1]);
  printf("        J2 peak accel  %.1f deg/s^2 (limit %.1f)\n", accel, MAX_JOINT_ACCEL[1]);

  // Differentiating an integer step trace twice quantises the result: one
  // microstep of jitter inside a sampling window appears as
  // (1 / USTEPS_PER_DEG) / dt^2 of acceleration. On J2 at 20 ms windows that
  // grain is 11.25 deg/s^2, so a purely multiplicative tolerance passes at a
  // high configured limit and fails at a low one for no physical reason -
  // measured peaks come out as exact multiples of the grain. Allow for the
  // measurement explicitly and keep the multiplicative headroom tight, so a
  // planner that really overshot would still be caught.
  const double sampleDt = 0.020;
  const double accelGrain = (1.0 / USTEPS_PER_DEG[1]) / (sampleDt * sampleDt);
  CHECK(accel <= MAX_JOINT_ACCEL[1] * 1.1 + 3.0 * accelGrain);
  pass("acceleration stays within the configured limit");

  // The first sampling window must be far below cruise. Without a ramp it would
  // be at full speed from the first step, which is what made the arm grind.
  double firstWindow = 0.0;
  if (sim.samples.size() >= 2) {
    const double dt = sim.samples[1].t - sim.samples[0].t;
    const double dDeg =
        (sim.samples[1].steps[1] - sim.samples[0].steps[1]) / (double)USTEPS_PER_DEG[1];
    firstWindow = std::fabs(dDeg / dt);
  }
  printf("        J2 speed in first 20 ms  %.2f deg/s\n", firstWindow);
  CHECK(firstWindow < MAX_JOINT_SPEED[1] * 0.25);
  pass("the move starts from rest instead of jumping to full speed");

  // ...and ends at rest rather than stopping dead.
  double lastWindow = 0.0;
  const size_t n = sim.samples.size();
  if (n >= 3) {
    const double dt = sim.samples[n - 1].t - sim.samples[n - 2].t;
    const double dDeg = (sim.samples[n - 1].steps[1] - sim.samples[n - 2].steps[1]) /
                        (double)USTEPS_PER_DEG[1];
    lastWindow = std::fabs(dDeg / dt);
  }
  CHECK(lastWindow < MAX_JOINT_SPEED[1] * 0.35);
  pass("the move decelerates into its target instead of stopping dead");
}

static void testStreamingStaysContinuous() {
  section("stepper: streamed trajectory does not stop at every point");

  resetWorld();
  StepperController stepper;
  Sim sim(stepper);
  sim.registerHardware();
  stepper.begin();
  stepper.enable();

  // 20 collinear points, 2 degrees of J2 apart - the shape a planned trajectory
  // arrives in. The old firmware held a single target and restarted a
  // constant-rate move on each one, giving a full stop per point.
  for (int k = 1; k <= 20; k++) {
    JointAngles p = makeAngles(0, 2.0f * k, 0, 0, 0, 0);
    CHECK(stepper.queueMove(p, 40.0f));
  }

  const uint32_t windowMs = 20;
  sim.sampleEvery(windowMs);
  CHECK(sim.runUntilIdle(30.0));

  const std::vector<double> speeds = windowSpeeds(sim.samples, 1);
  const double peak = peakSpeed(sim.samples, 1);
  CHECK(peak > 0.0);

  // Skip the opening ramp-up and the closing ramp-down. Reaching cruise takes
  // v/a seconds, so derive the window count from the configured limits rather
  // than guessing it.
  const double rampSeconds = MAX_JOINT_SPEED[1] / MAX_JOINT_ACCEL[1];
  const size_t rampWindows = (size_t)std::ceil(rampSeconds * 1000.0 / windowMs) + 2;

  double lowest = 1e18;
  size_t stalls = 0;
  for (size_t i = rampWindows; i + rampWindows < speeds.size(); i++) {
    lowest = std::min(lowest, speeds[i]);
    if (speeds[i] < peak * 0.1) stalls++;
  }
  if (lowest == 1e18) lowest = 0.0;

  printf("        J2 peak %.1f deg/s, slowest while cruising %.1f deg/s\n", peak, lowest);
  printf("        near-stops between the 20 queued points: %zu\n", stalls);

  // Nineteen junctions. Restarting the move at each of them - what the old
  // firmware did - would show up as nineteen dips to zero.
  CHECK(stalls == 0);
  CHECK(lowest > peak * 0.8);
  pass("speed is carried through all 19 junctions without a single stop");

  const JointAngles reached = stepper.currentAngles();
  CHECK_NEAR(reached[1], 40.0f, 1.0f / USTEPS_PER_DEG[1]);
  pass("the streamed sequence still lands exactly on the final point");
}

static void testDecelerateToStop() {
  section("stepper: graceful stop");

  resetWorld();
  StepperController stepper;
  Sim sim(stepper);
  sim.registerHardware();
  stepper.begin();
  stepper.enable();
  stepper.setPositionTrusted(true);

  CHECK(stepper.queueMove(makeAngles(0, 60, 0, 0, 0, 0), MAX_JOINT_SPEED[1]));
  sim.run(0.8);  // let it reach cruise

  const long before = mock::hw.motorSteps[1];
  CHECK(stepper.isMoving());

  stepper.decelerateToStop();
  CHECK(sim.runUntilIdle(5.0));

  const long travelled = mock::hw.motorSteps[1] - before;
  const double travelledDeg = travelled / (double)USTEPS_PER_DEG[1];

  printf("        overtravel after stop request: %.2f deg\n", travelledDeg);
  CHECK(travelled > 0);
  // v^2 / 2a at the J2 limits is about 8 degrees; near zero would mean it
  // stopped dead, much larger would mean it ignored the request.
  CHECK(travelledDeg < 12.0);
  CHECK(stepper.isIdle());
  CHECK(!stepper.isEmergencyStopped());
  pass("a graceful stop decelerates within its acceleration limit");

  // Unlike a hard stop, this one cannot lose steps, so the datum survives.
  CHECK(stepper.positionTrusted());
  pass("a graceful stop keeps the position trusted");

  // The queue is dropped, and the recorded position still matches the pulses.
  CHECK(stepper.queueDepth() == 0);
  const JointAngles a = stepper.currentAngles();
  CHECK_NEAR(a[1], mock::hw.motorSteps[1] / (double)USTEPS_PER_DEG[1], 0.001);
  pass("position stays consistent after an abort");
}

static void testEmergencyStop() {
  section("stepper: emergency stop");

  resetWorld();
  StepperController stepper;
  Sim sim(stepper);
  sim.registerHardware();
  stepper.begin();
  stepper.enable();
  stepper.setPositionTrusted(true);

  CHECK(stepper.queueMove(makeAngles(0, 60, 0, 0, 0, 0), MAX_JOINT_SPEED[1]));
  sim.run(0.5);

  const long before = mock::hw.motorSteps[1];
  stepper.emergencyStop();
  sim.run(0.2);

  CHECK(mock::hw.motorSteps[1] == before);
  pass("pulses stop immediately");

  CHECK(stepper.isEmergencyStopped());
  CHECK(!stepper.positionTrusted());
  pass("position is marked untrusted, because a hard stop can lose steps");

  CHECK(!stepper.queueMove(makeAngles(0, 10, 0, 0, 0, 0), 20.0f));
  pass("further moves are refused while the stop is latched");

  stepper.enable();
  CHECK(!stepper.isEmergencyStopped());
  CHECK(stepper.queueMove(makeAngles(0, 10, 0, 0, 0, 0), 20.0f));
  pass("enabling clears the latch");
}

static void testJointLimitClamping() {
  section("stepper: joint limits");

  resetWorld();
  StepperController stepper;
  Sim sim(stepper);
  sim.registerHardware();
  stepper.begin();
  stepper.enable();

  // Well beyond both ends of the travel.
  CHECK(stepper.queueMove(makeAngles(-999, 999, -999, 999, -999, 0), 60.0f));
  CHECK(sim.runUntilIdle(60.0));

  const JointAngles a = stepper.currentAngles();
  for (int i = 0; i < NUM_AXES; i++) {
    CHECK(a[i] >= JOINT_MIN[i] - 0.01f);
    CHECK(a[i] <= JOINT_MAX[i] + 0.01f);
  }
  CHECK_NEAR(a[0], JOINT_MIN[0], 0.05f);
  CHECK_NEAR(a[1], JOINT_MAX[1], 0.05f);
  pass("out-of-range targets are clamped to the mechanical limits");
}

// ---------------------------------------------------------------------------
// Homing
// ---------------------------------------------------------------------------

// Switch trip points, in virtual motor microsteps, relative to power-on.
static long g_switchAt[NUM_AXES];
static bool g_switchActive[NUM_AXES];

static void homingEndstopModel() {
  for (int i = 0; i < NUM_AXES; i++) {
    if (!g_switchActive[i]) {
      mock::hw.endstopClosed[i] = false;
      continue;
    }
    // The switch closes once the axis reaches the trip point travelling in its
    // homing direction, and stays closed beyond it.
    if (HOME_TOWARD_MIN[i]) {
      mock::hw.endstopClosed[i] = (mock::hw.motorSteps[i] <= g_switchAt[i]);
    } else {
      mock::hw.endstopClosed[i] = (mock::hw.motorSteps[i] >= g_switchAt[i]);
    }
  }
}

static void testHomingBookkeeping() {
  section("homing: per-axis datum (the hard-stop crash fix)");

  resetWorld();

  for (int i = 0; i < NUM_AXES; i++) {
    g_switchActive[i] = HAS_ENDSTOP[i];
    // Place each switch a few degrees away in the seek direction so the seek is
    // short. The sign follows the homing direction.
    const double away = 4.0 * USTEPS_PER_DEG[i];
    g_switchAt[i] = (long)(HOME_TOWARD_MIN[i] ? -away : away);
  }

  StepperController stepper;
  HomingController homing(stepper);
  Sim sim(stepper, &homing);
  sim.registerHardware();
  sim.setEndstopModel(homingEndstopModel);

  stepper.begin();
  homing.begin();
  stepper.enable();

  CHECK(homing.startAll());
  const bool finished = sim.runUntilIdle(180.0);
  CHECK(finished);
  CHECK(!homing.hasFailed());
  if (homing.hasFailed()) printf("        error: %s\n", homing.lastError());

  // The whole point: every homed axis ends up recorded at its own park angle.
  // The old routine assigned a fresh all-zero JointAngles at each datum, so
  // after homing J2, J3, J4, J5 in sequence only J5 was right and J2/J3/J4 read
  // 0 while parked at 5, 55 and 129 degrees. The next ordinary move then drove
  // J3 to 110 degrees against a 70 degree limit.
  const JointAngles recorded = stepper.currentAngles();
  for (int i = 0; i < NUM_AXES; i++) {
    if (!HAS_ENDSTOP[i]) continue;

    CHECK_NEAR(recorded[i], POST_HOME_ANGLES[i], 0.02f);
    printf("        J%d recorded %.2f deg, expected %.2f deg\n", i + 1,
           (double)recorded[i], (double)POST_HOME_ANGLES[i]);

    // And the motor really is that far from its switch, so the datum is
    // physically meaningful and not just a bookkeeping value.
    const double parkDeg = POST_HOME_ANGLES[i] - HOME_POSITION[i];
    const double actualFromSwitch =
        (mock::hw.motorSteps[i] - g_switchAt[i]) / (double)USTEPS_PER_DEG[i];
    CHECK_NEAR(actualFromSwitch, parkDeg, 1.0);

    CHECK(stepper.isHomed(i));
  }
  pass("each axis keeps its own datum through the whole sequence");
  pass("recorded position matches the motor's real distance from the switch");

  CHECK(stepper.positionTrusted());
  pass("position is trusted once every switched axis has been homed");

  // The move that used to crash: command the documented home pose and confirm
  // nothing moves far and nothing exceeds a limit.
  JointAngles pose;
  for (int i = 0; i < NUM_AXES; i++) pose[i] = POST_HOME_ANGLES[i];

  long beforeSteps[NUM_AXES];
  for (int i = 0; i < NUM_AXES; i++) beforeSteps[i] = mock::hw.motorSteps[i];

  CHECK(stepper.queueMove(pose, 20.0f));
  CHECK(sim.runUntilIdle(30.0));

  for (int i = 0; i < NUM_AXES; i++) {
    const double movedDeg =
        (mock::hw.motorSteps[i] - beforeSteps[i]) / (double)USTEPS_PER_DEG[i];
    CHECK_NEAR(movedDeg, 0.0, 0.05);

    const JointAngles now = stepper.currentAngles();
    CHECK(now[i] <= JOINT_MAX[i] + 0.01f);
    CHECK(now[i] >= JOINT_MIN[i] - 0.01f);
  }
  pass("commanding the post-home pose right after homing moves nothing");
}

static void testHomingMissingEndstop() {
  section("homing: failure handling");

  resetWorld();
  for (int i = 0; i < NUM_AXES; i++) g_switchActive[i] = false;  // no switch ever closes

  StepperController stepper;
  HomingController homing(stepper);
  Sim sim(stepper, &homing);
  sim.registerHardware();
  sim.setEndstopModel(homingEndstopModel);

  stepper.begin();
  homing.begin();
  stepper.enable();

  const uint8_t axes[1] = {1};
  CHECK(homing.start(axes, 1));

  // The seek runs out of travel and reports rather than hanging.
  const bool settled = sim.runUntilIdle(120.0);
  CHECK(settled);
  CHECK(homing.hasFailed());
  printf("        reported: %s\n", homing.lastError());
  pass("a missing endstop fails cleanly instead of hanging");

  // The seek must be bounded by the joint's own range, not a flat 360 degrees:
  // a wrong-direction seek used to grind J2 against its hard stop for 300 of
  // them before giving up.
  const double sought = std::fabs((double)mock::hw.motorSteps[1]) / USTEPS_PER_DEG[1];
  printf("        travel before giving up: %.0f deg (joint range %.0f deg)\n",
         sought, (double)(JOINT_MAX[1] - JOINT_MIN[1]));
  CHECK(sought <= HOMING_MAX_TRAVEL[1] + 1.0);
  pass("the seek is bounded by the joint's own range of travel");

  CHECK(!homing.isBusy());
  CHECK(homing.consumeFailure());
  CHECK(!homing.consumeFailure());
  pass("the failure is reported exactly once");
}

// An axis that powers up already pressed deep into its switch. J5 failed here
// on the arm with "Endstop still closed after back-off": a single fixed
// retraction of BACKOFF_DISTANCE did not travel far enough for the switch to
// re-open, so homing gave up on a switch that was working perfectly.
static void testHomingClearsADeeplyPressedSwitch() {
  section("homing: a switch that needs more than one back-off to clear");

  resetWorld();
  for (int i = 0; i < NUM_AXES; i++) g_switchActive[i] = HAS_ENDSTOP[i];

  const int axis = 4;  // J5

  // Put the trip point well behind where the axis starts, so the switch reads
  // closed from the outset and stays closed for several BACKOFF_DISTANCE steps.
  const float pressedBy = BACKOFF_DISTANCE * 3.5f;
  for (int i = 0; i < NUM_AXES; i++) g_switchAt[i] = 0;
  g_switchAt[axis] = (long)(pressedBy * USTEPS_PER_DEG[axis]);

  StepperController stepper;
  HomingController homing(stepper);
  Sim sim(stepper, &homing);
  sim.registerHardware();
  sim.setEndstopModel(homingEndstopModel);

  stepper.begin();
  homing.begin();
  stepper.enable();

  // The sim drives the switch model each tick; poll it here too, since this
  // runs before the sim starts.
  for (int i = 0; i < 40; i++) {
    homingEndstopModel();
    homing.pollEndstops();
    mock::hw.millisValue++;
  }
  CHECK(homing.isEndstopTriggered(axis));
  pass("the axis starts with its switch already closed");

  const uint8_t axes[1] = {(uint8_t)axis};
  CHECK(homing.start(axes, 1));
  CHECK(sim.runUntilIdle(240.0));

  if (homing.hasFailed()) printf("        reported: %s\n", homing.lastError());
  CHECK(!homing.hasFailed());
  pass("retreats far enough to clear it instead of failing");

  CHECK(!homing.isEndstopTriggered(axis) ||
        stepper.currentAngles()[axis] != 0.0f);
  CHECK_NEAR(stepper.currentAngles()[axis], POST_HOME_ANGLES[axis], 0.5f);
  pass("and still parks on the resting pose with a correct datum");
}

// A switch that never opens, however far the joint retreats, is a real fault
// and must still be reported rather than retreated from forever.
static void testHomingStillFailsOnAStuckSwitch() {
  section("homing: a switch that never opens");

  resetWorld();
  for (int i = 0; i < NUM_AXES; i++) g_switchActive[i] = false;

  StepperController stepper;
  HomingController homing(stepper);
  Sim sim(stepper, &homing);
  sim.registerHardware();
  // Model a switch welded shut: closed no matter where the axis is.
  sim.setEndstopModel([]() {
    for (int i = 0; i < NUM_AXES; i++) mock::hw.endstopClosed[i] = HAS_ENDSTOP[i];
  });

  stepper.begin();
  homing.begin();
  stepper.enable();

  const uint8_t axes[1] = {4};
  CHECK(homing.start(axes, 1));
  CHECK(sim.runUntilIdle(240.0));

  CHECK(homing.hasFailed());
  printf("        reported: %s\n", homing.lastError());
  pass("a stuck switch is still reported rather than retreated from forever");

  const double retreated =
      std::fabs((double)mock::hw.motorSteps[4]) / USTEPS_PER_DEG[4];
  printf("        retreated %.1f deg before giving up (bound %.1f)\n",
         retreated, (double)BACKOFF_MAX_DISTANCE);
  CHECK(retreated <= BACKOFF_MAX_DISTANCE + BACKOFF_DISTANCE + 1.0);
  pass("and the retreat is bounded");
}

static void testHomingRefusesJointsWithoutEndstops() {
  section("homing: joints without endstops");

  resetWorld();
  StepperController stepper;
  HomingController homing(stepper);
  Sim sim(stepper, &homing);
  sim.registerHardware();
  stepper.begin();
  homing.begin();
  stepper.enable();

  const uint8_t j1[1] = {0};
  CHECK(!homing.start(j1, 1));
  const uint8_t j6[1] = {5};
  CHECK(!homing.start(j6, 1));
  pass("J1 and J6 are refused, since they have no switch");
}


// ---------------------------------------------------------------------------
// Safety monitor
// ---------------------------------------------------------------------------

static void testSafetyMonitor() {
  section("safety: endstop closing during ordinary motion");

  resetWorld();
  for (int i = 0; i < NUM_AXES; i++) g_switchActive[i] = false;

  StepperController stepper;
  HomingController homing(stepper);
  SafetyMonitor safety(stepper, homing);
  Sim sim(stepper, &homing);
  sim.registerHardware();
  stepper.begin();
  homing.begin();
  stepper.enable();
  stepper.setPositionTrusted(true);

  // Idle and clear: nothing to report.
  CHECK(safety.update() == -1);
  CHECK(!safety.isTripped());
  pass("silent while the arm is idle");

  CHECK(stepper.queueMove(makeAngles(0, 50, 0, 0, 0, 0), MAX_JOINT_SPEED[1]));
  sim.run(0.4);
  CHECK(stepper.isMoving());
  CHECK(safety.update() == -1);
  pass("silent while moving with every switch open");

  // Close J3's switch mid-move. Debounce needs consecutive polls, so let the
  // simulated loop run rather than flipping the flag and checking immediately.
  mock::hw.endstopClosed[2] = true;
  for (int i = 0; i < 40; i++) {
    homing.pollEndstops();
    mock::hw.millisValue++;
  }

  const long stoppedAt = mock::hw.motorSteps[1];
  const int tripped = safety.update();

  CHECK(tripped == 2);
  printf("        reported J%d\n", tripped + 1);
  pass("reports the axis whose switch closed");

  CHECK(stepper.isEmergencyStopped());
  sim.run(0.1);
  CHECK(mock::hw.motorSteps[1] == stoppedAt);
  pass("stops immediately rather than decelerating into the hard stop");

  CHECK(!stepper.positionTrusted());
  pass("marks the position untrusted, so the arm gets re-homed");

  // A stuck switch must not flood the link.
  CHECK(safety.update() == -1);
  CHECK(safety.update() == -1);
  pass("reports once per movement, not once per loop pass");
}

static void testSafetyIgnoresHoming() {
  section("safety: homing drives onto the switches on purpose");

  resetWorld();
  for (int i = 0; i < NUM_AXES; i++) {
    g_switchActive[i] = HAS_ENDSTOP[i];
    const double away = 4.0 * USTEPS_PER_DEG[i];
    g_switchAt[i] = (long)(HOME_TOWARD_MIN[i] ? -away : away);
  }

  StepperController stepper;
  HomingController homing(stepper);
  SafetyMonitor safety(stepper, homing);
  Sim sim(stepper, &homing);
  sim.registerHardware();
  sim.setEndstopModel(homingEndstopModel);
  stepper.begin();
  homing.begin();
  stepper.enable();

  const uint8_t axes[1] = {1};
  CHECK(homing.start(axes, 1));

  // Run the whole homing sequence with the monitor polled throughout. It must
  // never trip, or homing could never touch a switch.
  bool everTripped = false;
  for (int i = 0; i < 4000000; i++) {
    sim.tick();
    if (i % (int)TICKS_PER_MS == 0) {
      if (safety.update() >= 0) everTripped = true;
      if (stepper.isIdle() && !homing.isBusy()) break;
    }
  }

  CHECK(!everTripped);
  CHECK(!homing.hasFailed());
  CHECK(!stepper.isEmergencyStopped());
  pass("never trips during a homing run");
}

// ---------------------------------------------------------------------------
// Protocol
// ---------------------------------------------------------------------------

static void feed(const char* line) {
  mock::hw.rxQueue += line;
  mock::hw.rxQueue += "\n";
}

/**
 * Run protocol.update() until the receive queue is drained.
 *
 * readSerial() deliberately reads a bounded number of characters per call so a
 * flooding host cannot starve endstop polling, so a single update() does not
 * consume a long feed. The real firmware calls update() every loop pass; this
 * models that.
 */
static void pump(SerialProtocol& protocol) {
  for (int guard = 0; guard < 500; guard++) {
    protocol.update();
    if (mock::hw.rxCursor >= mock::hw.rxQueue.size()) return;
  }
  printf("  FAIL  receive queue never drained\n");
  g_failures++;
}

static bool logContains(const char* needle) {
  return mock::hw.txLog.find(needle) != std::string::npos;
}

static void clearLog() { mock::hw.txLog.clear(); }

static void testProtocol() {
  section("protocol: command parsing and flow control");

  resetWorld();
  StepperController stepper;
  HomingController homing(stepper);
  SerialProtocol protocol(stepper, homing);
  Sim sim(stepper, &homing);
  sim.registerHardware();

  stepper.begin();
  homing.begin();
  protocol.begin(115200);

  clearLog();
  feed("E 1");
  pump(protocol);
  CHECK(logContains("OK Motors enabled"));
  CHECK(stepper.isEnabled());
  pass("E 1 enables the drivers");

  clearLog();
  feed("J 0 10 20 30 40 50 25");
  pump(protocol);
  CHECK(logContains("OK J "));
  CHECK(stepper.queueDepth() == 1);
  pass("a move is queued and the reply reports remaining queue space");

  // Speed is optional.
  clearLog();
  feed("J 0 12 20 30 40 50");
  pump(protocol);
  CHECK(logContains("OK J "));
  pass("the speed argument is optional");

  clearLog();
  feed("J 1 2 3");
  pump(protocol);
  CHECK(logContains("ERROR"));
  pass("a short move command is rejected");

  clearLog();
  feed("X nonsense");
  pump(protocol);
  CHECK(logContains("ERROR Unknown command"));
  pass("an unknown command is reported");

  // Fill the queue and confirm back-pressure rather than silent loss.
  clearLog();
  for (int k = 0; k < MOTION_QUEUE_LENGTH + 4; k++) {
    char line[64];
    snprintf(line, sizeof(line), "J 0 %d 20 30 40 50 25", 14 + k);
    feed(line);
  }
  pump(protocol);
  CHECK(logContains("BUSY "));
  pass("a full queue answers BUSY so the host can retry");

  clearLog();
  feed("Q");
  pump(protocol);
  CHECK(logContains("POS "));
  CHECK(logContains("ENDSTOP "));
  CHECK(logContains("STATUS "));
  pass("Q returns position, endstops and status");

  // The status line must say whether the drivers are live, so the host does not
  // have to assume its own E command worked.
  clearLog();
  feed("E 1");
  feed("Q");
  pump(protocol);
  {
    const size_t at = mock::hw.txLog.rfind("STATUS ");
    const std::string line = mock::hw.txLog.substr(at, mock::hw.txLog.find('\n', at) - at);
    printf("        %s\n", line.c_str());
    CHECK(line.size() > 2 && line[line.size() - 1] == '1');
  }

  clearLog();
  feed("E 0");
  feed("Q");
  pump(protocol);
  {
    const size_t at = mock::hw.txLog.rfind("STATUS ");
    const std::string line = mock::hw.txLog.substr(at, mock::hw.txLog.find('\n', at) - at);
    CHECK(line.size() > 2 && line[line.size() - 1] == '0');
  }
  pass("status reports whether the drivers are energised");

  clearLog();
  feed("S");
  pump(protocol);
  CHECK(logContains("OK Emergency stop"));
  CHECK(stepper.isEmergencyStopped());
  pass("S latches an emergency stop");

  clearLog();
  feed("J 0 10 20 30 40 50 25");
  pump(protocol);
  CHECK(logContains("ERROR"));
  pass("moves are refused while the stop is latched");

  // An over-long line must not overflow the buffer.
  clearLog();
  std::string huge = "J ";
  for (int k = 0; k < 400; k++) huge += "1 ";
  feed(huge.c_str());
  pump(protocol);
  CHECK(logContains("ERROR Command too long"));
  pass("an over-long line is dropped with an error, not overflowed");

  // The parser recovers and accepts the next command.
  clearLog();
  feed("E 0");
  pump(protocol);
  CHECK(logContains("OK Motors disabled"));
  pass("parsing recovers after a dropped line");
}

static void testLivePositionReporting() {
  section("protocol: position reports track the arm while it moves");

  resetWorld();
  StepperController stepper;
  HomingController homing(stepper);
  SerialProtocol protocol(stepper, homing);
  Sim sim(stepper, &homing);
  sim.registerHardware();

  stepper.begin();
  homing.begin();
  protocol.begin(115200);
  stepper.enable();

  CHECK(stepper.queueMove(makeAngles(0, 50, 0, 0, 0, 0), MAX_JOINT_SPEED[1]));

  // Sample the reported angle part way through the move.
  sim.run(0.6);
  const JointAngles mid = stepper.currentAngles();

  // The old firmware only refreshed its angle cache when a move finished, so
  // this read zero for the whole move.
  CHECK(mid[1] > 0.5f);
  CHECK(mid[1] < 50.0f);
  printf("        J2 reported mid-move: %.2f deg\n", (double)mid[1]);
  pass("the reported angle is live during motion");

  CHECK(sim.runUntilIdle(30.0));
  const JointAngles end = stepper.currentAngles();
  CHECK_NEAR(end[1], 50.0f, 1.0f / USTEPS_PER_DEG[1]);
  pass("and settles on the target");
}

// ---------------------------------------------------------------------------

int main() {
  printf("Firmware motion tests\n");
  printf("=====================\n");

  testPlannerLimits();
  testPlannerRequestedSpeed();
  testPlannerCollinearAndReversal();
  testPlannerQueueAccounting();
  testProfileShape();

  testExactPositioning();
  testNoInstantStart();
  testStreamingStaysContinuous();
  testDecelerateToStop();
  testEmergencyStop();
  testJointLimitClamping();

  testHomingBookkeeping();
  testHomingMissingEndstop();
  testHomingClearsADeeplyPressedSwitch();
  testHomingStillFailsOnAStuckSwitch();
  testHomingRefusesJointsWithoutEndstops();

  testSafetyMonitor();
  testSafetyIgnoresHoming();

  testProtocol();
  testLivePositionReporting();

  printf("\n=====================\n");
  printf("%d checks, %d failures\n", g_checks, g_failures);
  return g_failures == 0 ? 0 : 1;
}
