#include "TrajectoryExecutor.h"
#include <math.h>

TrajectoryExecutor::TrajectoryExecutor(StepperController& stepper)
  : stepper_(stepper),
    pointCount_(0),
    running_(false),
    startMs_(0),
    startUs_(0),
    elapsedMs_(0),
    currentPointIndex_(0),
    lastTickUs_(0),
    progressEventPending_(false),
    progressPointIndex_(0),
    progressElapsedMs_(0),
    lastProgressEmitMs_(0),
    lastProgressPointIndex_(0),
    doneEventPending_(false),
    tickJitterUsMax_(0),
    queueUnderrunCount_(0),
    stepOverrunCount_(0),
    decelerationTickCount_(0) {}

void TrajectoryExecutor::begin() {
  clear();
  resetDiagnostics();
}

void TrajectoryExecutor::clear() {
  running_ = false;
  pointCount_ = 0;
  startMs_ = 0;
  startUs_ = 0;
  elapsedMs_ = 0;
  currentPointIndex_ = 0;
  lastTickUs_ = 0;
  progressEventPending_ = false;
  doneEventPending_ = false;
  lastProgressEmitMs_ = 0;
  lastProgressPointIndex_ = 0;
  decelerationTickCount_ = 0;
}

void TrajectoryExecutor::resetDiagnostics() {
  tickJitterUsMax_ = 0;
  queueUnderrunCount_ = 0;
  stepOverrunCount_ = 0;
}

bool TrajectoryExecutor::validatePoint(const TrajectoryPointCommand& point, String& errorMessage) const {
  for (int i = 0; i < 6; i++) {
    if (!isfinite(point.qDeg[i]) || !isfinite(point.qdDegS[i])) {
      errorMessage = "non-finite";
      return false;
    }
    if (point.qDeg[i] < JOINT_MIN[i] || point.qDeg[i] > JOINT_MAX[i]) {
      errorMessage = String("joint range J") + String(i + 1);
      return false;
    }
    if (fabs(point.qdDegS[i]) > STREAM_MAX_SPEED_DEG_S * 2.0f) {
      errorMessage = String("velocity range J") + String(i + 1);
      return false;
    }
  }
  return true;
}

bool TrajectoryExecutor::addPoint(
  uint32_t tMs,
  const JointAngles& qDeg,
  const JointAngles& qdDegS,
  String& errorMessage
) {
  if (running_) {
    errorMessage = "running";
    return false;
  }

  if (pointCount_ >= TRAJECTORY_MAX_POINTS) {
    errorMessage = "queue full";
    return false;
  }

  if (pointCount_ > 0 && tMs <= points_[pointCount_ - 1].tMs) {
    errorMessage = "time not monotonic";
    return false;
  }

  TrajectoryPointCommand point;
  point.tMs = tMs;
  point.qDeg = qDeg;
  point.qdDegS = qdDegS;
  if (!validatePoint(point, errorMessage)) {
    return false;
  }

  points_[pointCount_] = point;
  pointCount_++;
  return true;
}

bool TrajectoryExecutor::run(String& errorMessage) {
  if (running_) {
    errorMessage = "already running";
    return false;
  }
  if (pointCount_ < 2) {
    errorMessage = "need >=2 points";
    return false;
  }
  if (!stepper_.isEnabled()) {
    errorMessage = "motors disabled";
    return false;
  }

  running_ = true;
  doneEventPending_ = false;
  progressEventPending_ = false;
  startUs_ = micros();
  startMs_ = millis();
  elapsedMs_ = 0;
  currentPointIndex_ = 0;
  lastTickUs_ = startUs_;
  lastProgressEmitMs_ = 0;
  lastProgressPointIndex_ = 0;
  decelerationTickCount_ = 0;

  stepper_.clearStop();
  stepper_.setStreamingCommand(StreamingMotionCommand(points_[0].qDeg, points_[0].qdDegS));
  return true;
}

void TrajectoryExecutor::stop() {
  running_ = false;
}

void TrajectoryExecutor::emergencyStop() {
  running_ = false;
  stepper_.emergencyStop();
}

void TrajectoryExecutor::evaluateHermite(
  float elapsedMs,
  StreamingMotionCommand& outCommand,
  uint16_t& outSegment
) const {
  if (pointCount_ == 0) {
    outCommand = StreamingMotionCommand();
    outSegment = 0;
    return;
  }

  if (elapsedMs <= (float)points_[0].tMs) {
    outCommand = StreamingMotionCommand(points_[0].qDeg, points_[0].qdDegS);
    outSegment = 0;
    return;
  }

  if (elapsedMs >= (float)points_[pointCount_ - 1].tMs) {
    outCommand = StreamingMotionCommand(points_[pointCount_ - 1].qDeg, points_[pointCount_ - 1].qdDegS);
    outSegment = pointCount_ - 1;
    return;
  }

  uint16_t segment = 0;
  for (uint16_t i = 0; i < pointCount_ - 1; i++) {
    if (elapsedMs >= (float)points_[i].tMs && elapsedMs <= (float)points_[i + 1].tMs) {
      segment = i;
      break;
    }
  }
  outSegment = segment;

  const TrajectoryPointCommand& p0 = points_[segment];
  const TrajectoryPointCommand& p1 = points_[segment + 1];

  float t0 = (float)p0.tMs * 0.001f;
  float t1 = (float)p1.tMs * 0.001f;
  float t = elapsedMs * 0.001f;
  float dt = t1 - t0;
  if (dt < 1e-6f) {
    outCommand = StreamingMotionCommand(p1.qDeg, p1.qdDegS);
    return;
  }

  float s = (t - t0) / dt;
  if (s < 0.0f) s = 0.0f;
  if (s > 1.0f) s = 1.0f;
  float s2 = s * s;
  float s3 = s2 * s;

  float h00 = 2.0f * s3 - 3.0f * s2 + 1.0f;
  float h10 = s3 - 2.0f * s2 + s;
  float h01 = -2.0f * s3 + 3.0f * s2;
  float h11 = s3 - s2;

  float dh00 = 6.0f * s2 - 6.0f * s;
  float dh10 = 3.0f * s2 - 4.0f * s + 1.0f;
  float dh01 = -6.0f * s2 + 6.0f * s;
  float dh11 = 3.0f * s2 - 2.0f * s;

  for (int j = 0; j < 6; j++) {
    float q0 = p0.qDeg[j];
    float q1 = p1.qDeg[j];
    float v0 = p0.qdDegS[j];
    float v1 = p1.qdDegS[j];

    outCommand.targetAngles[j] = h00 * q0 + h10 * dt * v0 + h01 * q1 + h11 * dt * v1;
    outCommand.targetVelocityDegS[j] = (dh00 / dt) * q0 + dh10 * v0 + (dh01 / dt) * q1 + dh11 * v1;
  }
}

void TrajectoryExecutor::update() {
  if (!running_) return;

  if (stepper_.isStopRequested()) {
    running_ = false;
    return;
  }

  unsigned long nowUs = micros();
  unsigned long dtUs = nowUs - lastTickUs_;
  if (dtUs < TRAJECTORY_TICK_US) return;

  unsigned long jitter = (dtUs > TRAJECTORY_TICK_US)
    ? (dtUs - TRAJECTORY_TICK_US)
    : (TRAJECTORY_TICK_US - dtUs);
  if (jitter > tickJitterUsMax_) {
    tickJitterUsMax_ = jitter;
  }
  if (dtUs > (TRAJECTORY_TICK_US * 2UL)) {
    stepOverrunCount_++;
  }

  lastTickUs_ = nowUs;

  // Use microseconds elapsed for sub-millisecond Hermite timing precision.
  // millis() has 1ms resolution and drifts relative to the micros()-based tick,
  // causing every other tick to evaluate the same Hermite point twice (stutter)
  // or skip a point (lurch). Float microseconds removes that quantization entirely.
  uint32_t elapsedUs = nowUs - startUs_;
  elapsedMs_ = elapsedUs / 1000;
  float elapsedMsFloat = (float)elapsedUs / 1000.0f;

  // End of trajectory: deceleration continuation phase.
  // Keep calling setStreamingCommand with the final position and zero velocity at every
  // 1 kHz tick so the speed slew ramps appliedSpeedDegS toward STREAM_MIN instead of
  // stopping abruptly when the last Hermite knot is passed.
  if (elapsedMsFloat >= (float)points_[pointCount_ - 1].tMs) {
    currentPointIndex_ = pointCount_ - 1;
    JointAngles zeroVel;
    for (int i = 0; i < 6; i++) zeroVel[i] = 0.0f;
    stepper_.setStreamingCommand(
      StreamingMotionCommand(points_[pointCount_ - 1].qDeg, zeroVel)
    );
    decelerationTickCount_++;
    if (!stepper_.isMoving() || decelerationTickCount_ >= TRAJECTORY_DECELERATION_TICKS) {
      running_ = false;
      decelerationTickCount_ = 0;
      doneEventPending_ = true;
    }
    return;
  }

  StreamingMotionCommand command;
  uint16_t segment = 0;
  evaluateHermite(elapsedMsFloat, command, segment);
  currentPointIndex_ = segment;

  if (segment >= pointCount_) {
    queueUnderrunCount_++;
    segment = pointCount_ - 1;
    currentPointIndex_ = segment;
  }

  stepper_.setStreamingCommand(command);

  bool emitProgress = false;
  if (segment != lastProgressPointIndex_) {
    emitProgress = true;
  } else if (elapsedMs_ - lastProgressEmitMs_ >= TRAJECTORY_PROGRESS_INTERVAL_MS) {
    emitProgress = true;
  }

  if (emitProgress) {
    progressEventPending_ = true;
    progressPointIndex_ = segment;
    progressElapsedMs_ = elapsedMs_;
    lastProgressEmitMs_ = elapsedMs_;
    lastProgressPointIndex_ = segment;
  }
}

bool TrajectoryExecutor::isRunning() const {
  return running_;
}

bool TrajectoryExecutor::hasPoints() const {
  return pointCount_ > 0;
}

uint16_t TrajectoryExecutor::getPointCount() const {
  return pointCount_;
}

uint16_t TrajectoryExecutor::getCurrentPointIndex() const {
  return currentPointIndex_;
}

uint32_t TrajectoryExecutor::getElapsedMs() const {
  return elapsedMs_;
}

bool TrajectoryExecutor::consumeDoneEvent() {
  if (!doneEventPending_) return false;
  doneEventPending_ = false;
  return true;
}

bool TrajectoryExecutor::consumeProgressEvent(uint16_t& pointIndex, uint32_t& elapsedMs) {
  if (!progressEventPending_) return false;
  progressEventPending_ = false;
  pointIndex = progressPointIndex_;
  elapsedMs = progressElapsedMs_;
  return true;
}

MotionKernelDiagnostics TrajectoryExecutor::getDiagnostics() const {
  MotionKernelDiagnostics diag;
  diag.tickJitterUs = tickJitterUsMax_;
  diag.queueUnderrun = queueUnderrunCount_;
  diag.stepOverrun = stepOverrunCount_;
  return diag;
}
