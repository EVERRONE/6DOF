#ifndef TRAJECTORY_EXECUTOR_H
#define TRAJECTORY_EXECUTOR_H

#include "config.h"
#include "types.h"
#include "StepperController.h"

struct TrajectoryPointCommand {
  uint32_t tMs;
  JointAngles qDeg;
  JointAngles qdDegS;
};

struct MotionKernelDiagnostics {
  uint32_t tickJitterUs;
  uint32_t queueUnderrun;
  uint32_t stepOverrun;
};

class TrajectoryExecutor {
public:
  explicit TrajectoryExecutor(StepperController& stepper);

  void begin();
  void clear();
  bool addPoint(uint32_t tMs, const JointAngles& qDeg, const JointAngles& qdDegS, String& errorMessage);
  bool run(String& errorMessage);
  void stop();
  void emergencyStop();
  void update();

  bool isRunning() const;
  bool hasPoints() const;
  uint16_t getPointCount() const;
  uint16_t getCurrentPointIndex() const;
  uint32_t getElapsedMs() const;

  bool consumeDoneEvent();
  bool consumeProgressEvent(uint16_t& pointIndex, uint32_t& elapsedMs);
  MotionKernelDiagnostics getDiagnostics() const;
  void resetDiagnostics();

private:
  StepperController& stepper_;
  TrajectoryPointCommand points_[TRAJECTORY_MAX_POINTS];
  uint16_t pointCount_;

  bool running_;
  uint32_t startMs_;
  uint32_t startUs_;
  uint32_t elapsedMs_;
  uint16_t currentPointIndex_;
  unsigned long lastTickUs_;

  bool progressEventPending_;
  uint16_t progressPointIndex_;
  uint32_t progressElapsedMs_;
  uint32_t lastProgressEmitMs_;
  uint16_t lastProgressPointIndex_;

  bool doneEventPending_;
  uint32_t tickJitterUsMax_;
  uint32_t queueUnderrunCount_;
  uint32_t stepOverrunCount_;
  uint16_t decelerationTickCount_;

  bool validatePoint(const TrajectoryPointCommand& point, String& errorMessage) const;
  void evaluateHermite(float elapsedMs, StreamingMotionCommand& outCommand, uint16_t& outSegment) const;
};

#endif
