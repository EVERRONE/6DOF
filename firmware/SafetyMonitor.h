#ifndef SAFETY_MONITOR_H
#define SAFETY_MONITOR_H

#include "config.h"
#include "types.h"
#include "HomingController.h"
#include "StepperController.h"

/**
 * Watches for an endstop closing when it should not.
 *
 * During ordinary motion a closed switch means the arm is somewhere it should not
 * be, with the hard stop close behind. This is the last line of software defence,
 * which is exactly why it does not live in firmware.ino: the sketch is the one
 * file the host tests cannot compile, so safety logic kept there would be the
 * only untested safety logic in the project.
 */
class SafetyMonitor {
public:
  SafetyMonitor(StepperController& stepper, HomingController& homing);

  /**
   * Check the endstops and stop the arm if one has tripped. Call every loop pass,
   * after the endstops have been polled.
   *
   * @return the axis that just tripped, or -1. Returns an axis at most once per
   *         movement, so a stuck switch cannot flood the link; it re-arms when
   *         motion has stopped.
   */
  int update();

  /** True while a trip is latched, i.e. until motion stops again. */
  bool isTripped() const { return tripped_; }

private:
  StepperController& stepper_;
  HomingController& homing_;
  bool tripped_;
};

#endif
