// 6DOF robot arm firmware - Teensy 4.1
//
// The main loop does no step generation and contains no blocking delay. Steps
// come from a timer interrupt (see StepperController), homing is a state machine
// (see HomingController), and the loop only shuttles data and makes decisions.
// That is what keeps step timing clean: anything that blocks here shows up as
// jitter on every pulse.

#include "config.h"
#include "types.h"
#include "MotionPlanner.h"
#include "StepperController.h"
#include "HomingController.h"
#include "SerialProtocol.h"

StepperController stepper;
HomingController homing(stepper);
SerialProtocol protocol(stepper, homing);

void setup() {
  stepper.begin();   // also starts the step timer; drivers begin disabled
  homing.begin();
  protocol.begin(115200);
}

void loop() {
  // Endstops first, so the safety check below and the homing state machine both
  // act on this pass's reading.
  homing.pollEndstops();

  protocol.update();
  stepper.service();
  homing.update();

  // Safety: an endstop closing during ordinary motion means the arm is
  // somewhere it should not be, and the hard stop is close behind.
  //
  // This stops immediately rather than decelerating. Deceleration would be
  // gentler on the mechanics but takes distance - J3 at 60 deg/s needs about
  // 12 degrees to stop within its acceleration limit - which is more than the
  // margin between an endstop and its hard stop. Cutting the pulses may lose
  // steps, so the stop also marks the position untrusted and the arm needs
  // re-homing, which is the right outcome after an unexpected limit hit.
  static bool endstopTripped = false;

  if (!homing.isBusy() && stepper.isMoving()) {
    if (!endstopTripped) {
      for (int i = 0; i < NUM_AXES; i++) {
        if (homing.isEndstopTriggered(i)) {
          stepper.emergencyStop();
          endstopTripped = true;
          Serial.print("ERROR Endstop triggered during move on J");
          Serial.println(i + 1);
          break;
        }
      }
    }
  } else if (!stepper.isMoving()) {
    // Re-arm once motion has stopped, so a stuck switch reports once per move
    // instead of flooding the link every loop pass.
    endstopTripped = false;
  }
}
