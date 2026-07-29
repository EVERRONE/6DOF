// 6DOF robot arm firmware - Teensy 4.1
//
// The main loop does no step generation and contains no blocking delay. Steps
// come from a timer interrupt (see StepperController), homing is a state machine
// (see HomingController), and the endstop safety trip lives in SafetyMonitor.
// What is left here is wiring: poll, shuttle, report.
//
// That is deliberate. Anything that blocks in this loop shows up as jitter on
// every step pulse, and anything with logic in it cannot be reached by the host
// tests in firmware/test, which do not compile this file.

#include "config.h"
#include "types.h"
#include "MotionPlanner.h"
#include "StepperController.h"
#include "HomingController.h"
#include "SafetyMonitor.h"
#include "SerialProtocol.h"

StepperController stepper;
HomingController homing(stepper);
SafetyMonitor safety(stepper, homing);
SerialProtocol protocol(stepper, homing);

void setup() {
  stepper.begin();   // also starts the step timer; drivers begin disabled
  homing.begin();
  protocol.begin(115200);
}

void loop() {
  // Endstops first, so the safety check and the homing state machine both act on
  // this pass's reading.
  homing.pollEndstops();

  protocol.update();
  stepper.service();
  homing.update();

  const int tripped = safety.update();
  if (tripped >= 0) {
    Serial.print("ERROR Endstop triggered during move on J");
    Serial.println(tripped + 1);
  }
}
