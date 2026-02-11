#include "config.h"
#include "types.h"
#include "StepperController.h"
#include "HomingController.h"
#include "SerialProtocol.h"

// Global objects
StepperController stepper;
HomingController homing(stepper);
SerialProtocol protocol(stepper, homing);

void setup() {
  // Initialize subsystems
  stepper.begin();
  homing.begin();
  protocol.begin(115200);

  // Safety: start with motors disabled
  stepper.disable();
}

void loop() {
  // Handle serial communication
  protocol.update();

  // Update stepper controller
  stepper.update();

  // Safety: stop if endstop triggered during normal movement
  if (stepper.isMoving()) {
    for (int i = 0; i < 6; i++) {
      if (homing.isEndstopTriggered(i)) {
        stepper.emergencyStop();
        Serial.println("ENDSTOP_STOP");
        break;
      }
    }
  }

  // Small delay to prevent overwhelming the CPU
  delayMicroseconds(10);
}
