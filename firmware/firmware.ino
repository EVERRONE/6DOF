#include "config.h"
#include "types.h"
#include "StepperController.h"
#include "HomingController.h"
#include "SerialProtocol.h"
#include "TrajectoryExecutor.h"
#include <math.h>

// Global objects
StepperController stepper;
HomingController homing(stepper);
TrajectoryExecutor trajectory(stepper);
SerialProtocol protocol(stepper, homing, trajectory);

void setup() {
  // Initialize subsystems
  stepper.begin();
  homing.begin();
  trajectory.begin();
  protocol.begin(115200);

  // Safety: start with motors disabled
  stepper.disable();
}

void loop() {
  // Handle serial communication
  protocol.update();

  // Update stepper controller
  stepper.update();

  // Safety: stop only when a joint is moving into its triggered endstop
  if (stepper.isMoving()) {
    JointAngles current = stepper.getCurrentAngles();
    JointAngles target = stepper.getTargetAngles();
    const float epsilon = 0.001f;

    for (int i = 0; i < 6; i++) {
      if (!homing.isEndstopTriggered(i)) continue;

      float delta = target[i] - current[i];
      if (fabs(delta) <= epsilon) continue;  // Joint is not moving

      bool movingTowardEndstop = HOME_TOWARD_MIN[i] ? (delta < 0.0f) : (delta > 0.0f);
      if (movingTowardEndstop) {
        stepper.emergencyStop();
        Serial.println("ENDSTOP_STOP");
        break;
      }
    }
  }

  // Small delay to prevent overwhelming the CPU
  delayMicroseconds(10);
}
