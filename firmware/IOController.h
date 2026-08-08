#ifndef IO_CONTROLLER_H
#define IO_CONTROLLER_H

#include <Arduino.h>
#include "config.h"

/**
 * Digital outputs and inputs.
 *
 * The arm's only way to do anything besides move. An output drives a gripper, a
 * valve, a light or a signal to another machine; an input reads a sensor that a
 * path can wait on.
 *
 * Two things here are not obvious and both are about not surprising whoever is
 * standing next to the machine:
 *
 *  - **Outputs have a safe state**, applied at start-up and when the drivers are
 *    switched off. A gripper energised by a program that crashed, or by a board
 *    that reset mid-cycle, is a hazard nobody is watching. An emergency stop is
 *    deliberately left alone: opening a gripper mid-stop drops the part, which
 *    is usually worse than the stop.
 *
 *  - **Inputs are debounced**, on the same counter as the endstops. A mechanical
 *    sensor bounces for milliseconds; a path step waiting on one would otherwise
 *    continue on a contact whisker rather than on the part actually arriving.
 */
class IOController {
public:
  IOController();

  /** Configure the pins and drive every output to its safe state. */
  void begin();

  /** Sample the inputs. Call every loop pass. */
  void update();

  /** Drive one output. False when the index is outside the configured set. */
  bool setOutput(uint8_t index, bool high);

  /** What an output is currently driven to. */
  bool outputState(uint8_t index) const;

  /** A debounced input reading, already corrected for active-low wiring. */
  bool inputState(uint8_t index) const;

  /**
   * Drive every output to its configured safe state.
   *
   * Called at start-up and when the drivers are de-energised. Not called on an
   * emergency stop - see the note above.
   */
  void allSafe();

private:
  bool outputs_[NUM_OUTPUTS];

  // Debounce, one counter per input. Identical in shape to the endstop
  // debouncing in StepperController, deliberately: two mechanisms that behave
  // differently under a bouncing contact would be two things to reason about.
  bool inputs_[NUM_INPUTS];
  bool rawLast_[NUM_INPUTS];
  uint8_t stable_[NUM_INPUTS];
};

#endif
