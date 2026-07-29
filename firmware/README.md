# 6DOF Robot Arm - Teensy Firmware

Controls a 6-axis arm with a Teensy 4.1 and TMC2209 stepper drivers.

## Hardware

- Teensy 4.1 (600 MHz)
- 6x TMC2209 stepper drivers, 1/16 microstepping
- 6x NEMA stepper motors
- 4x Omron D2F-L endstops (J2-J5)
- 24 V supply

### Pins

| Joint | STEP | DIR | Endstop | Reduction |
|-------|------|-----|---------|-----------|
| J1 | 2 | 5 | - | 6.3 : 1 |
| J2 | 3 | 6 | 30 | 25 : 1 |
| J3 | 4 | 7 | 31 | 6.3 : 1 |
| J4 | 22 | 23 | 32 | 3.7 : 1 |
| J5 | 24 | 25 | 33 | 2 : 1 |
| J6 | 26 | 27 | - | 1 : 1 |

Enable: pin 8 (drivers J1-J3) and pin 9 (drivers J4-J6), both active LOW.
Endstop wiring: C to the Teensy pin, NO to GND. `INPUT_PULLUP` is enabled, so a
closed switch reads LOW.

## Architecture

| File | Responsibility |
|------|----------------|
| `config.h` | Pins, limits, calibration, motion tuning. Host-safe: no `<Arduino.h>`. |
| `types.h` | `JointAngles`, `EndstopState`, `RobotState`. |
| `MotionPlanner.h/.cpp` | Move queue, velocity profiles, junction planning. No Arduino dependency, so it can be unit-tested on a host. |
| `StepperController.h/.cpp` | Timer-interrupt step generation, position tracking, stopping. |
| `HomingController.h/.cpp` | Homing as a non-blocking state machine, endstop debouncing. |
| `SafetyMonitor.h/.cpp` | Stops the arm if an endstop closes during ordinary motion. |
| `SerialProtocol.h/.cpp` | Host line protocol. See `docs/SERIAL_PROTOCOL.md`. |
| `firmware.ino` | Wiring and the main loop. Deliberately holds no logic: it is the one file the host tests cannot compile. |
| `test/` | Host-side unit tests. |

### How motion works

**Steps come from a timer interrupt at `STEP_ISR_HZ` (100 kHz), not from the main
loop.** The interrupt advances a phase accumulator at the rate the velocity
profile asks for, and distributes each step event across the axes by Bresenham
against the axis with the most travel, so a multi-axis move stays coordinated to
within one microstep. A step spans two ticks - one high, one low - which
satisfies the driver's pulse timing without a single blocking delay and caps the
step rate at 50 kHz. The fastest axis needs 8.9 kHz.

**Every move is acceleration limited.** The profile is expressed as velocity
against position:

```
v(s) = min( cruise, sqrt(entry^2 + 2*a*s), sqrt(exit^2 + 2*a*(end - s)) )
```

which needs no phase bookkeeping, always arrives at no more than the planned exit
speed, and degrades to a triangular profile on a short move with no special case.

**Moves are queued and planned together.** `MOTION_QUEUE_LENGTH - 1` moves are
held, and a reverse/forward pass pair sets each junction speed so consecutive
moves run as one continuous motion. Junction speed scales with the cosine of the
turn angle in joint space: collinear moves pass at full speed, a reversal stops.

**The main loop never blocks.** It polls endstops, shuttles serial data, starts
queued moves and advances the homing state machine. Anything that blocks there
shows up as jitter on every step pulse.

### Why it is built this way

The previous version generated steps from `loop()` with blocking
`delayMicroseconds()` calls and held a single target with a constant step
interval. Three consequences, all of which the arm made audible:

- **No acceleration.** J2 (25:1) was asked to go from standstill to 125-250 RPM
  at the motor in one step, far beyond the pull-in torque of a loaded NEMA17, so
  it lost steps and ground.
- **Timing jitter of +/-27-53% of a step period,** from the blocking delays and
  from stamping each step with the current time instead of accumulating the
  interval, which also let the axes drift out of sync during a move.
- **Ten start/stop cycles per second** while streaming a trajectory, because
  every command replaced the target and restarted a constant-rate move.

A fourth, worse problem was in homing: setting the datum assigned a whole
zero-filled `JointAngles`, wiping the calibration of every other axis. After
`H ALL` the firmware believed J2, J3 and J4 were at 0 while they were parked at
5, 55 and 129 degrees, so the next ordinary move drove J3 to 110 degrees against
a 70 degree limit - straight into the hard stop. `setJointAngle` now touches one
axis, and a test asserts the whole sequence.

## Tuning

Everything worth tuning is in `config.h`.

### Joint limits

`JOINT_MIN` / `JOINT_MAX`, in degrees. **These are mirrored in the web app** at
`robot-arm-control/src/kinematics/robotModel.ts`, and a test there asserts the
exact values, so change both together. If the solver works in a different box
than the firmware, the firmware silently clamps the solution and the arm ends up
somewhere the solver never asked for.

### Calibration

`USTEPS_PER_DEG` - microsteps per degree of joint travel, including the gear
reduction.

To calibrate one axis: home it, command a known move (`J 0 0 45 0 0 0 10`),
measure what you actually get, and scale that axis's value by
`commanded / measured`. Re-upload and repeat until it converges.

### Speed and acceleration

`MAX_JOINT_SPEED` and `MAX_JOINT_ACCEL` are the physical capability of each axis.
Every move is scaled so no axis exceeds either, whatever the host asks for.

Acceleration is the parameter that decides whether the arm is quiet. Raise it
only while listening: **if a joint knocks as it starts or stops, its acceleration
is too high for the load.** J2 carries the whole arm and wants the gentlest ramp.

Raising `MAX_JOINT_SPEED` past the point where the driver holds torque will bring
the grinding back, ramp or no ramp.

### Homing

| Constant | Meaning |
|----------|---------|
| `HOME_TOWARD_MIN` | Seek direction per axis |
| `HOME_POSITION` | Angle assigned when the switch is found |
| `POST_HOME_ANGLES` | Where the joint parks afterwards |
| `HOMING_SPEED` | Fast seek |
| `HOMING_FINE_SPEED` | Slow second approach; this sets the repeatability of the datum |
| `BACKOFF_DISTANCE` | Retraction between the two approaches |

`BACKOFF_DISTANCE` must exceed the stop distance at `HOMING_SPEED`, which is
`v^2 / (2a)`. At the stock values that is well under a degree.

## Uploading

1. Arduino IDE 2.x with [Teensyduino](https://www.pjrc.com/teensy/td_download.html)
2. Open `firmware.ino`
3. **Tools > Board > Teensy 4.1**
4. **Tools > USB Type > Serial**
5. **Tools > CPU Speed > 600 MHz**
6. Upload

## Tests

The motion path has host-side unit tests. They compile the real
`StepperController`, `HomingController`, `MotionPlanner` and `SerialProtocol`
against a mock Arduino layer, so what is tested is what runs on the Teensy. The
mock counts STEP pulses into a virtual motor position, which lets a test compare
where the motor actually ends up against what the firmware believes.

```bash
cd firmware/test
make
./test_motion
```

Covered: per-axis speed and acceleration limits, the requested-speed mapping,
junction planning for collinear, shallow, right-angle and reversing moves, queue
accounting and back-pressure, profile shape including the triangular case, exact
arrival on target, the acceleration ramp, continuity across a 20-point stream,
graceful and emergency stops, joint-limit clamping, homing bookkeeping and its
failure modes, the endstop safety trip and that it stays quiet during homing,
command parsing, and live position reporting.

These tests cannot check anything electrical or mechanical. Bring-up on the real
arm still needs the checks below.

## Bring-up on the arm

The full ordered checklist is [docs/BRINGUP.md](../docs/BRINGUP.md). In short, with
the arm free to move and a hand near the power switch:

1. **Direction.** `E 1`, then a small move per joint (`J 0 5 0 0 0 0 5`). If a
   joint goes the wrong way, flip its entry in `INVERT_DIR`.
2. **Calibration.** Command a known move per joint and measure it. Adjust
   `USTEPS_PER_DEG`.
3. **Endstops.** Press each switch by hand and watch the `ENDSTOP` report. Fix
   wiring before homing anything.
4. **Homing, one joint at a time.** `H 2`, and watch it: fast seek, back off,
   slow approach, park at 5 degrees, `HOMED 2`. Only then try `H ALL`.
5. **Acceleration.** Run a full-speed move per joint and listen. Knocking at the
   start or end means `MAX_JOINT_ACCEL` is too high for that joint.
6. **Emergency stop.** Start a slow move, send `S` mid-move, confirm it stops and
   that the app shows `POSITION UNVERIFIED`.

## Safety

- Drivers start disabled. `E 1` energises them.
- `E 0` and a disable stop motion first, then cut the drivers. A geared arm can
  sag once they are off.
- `S` cuts pulses immediately and keeps the drivers energised, so the arm holds
  position. It may lose steps, so it clears the position-trusted flag: re-home
  before relying on the reported angles.
- `A` decelerates to a stop instead, and keeps the position. Use it for a cancel.
- An endstop closing during ordinary motion triggers an immediate stop.
  Decelerating would be gentler but needs distance - J3 at 60 deg/s needs about
  12 degrees - which is more than the margin between an endstop and its hard
  stop.
- Homing deliberately ignores the joint limits while seeking, because before the
  arm is homed its recorded position is arbitrary. The switch is the limit during
  that move.
- Keep mechanical hard stops as the last line of defence. Soft limits are only as
  good as the calibration behind them.

## Troubleshooting

**A joint knocks or grinds when starting or stopping.** Its `MAX_JOINT_ACCEL` is
too high for the load. Lower it.

**A joint grinds at speed.** Its `MAX_JOINT_SPEED` is past the point where the
driver holds torque. Lower it, or raise the driver current.

**A joint does not move.** Check the enable pin is LOW, the driver Vref
(~0.6-1.0 V depending on the motor), the 24 V supply, and that `E 1` was sent.

**Positions drift over repeated moves.** Check `USTEPS_PER_DEG`, then mechanical
binding, then driver current - a motor losing steps under load reads as drift.

**An endstop is never detected.** Check the wiring (C to the pin, NO to GND) and
that the switch reads LOW when pressed.

**Homing reports `Endstop not found within travel limit`.** Wiring, or
`HOME_TOWARD_MIN` has the wrong direction for that axis.

**Homing reports `Endstop still closed after back-off`.** The switch is stuck or
miswired. Homing stops rather than drive into the hard stop.

**`POSITION UNVERIFIED` in the web app.** Either the arm has not been homed, or a
hard stop happened. Re-home.
