# 6DOF Robot Arm

Control software for a DIY 6-axis robot arm: Teensy 4.1 firmware for motion, and
a browser app for kinematics, path planning and 3D visualisation over Web Serial.

```
robot-arm-control/     React + TypeScript app (kinematics, planning, 3D view)
firmware/              Teensy 4.1 firmware (step generation, homing, protocol)
firmware/test/         Host-side firmware tests, no board required
docs/                  Hardware, kinematics and protocol reference
stl_meshes/            Link geometry for the 3D view
URDF.md                Link and joint geometry, the source for the kinematic chain
```

## Quick start

**Firmware.** Open `firmware/firmware.ino` in the Arduino IDE with Teensyduino,
select Teensy 4.1, USB Type Serial, 600 MHz, and upload.

**App.**

```bash
cd robot-arm-control
npm install
npm start
```

Then open it in Chrome or Edge — Web Serial exists nowhere else — and press
*Connect to Robot*.

**Before moving the arm for the first time**, work through the bring-up sequence
in [firmware/README.md](firmware/README.md#bring-up-on-the-arm). Direction
inversion and homing direction are not verified for this build; see
[Unresolved conflicts](docs/HARDWARE.md#unresolved-conflicts). Getting them wrong
drives a joint into a hard stop.

## Tests

```bash
make -C firmware/test test              # 593 checks, no hardware needed
cd robot-arm-control && npm test        # 83 tests
```

The firmware tests compile the real `StepperController`, `HomingController`,
`MotionPlanner` and `SerialProtocol` against a mock Arduino layer, so what is
tested is what runs on the Teensy. The mock counts STEP pulses into a virtual
motor position, which is how the homing tests check where the motor really ends up
against what the firmware believes.

CI runs both on every push.

## How it fits together

```
   Browser                                    Teensy 4.1
   ┌──────────────────────────────┐           ┌──────────────────────────────┐
   │ UI, 3D view (three.js)       │           │ SerialProtocol               │
   │ ────────────────────────────  │           │  queue space, back-pressure  │
   │ Kinematics                   │  USB      │ ────────────────────────────  │
   │  FK from the URDF chain      │◄─────────►│ MotionPlanner                │
   │  IK: damped least squares    │  ASCII    │  look-ahead, junction speeds │
   │ ────────────────────────────  │  lines    │ ────────────────────────────  │
   │ Path planning                │           │ StepperController            │
   │  waypoints → joint poses     │           │  100 kHz timer ISR, ramps    │
   └──────────────────────────────┘           └──────────────────────────────┘
                                                          │
                                              6× TMC2209 → steppers
```

The division of labour matters: **the app produces geometry, the firmware owns
timing.** The app turns waypoints into joint poses and streams them; the firmware
holds a queue of up to 23 and plans one continuous, acceleration-limited velocity
profile across the whole lot. The app must keep that queue fed and must not pace
points off its own clock — queue depth is what turns a series of points into one
smooth motion instead of a stop at every one.

Joint limits live in two places by necessity — `firmware/config.h` and
`robot-arm-control/src/kinematics/robotModel.ts` — and a test asserts they match.
If the solver works in a different box than the firmware enforces, the firmware
silently clamps the solution and the arm ends up somewhere the solver never asked
for.

## Documentation

| | |
|---|---|
| [docs/HARDWARE.md](docs/HARDWARE.md) | Motors, reductions, pins, limits, calibration, and the unresolved config conflicts |
| [docs/KINEMATICS.md](docs/KINEMATICS.md) | The kinematic chain, FK, the Jacobian, the IK solver, workspace |
| [docs/SERIAL_PROTOCOL.md](docs/SERIAL_PROTOCOL.md) | Wire format, flow control, how to stream a trajectory |
| [firmware/README.md](firmware/README.md) | Firmware architecture, tuning, bring-up, troubleshooting |
| [URDF.md](URDF.md) | Link and joint geometry |

## Current state

Working: forward and inverse kinematics, 3D visualisation, joint and Cartesian
jogging, teach-and-playback of waypoint paths, homing, acceleration-limited
coordinated motion, and reconnection after the link drops.

Not verified on hardware: direction inversion, homing direction, and the
calibration constants. The software has been tested as far as it can be without
the arm — everything mechanical and electrical still needs the bring-up run.

Not built: G-code import, SVG tracing, and the Electron desktop wrapper that the
original project notes list as later goals.

## Safety

- The drivers start disabled; `E 1` energises them.
- `S` is an emergency stop: it cuts step pulses immediately and keeps the drivers
  energised so the arm holds position. It can lose steps, so it marks the position
  untrusted and the arm needs re-homing.
- `A` decelerates to a stop within the acceleration limit and keeps the position.
  That is the right stop for a cancel.
- Disabling the drivers lets a geared arm sag under its own weight.
- Soft limits are only as good as the calibration behind them. Keep mechanical
  hard stops as the last line of defence.
