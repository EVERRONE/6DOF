# 6DOF Robot Arm — Web Control Application

React + TypeScript app for driving the arm over Web Serial: kinematics, path
planning, and a live 3D view.

See the [repository README](../README.md) for how this fits with the firmware, and
[docs/KINEMATICS.md](../docs/KINEMATICS.md) for the maths.

## Browser support

Chrome or Edge 89+. Web Serial does not exist in Firefox or Safari, and it
requires `https://` or `localhost`.

## Running it

```bash
npm install
npm start        # http://localhost:3000
npm test         # 83 tests
npm run build
```

## What it does

**Connect.** Press *Connect to Robot* and pick the Teensy port. If the cable is
pulled the app notices — by stream error, by the browser's disconnect event, or by
a watchdog when reports simply stop — abandons any running path, and reopens the
port by itself on a backoff. Reconnecting needs no new port dialog, because the
browser remembers a granted port.

**Jog in joint space.** Sliders per joint, then *Move to Target*.

**Jog in Cartesian space.** Type an XYZ target in millimetres. The input ranges are
computed from the kinematic model, not hardcoded, but they are the outer bounding
box — a point inside them can still be unreachable, and the IK result is what says
so. A residual is reported when it fails.

**Teach and play back paths.** Record waypoints from the arm's current pose,
reorder them, set a feed rate per waypoint, then plan and run. Two interpolation
modes:

- *joint* — a straight line in joint space. Predictable, curved in Cartesian space.
- *linear* — a straight line in Cartesian space, sampled by arc length with a 2 mm
  chord limit, solving IK per sample.

Paths export and import as JSON.

**3D view.** The arm is built from the same kinematic chain the solvers use, so
what is rendered is what FK and IK compute — a test asserts the two agree to
within 1e-9 m. Shows the planned path, the target marker and the workspace bounds.

## Reading the status bar

| | |
|---|---|
| State | From the firmware, not guessed. `MOVING` persists while its queue drains. |
| `POSITION UNVERIFIED` | A hard stop may have lost steps, or the arm has not been homed. Re-home before trusting the angles. |
| Queue | Free slots in the firmware motion queue. Depth is what keeps a streamed path continuous. |
| Homed | Which joints have a datum. Without one, that joint's angle is only relative to wherever it powered up. |
| Endstops | Debounced switch states. |

## Structure

```
src/
├── kinematics/
│   ├── robotModel.ts        Chain and limits. Single source of truth,
│   │                        mirrors firmware/config.h.
│   ├── linalg.ts            Transforms, rotations, Cholesky solver
│   ├── ForwardKinematics.ts FK from the URDF chain + analytic Jacobian
│   ├── InverseKinematics.ts Damped least squares IK, workspace bounds
│   └── kinematics.test.ts
├── motion/
│   ├── VelocityProfile.ts   Trapezoidal profile
│   ├── PathInterpolator.ts  Waypoint pair → intermediate joint poses
│   ├── TrajectoryPlanner.ts Multi-waypoint planning
│   └── motion.test.ts
├── communication/
│   ├── webSerial.ts         Injectable Web Serial interfaces
│   ├── SerialManager.ts     Transport, connection lifecycle, flow control
│   └── serialManager.test.ts
├── viewer3d/                three.js model building and STL loading
├── components/              UI panels
├── store/robotStore.ts      Zustand state and the trajectory sender
└── App.tsx
```

The kinematics and motion layers hold no React and no I/O, which is why they are
straightforward to test.

## Troubleshooting

**"Web Serial is not available in this browser."** Chrome or Edge, over
`localhost` or `https://`.

**The port will not open.** Something else is holding it — the Arduino IDE's
serial monitor is the usual culprit.

**It keeps saying *Reconnecting*.** The controller is not answering. Check the
cable, then that the firmware is actually running; the app polls a granted port
and gives up after six attempts.

**Positions do not update.** The firmware pushes a report every 50 ms. If nothing
arrives for two seconds the app declares the link dead, so a frozen position with
a green indicator should not be possible — if you see it, the browser console will
have the parse errors.

**IK reports a residual.** The target is out of reach from the current pose. The
arm is tightly limited on J1 and J2; see the workspace table in
[docs/HARDWARE.md](../docs/HARDWARE.md).

**A move is refused.** Motors disabled, an emergency stop still latched (clear it
with *Motors Enabled*), or homing in progress.

## Stack

React 19, TypeScript, Zustand, three.js with React Three Fiber, Tailwind CSS,
Web Serial. Kinematics and motion maths are dependency-free.
