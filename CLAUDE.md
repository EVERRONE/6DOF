# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

This file is the source-grounded operating guide for working in this repository.
It was audited against code on 2026-03-25.

## Audit Status

- Web tests verified: `npm test -- --watchAll=false --runInBand`
  - Result: 24 suites passed, 102 tests passed (as of 2026-08-08)
- Web production build verified: `npm run build`
  - Result: succeeds with warnings
  - Current warning: missing source map in transitive dependency `@mediapipe/tasks-vision`
  - Current output: large CRA bundle, main JS about 1.13 MB gzip
- Firmware build tooling in this environment:
  - `arduino-cli`: not installed
  - `platformio` / `pio`: not installed
  - Repo does not contain PlatformIO config
  - Practical firmware upload path is Arduino IDE + Teensyduino with `firmware/firmware.ino`

## Project Summary

This is a 6-DoF DIY robot arm project split into two main halves:

- `firmware/`
  - Teensy 4.1 firmware in C++
  - Real-time step generation
  - Endstop homing
  - Serial ASCII command protocol
  - Queue-based trajectory playback with Hermite interpolation
- `robot-arm-control/`
  - React + TypeScript web app built with CRA
  - Web Serial communication
  - 3D visualization
  - FK / IK
  - Cartesian planning with stage-1 and stage-2 refinement
  - Calibration and home-pose UI

There is also a root `stl_meshes/` folder with mesh source assets and a mirrored copy in `robot-arm-control/public/stl_meshes/` that the browser app actually serves.

## Recommended First Reads

Read these files first when orienting or debugging:

- `firmware/config.h`
- `firmware/firmware.ino`
- `firmware/SerialProtocol.cpp`
- `firmware/StepperController.cpp`
- `firmware/HomingController.cpp`
- `firmware/TrajectoryExecutor.cpp`
- `robot-arm-control/src/store/robotStore.ts`
- `robot-arm-control/src/communication/SerialManager.ts`
- `robot-arm-control/src/kinematics/angleMapping.ts`
- `robot-arm-control/src/kinematics/ForwardKinematics.ts`
- `robot-arm-control/src/kinematics/URDFParser.ts`
- `robot-arm-control/src/kinematics/HybridIKSolver.ts`
- `robot-arm-control/src/services/cartesian/ConstraintAdapterService.ts`
- `robot-arm-control/src/services/cartesian/TrajectoryExecutionService.ts`
- `docs/SERIAL_PROTOCOL.md`
- `docs/KINEMATICS.md`
- `docs/CALIBRATION.md`

## Top-Level Layout

- `firmware/`
  - Teensy sketch and all firmware subsystems
- `robot-arm-control/`
  - React app, tests, worker, scripts
- `docs/`
  - Protocol, kinematics, calibration, hardware audit, IK reports
- `robot-agent-bridge/`
  - Standalone Node broker that lets an external AI agent drive the arm
    through the web app. Additive: nothing else depends on it.
- `stl_meshes/`
  - Root mesh sources
- `IMPLEMENTATION_PLAN.md`, `PHASE_*_COMPLETE.md`
  - Historical planning / phase notes, not source of truth

## Source Of Truth Rules

These rules matter more than any high-level doc.

### Working approach

- Available skills may be used whenever they materially help with the task.
- Subagents may be used freely for bounded exploration, parallel analysis, implementation, or verification when that makes the work more efficient.

### Hardware, limits, and motion behavior

Authoritative files:

- `firmware/config.h`
- `firmware/StepperController.cpp`
- `firmware/HomingController.cpp`
- `firmware/TrajectoryExecutor.cpp`
- `firmware/SerialProtocol.cpp`

If a README or design note disagrees with those files, the code wins.

### Runtime kinematics

Authoritative files:

- `robot-arm-control/src/kinematics/URDFParser.ts`
- `robot-arm-control/src/kinematics/UrdfChainKinematics.ts`
- `robot-arm-control/src/kinematics/ForwardKinematics.ts`
- `robot-arm-control/src/kinematics/angleMapping.ts`

Important facts:

- Runtime FK goes through `ForwardKinematics.solve()`, which delegates to `UrdfChainKinematics`.
- **Prefer `ForwardKinematics.solveWithJacobian(q)` inside IK loops** — returns `{ fk, jacobian }` from a single chain traversal via `UrdfChainKinematics.solveWithJointFrames()` + `GeometricJacobian.computeFromFrames()`. Never call `solve()` and `computeJacobian()` separately in a hot loop.
- `AnalyticalPieperIK.refineWristOrientation()` uses the analytic angular sub-block (rows 3-5, cols 3-5 of the geometric Jacobian) — not finite differences. Preserve this if touching wrist refinement.
- The active kinematic chain is the hardcoded `ROBOT_KINEMATIC_CHAIN` in `URDFParser.ts`.
- There is no runtime `robot.urdf` asset in this repo right now. `URDF.md` is documentation, not the live runtime model.
- Never feed logical firmware angles directly into FK, IK, or the 3D viewer. Always pass through `angleMapping.ts`.

### Runtime constraints

The firmware config message (`CFG`) is the runtime source of truth for limits and calibration in the web app.

Flow:

- firmware emits `CFG {json}`
- `SerialManager` parses it
- `robotStore` stores it as `firmwareConfig`
- `ConstraintAdapterService.fromFirmwareConfig()` maps logical firmware limits into URDF-frame limits
- the IK solvers and trajectory planner consume those mapped URDF limits

Do not hardcode limits in UI or planner logic if a runtime `firmwareConfig` is available.

### Effective URDF offsets

The web app does not use raw `urdfOffsetDeg` blindly.

`getEffectiveUrdfOffsets()` applies these rules:

- J1 and J6 use `cfg.joints[i].urdfOffsetDeg`
- J2..J5 use `-homePose.jointsDeg` after direction adjustment when home-pose data exists
- effective URDF direction multipliers are currently `[1, 1, 1, 1, -1, -1]`

Meaning:

- `HP` settings shift the kinematic frame used by FK / IK / 3D for J2..J5
- `CAL` changes raw-to-logical mapping
- `HP` changes operational post-home target and the web-side URDF anchor
- J5 and J6 are inverted in URDF space relative to logical firmware angles

## Verified Dev Commands

Run these from `robot-arm-control/`:

```bash
npm start
npm test -- --watchAll=false --runInBand
npm run build
npm run atlas:generate
npm run model:fit -- <samples.json> [output.json]
```

Notes:

- This is CRA, not Vite. Use `npm start`, not `npm run dev`.
- Web Serial requires Chrome or Edge. Firefox and Safari are not usable for live hardware control here.
- The repo root path contains spaces, so prefer setting the working directory explicitly before running commands.

## Firmware Architecture

### Main loop

`firmware/firmware.ino` is intentionally small:

1. `protocol.update()`
2. `stepper.update()`
3. stop if a moving joint is driving into a triggered endstop
4. `delayMicroseconds(10)`

### StepperController

`StepperController` is the low-level motion layer.

Responsibilities:

- owns current steps and target steps
- converts logical degrees <-> raw degrees using calibration
- runs timer-driven stepping with `IntervalTimer`
- handles speed slewing for streaming targets
- clamps manual targets to firmware joint limits
- persists calibration to EEPROM

Key constants from `config.h`:

| Joint | Min | Max | Endstop | Home Toward Min | Invert Dir |
| --- | ---: | ---: | --- | --- | --- |
| J1 | -60 | 60 | no | n/a | false |
| J2 | -120 | 0 | yes | false | true |
| J3 | 0 | 120 | yes | true | true |
| J4 | -355 | 0 | yes | false | false |
| J5 | -355 | 0 | yes | false | false |
| J6 | -360 | 360 | no | n/a | false |

Other important motion constants:

- step timer tick: 10 us
- trajectory update target: 1 kHz
- queue max points: 256
- stream speed clamp: 5..120 deg/s

### HomingController

`HomingController` performs blocking homing with active-low debounced endstops.

Important current behavior:

- active homing path is "move to switch -> set logical zero -> stay on switch"
- helper methods `backOff()` and `fineApproach()` exist, but are not used in the active homing flow
- `homeAll()` sequence is J2 -> J3 -> J4 -> J5
- `H ALL` may be followed by operational home-pose moves if HP is enabled

### TrajectoryExecutor

`TrajectoryExecutor` is the queue runner for `TQ` commands.

It:

- stores up to 256 queue points
- requires strictly increasing `tMs`
- validates point finiteness
- rejects out-of-range joint samples
- rejects very high velocities
- evaluates cubic Hermite interpolation between queue points
- drives `StepperController.setStreamingCommand()` at 1 kHz
- Hermite elapsed time is tracked with **microsecond precision** (`startUs_ = micros()` at `TQ RUN`, `elapsedMsFloat = elapsedUs / 1000.0f` per tick). Do not revert to `millis()` — the 1 ms quantization of `millis()` caused systematic velocity stuttering (every other tick evaluated the same Hermite point twice or skipped one).
- after the last knot is passed, enters a **deceleration continuation phase**: keeps calling `setStreamingCommand(finalPos, zeroVel)` at every tick until `isMoving()` returns false or `TRAJECTORY_DECELERATION_TICKS` (400 ms) elapses — only then emits `TQ DONE`. This prevents the abrupt stop that was the primary source of end-of-move jerk.
- tracks diagnostics:
  - tick jitter
  - queue underrun count
  - step overrun count

### SerialProtocol

`SerialProtocol` is the firmware command parser and status emitter.

Notable behavior:

- position and endstop state broadcast every 100 ms
- motion-kernel status (`MQ STAT`) broadcasts every 500 ms, not just on explicit `MQ?`
- `CFG?`, `HP?`, `TQ?`, `MQ?` are special-case top-level queries
- queue commands emit both high-level status lines and optional `ACK` messages
- home pose and calibration persist in EEPROM

## Serial Protocol Summary

Main host-to-robot commands:

- `J <j1..j6> <speed>`
- `JR J<n> <delta> <speed>`
- `H <joints>` / `H ALL`
- `E <0|1>`
- `S`
- `Q`
- `CFG?`
- `CAL SET`, `CAL ZERO`, `CAL SAVE`, `CAL LOAD`, `CAL RESET`
- `HP?`, `HP EN`, `HP SET`, `HP SETALL`, `HP SPD`, `HP SAVE`, `HP LOAD`, `HP RESET`
- `TQ CLEAR`, `TQ PT`, `TQ RUN`, `TQ STOP`, `TQ?`
- `MQ?`, `MQ RESET`

Main robot-to-host messages:

- `POS`
- `ENDSTOP`
- `HOMED`
- `CFG`
- `HP`
- `HOMEPOSE_REACHED`
- `TQ READY`
- `TQ STAT`
- `TQ PROG`
- `TQ DONE`
- `TQ ERR`
- `MQ STAT`
- `ACK`
- `OK`
- `ERROR`

Important command semantics:

- Manual `J` / `JR` targets are clamped by firmware.
- Queue points (`TQ PT`) are validated and rejected if out of range.
- Queue execution is meant to be transactional:
  - `TQ CLEAR`
  - `TQ PT` x N
  - `TQ?` to verify count
  - `TQ RUN`

## Web App Architecture

### App shell

`App.tsx` composes the UI as:

- `ConnectionPanel`
- `PathPlannerPanel`
- `CartesianControlPanel`
- `CalibrationPanel`
- `JointControlPanel`
- `RobotViewer3D`
- `StatusBar`
- `EmergencyStop`

### Central state

`robot-arm-control/src/store/robotStore.ts` is the main application brain.

It owns:

- connection state
- latest firmware config
- robot state and positions
- homing state
- IK settings and diagnostics
- Cartesian planning state
- waypoint and path state
- execution progress
- serial event handling

Important implementation detail:

- `trajectoryPlanner`, `hybridEndpointSolver`, and `legacyEndpointSolver` are module-level singletons in `robotStore.ts`
- they are updated from incoming `CFG` / `HP` data
- disconnect clears store state, but does not recreate those singleton instances
- in reconnect-heavy flows and tests, do not assume a fresh planner / solver instance unless you explicitly rebuild or re-seed it

On connect it:

1. opens Web Serial
2. registers a message handler
3. queries `CFG?`
4. queries `TQ?`
5. queries `Q`

### SerialManager

`SerialManager.ts` is the ASCII line protocol wrapper.

Important behavior:

- commands are encoded as ASCII only
- `sendAndAwait()` supports timeout, retry, and predicate matching
- queue transactions accept either `ACK` or the expected status message
- `ERROR TQ ...` is normalized into `TQ_ERR`
- parser knows about `CFG`, `HP`, `TQ_*`, `MQ_STAT`, and `ACK`

If you change firmware message formats, you must update:

- `firmware/SerialProtocol.cpp`
- `robot-arm-control/src/communication/SerialManager.ts`
- `robot-arm-control/src/communication/types.ts`
- `robot-arm-control/src/store/robotStore.ts`
- `docs/SERIAL_PROTOCOL.md`
- related tests

### 3D viewer

The viewer uses:

- `RobotViewer3D.tsx`
- `viewer3d/RobotModel3D.ts`
- `viewer3d/STLLoader.ts`

Important facts:

- the 3D hierarchy is built from `ROBOT_KINEMATIC_CHAIN`
- logical angles are mapped into URDF space before pose updates
- Euler order is explicitly `'ZYX'` to match URDF fixed-axis XYZ interpretation
- root `stl_meshes/` and `public/stl_meshes/` can drift if only one side is updated
- the workspace box shown in UI / 3D is advisory only, not the final motion authority

## Kinematics And Planning

### FK / IK stack

Key files:

- `ForwardKinematics.ts`
- `UrdfChainKinematics.ts`
- `QuaternionMath.ts`
- `HybridIKSolver.ts`
- `InverseKinematics.ts`
- `AnalyticalPieperIK.ts`
- `ContinuityPolicy.ts`
- `GeometricJacobian.ts`
- `SingularityHandler.ts`
- `CollisionModel.ts`

Current solver strategy:

1. analytical candidate generation
2. branch ranking / continuity policy
3. numerical refinement
4. numeric fallback if needed

Feature flags are read from env in `robotStore.ts`:

- `REACT_APP_IK_ENGINE_V2`
- `REACT_APP_REACHABILITY_ATLAS_V1`
- `REACT_APP_MOTION_KERNEL_V2`
- `REACT_APP_IK_ANALYTIC_PRIMARY_V1`
- `REACT_APP_IK_RESOLVED_RATE_V1`
- `REACT_APP_IK_COLLISION_CHECK_V1`

Defaults in code:

- most flags enabled
- collision checking disabled unless explicitly set to `1`

### PathInterpolator motion quality notes

- `MAX_JOINT_VEL_DEG_S` is **55 deg/s**. Firmware cubic Hermite overshoots knot velocities by ~1.5×, so 55 × 1.5 = 82.5 deg/s — well below the 120 deg/s motor clamp. The old value of 80 left zero headroom and caused audible crackling at the limit.
- `smoothJointAngles()` (3-tap weighted average: 0.25/0.5/0.25) is applied **3 times** (3 passes) before velocity computation. IK samples converge within tolerance but can have ~0.1–0.3° noise; a single pass was insufficient to prevent Hermite from amplifying that noise into velocity oscillation. Three passes attenuate it more aggressively while preserving endpoints.
- `STREAM_SPEED_ACCEL_DEG_S2` in `firmware/config.h` is **300 deg/s²**. In streaming mode (trajectory playback) this constant is **not used** — `applyMotionCommand` bypasses slewing entirely for `STEPPER_MOTION_STREAMING` and sets `appliedSpeedDegS = requestedSpeedDegS` directly. The slew constant only applies to PTP (`J` / `JR` commands). Do not re-introduce slewing in streaming mode; the Hermite polynomial already provides smooth velocity transitions.
- In streaming mode, `StepperController` uses **velocity-based step intervals** (`streamingActive_` flag + `motionProfile_.streamingVelocity`). Step intervals are computed from per-joint Hermite tangent velocity rather than from remaining integer step distance. This eliminates integer quantization aliasing: without it, joints with <1 step/ms (e.g. J5 at 30°/s = 0.533 steps/ms) alternate between delta=0 and delta=1 every tick, causing the ISR to fire irregular burst/skip patterns at ~111 Hz — audible roughness. The ISR also maintains its countdown when delta=0 and `streamingActive_=true` (joint momentarily between integer positions). Do not revert to distance-based intervals for streaming mode.
- Upload velocity sanitizer in `TrajectoryExecutionService` is **85 / 60 deg/s** (first attempt / retry). Previously 220 / 120 — those values allowed velocities that the PathInterpolator's 55 deg/s knot cap never produces, but closed no safety gap.
- Default `tolerancePositionM` is 0.0025 m (tracking_local). Stage1 relaxed retry uses 0.003 m.
- Orientation along Cartesian paths is SLERP'd from start to target using `QuaternionMath.slerp()` at the minimum-jerk progress `s(u)`. `refineTerminalSample()` intentionally uses the full `lockedOrientation` (not SLERP'd). Do not change this asymmetry.

### Reachability atlas

The atlas lives in:

- `src/kinematics/reachabilityAtlas.generated.json`
- generated by `scripts/generateReachabilityAtlas.ts`

The planner uses it as a coarse pre-check and optional seed source, not as the final authority.
Final acceptance still depends on IK, strict trajectory quality gates, queue budget, and firmware limits.

### Direct Cartesian execution path

The direct Cartesian path in `robotStore.moveToPosition()` is the newer queue-based path.

High-level flow:

1. validate connection / motors / homing / frame sync
2. normalize firmware constraints into URDF limits
3. solve endpoint feasibility
4. generate coarse stage-1 Cartesian path
5. run stricter stage-2 refinement, usually in a worker
6. validate all generated points against firmware constraints
7. upload via `TQ`
8. run queue and monitor progress

Important protections:

- fail-closed joint-limit validation before upload
- queue point budget derived from firmware capability
- stage-2 timeout logic with adaptive budget
- fallback to a strict-valid stage-1 path when stage-2 times out or misses the final strict gate
- queue upload retries once with a lower velocity clamp if firmware rejects velocity range

### Stage1 fallback chain (in `robotStore.moveToPosition`)

Stage1 runs up to three attempts before failing:

1. **22 Hz base** — `orientationWeight: 0.20`, `tolerancePositionM: 0.0025`
2. **~16 Hz relaxed** — same weights but slower sampling (75% of base) and looser tolerances (`tolerancePositionM: 0.003`, `orientationWeight: 0.30`)
3. **Soft pose-lock** — `orientationWeight: 0.05`, `tolerancePositionM: 0.003`, `toleranceOrientationRad: 0.10`; only attempted when the position-only probe passes but pose-lock fails; Stage2 tightens orientation back to 0.026 rad

Between attempts 1 and 2, a **position-only probe** (`orientationWeight: 0.0`, `tolerancePositionM: 0.006`) determines whether the target position is intrinsically unreachable (`joint_limit`) or just orientation-infeasible.

**Critical IK empiric:** For near-limit poses, HIGHER `orientationWeight` in the 9 Hz fallback (0.30) improves convergence by guiding the solver into the correct basin of attraction. Reducing it to 0.20 in the fallback is a regression — do not lower it.

**Position-only probe must use `endpointPreferredBranch`** — passing `preferredBranch: undefined` causes the probe to explore a different arm configuration and falsely classify reachable targets as `joint_limit`.

### Waypoint / path planner execution path

This is a major current architectural caveat.

`PathPlannerPanel` + `TrajectoryPlanner` still plan waypoint trajectories, but `robotStore.executeTrajectory()` executes them by sending many manual `J` commands point-by-point.

That means:

- direct Cartesian moves use `TQ`
- saved / waypoint path execution still uses legacy point streaming
- pause / resume logic only applies to that legacy execution loop
- waypoint execution does not currently get the same queue-transaction guarantees as direct Cartesian motion
- per-point planned velocities are not actually used during `executeTrajectory()`
- host wait timing is capped, so slow segments are not reproduced with device-level fidelity

If you are improving motion quality or execution robustness, `executeTrajectory()` is one of the first places to revisit.

### Waypoint semantics

Waypoint data is less rich than the direct Cartesian planner pipeline.

Important facts:

- taught waypoints store Cartesian position plus a joint snapshot
- taught waypoints do not store orientation by default
- in linear interpolation mode, missing waypoint orientation is effectively taken from the segment-start pose
- exported path files store waypoints, planner config, and a coarse `kinematicsFrame` tag
- exported path files do not snapshot firmware config, calibration, or home-pose anchors

Practical implication:

- replaying an old saved path after changing firmware limits, calibration, or `HP` can silently change what that path means
- a "taught" waypoint is not a fully taught tool pose unless orientation was provided explicitly

## Agent Bridge (add-on)

`robot-agent-bridge/` is a standalone Node broker that lets an external AI agent
drive the arm. Design rationale: `docs/AGENT_CONTROL_API.md`.

Facts that matter when working on it:

- The broker runs beside the **agent**, not beside the robot. The browser dials
  *out* over WebSocket, so the machine with the serial port needs no inbound
  port and no stable address.
- The browser executor (`src/agent/AgentBridgeClient.ts`) calls existing store
  actions. It does no kinematics, no serial, no trajectory work.
- **The bridge is opt-in and off by default.** With it disabled, or the broker
  not running, the app behaves exactly as it did before. Preserve this.
- Arming is a **time-boxed window** owned by the executor (the browser), because
  that is where the human at the machine is. The broker only mirrors it.
- Commands: `get_status`, `preview_move` (dry run), `move_relative`, `move_to`,
  `stop`. Motion requires arming; `stop` never does.
- **Move outcomes are observed from the store, never awaited.**
  `moveToPosition` resolves at `TQ RUN` rather than on arrival, and it never
  throws — failures land in `planningState: 'failed'`. `watchMoveOutcome()`
  subscribes before commanding and settles on `TQ_DONE`, `'failed'`, an
  emergency stop, or a timeout that reports "unknown". Do not replace this with
  an `await`.
- Motion was verified by unit tests only; it has not yet run against hardware.

Only one existing file is touched by the whole feature: `App.tsx` mounts the
panel. Keep it that way.

## Calibration And Home Pose

Two distinct systems exist. Do not mix them up.

### Calibration (`CAL`)

Purpose:

- adjust raw-degrees to logical-degrees mapping

Formula in firmware:

```text
raw_deg = steps / stepsPerDeg
logical_deg = raw_deg * scale + offset
raw_deg = (logical_deg - offset) / scale
```

Stored in EEPROM through `CAL SAVE`.

### Home pose (`HP`)

Purpose:

- define where the arm moves after `H ALL`
- provide the web app's URDF anchor for J2..J5

Facts:

- only J2..J5 are writable through `HP SET` / `HP SETALL`
- speed is clamped to 5..40 deg/s
- `HOMEPOSE_APPLY_AFTER_HALL` is enabled in code
- HP persists separately from calibration

Practical implication:

- `CAL` changes logical angle accuracy
- `HP` changes the post-home working pose and the web-side FK / IK anchor for J2..J5

## Current Safety-Critical Behaviors

- firmware starts with motors disabled
- emergency stop halts both queue execution and stepper motion
- homing polls serial for stop requests
- loop-level endstop guard stops only when a joint is moving into an already-triggered endstop
- manual motion clamps to firmware joint limits
- queue motion rejects invalid points instead of clamping them

Live hardware assumptions:

- Teensy 4.1
- two enable pins: 8 and 9
- endstops on J2..J5 only
- endstops active low with `INPUT_PULLUP`
- Web Serial browser support required for the UI

## Known Stale Or Misleading Docs

These files are useful, but not fully current:

- `robot-arm-control/README.md`
  - still describes a much smaller phase-1 app
  - still says React 18 / TypeScript 5
  - actual `package.json` is React 19 / TypeScript 4.9
- `firmware/README.md`
  - still says single enable pin 8
  - actual code uses enable pins 8 and 9
When in doubt, trust the code and then update the docs.

## Editing Checklist For Future Work

If you touch motion, protocol, or kinematics, audit the corresponding cross-cutting files.

### If you change firmware limits, calibration, or home-pose semantics

Update:

- `firmware/config.h`
- `firmware/SerialProtocol.cpp`
- `robot-arm-control/src/types/robot.ts`
- `robot-arm-control/src/kinematics/angleMapping.ts`
- `robot-arm-control/src/services/cartesian/ConstraintAdapterService.ts`
- `robot-arm-control/src/store/robotStore.ts`
- `docs/CALIBRATION.md`
- `docs/SERIAL_PROTOCOL.md`

### If you change serial messages

Update:

- firmware parser / emitter
- `SerialManager.ts`
- `communication/types.ts`
- store message handling
- protocol docs
- protocol tests

### If you change the kinematic chain or angle conventions

Update:

- `URDFParser.ts`
- `UrdfChainKinematics.ts`
- `ForwardKinematics.ts`
- `angleMapping.ts`
- `RobotViewer3D.tsx`
- `RobotModel3D.ts`
- IK tests and frame-consistency tests

### If you upgrade waypoint execution to queue-based execution

Audit:

- `robotStore.executeTrajectory()`
- `TrajectoryExecutionService.ts`
- `PathPlannerPanel.tsx`
- progress / pause / cancel semantics
- queue diagnostics and UI feedback

## Bottom Line

The most important architectural facts in this repo are:

- firmware config is the runtime source of truth for limits and calibration
- logical-to-URDF mapping is mandatory before FK / IK / 3D
- the runtime kinematic model is the hardcoded chain in `URDFParser.ts`
- direct Cartesian moves are queue-based and heavily validated
- waypoint/path execution is still on the older `J`-streaming path
- some high-level docs are stale enough that code should be checked first
