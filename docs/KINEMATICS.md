# Kinematics Documentation

**6DOF Robot Arm - Forward and Inverse Kinematics**

---

## Overview

This document describes the kinematics implementation for the 6DOF robot arm, including:
- Forward Kinematics (FK): Joint angles → Cartesian position
- Inverse Kinematics (IK): Cartesian position → Joint angles
- URDF-chain runtime frame model (shared by FK, IK, and 3D viewer)
- Usage examples and troubleshooting

---

## Coordinate Systems

### Runtime Kinematic Frame (Authoritative)
- Runtime FK/IK uses URDF chain transforms from `ROBOT_KINEMATIC_CHAIN`.
- The 3D robot, Cartesian target marker, and `currentPosition` all use this same URDF frame.
- This avoids visual/numeric mismatch between marker position and robot pose.

### Joint Space
- **Representation:** 6 joint angles [J1, J2, J3, J4, J5, J6]
- **Units:** Degrees
- **Range:** Defined by `jointLimits` in URDF

### Cartesian Space
- **Representation:** Position (X, Y, Z) + Orientation (Roll, Pitch, Yaw)
- **Units:** Meters for position, radians for orientation
- **Origin:** Robot base center (linkB)
- **Axes:**
  - X: Forward/backward
  - Y: Left/right
  - Z: Up/down (vertical)

### Logical-to-URDF Angle Mapping

FK/IK/3D do not consume raw logical angles directly; they first apply an effective URDF offset:
- `J2..J5`: `effectiveUrdfOffsetDeg = -homePose.jointsDeg`
- `J1/J6`: `effectiveUrdfOffsetDeg = cfg.joints[j].urdfOffsetDeg`

Direction multipliers are also applied:
- `urdfDir = [1, 1, 1, 1, -1, -1]` (J5/J6 inverted)

Conversions used globally:
- `urdfDeg = logicalDeg * urdfDir + effectiveUrdfOffsetDeg`
- `logicalDeg = (urdfDeg - effectiveUrdfOffsetDeg) / urdfDir`

### Why J6 Does Not Affect XYZ
- With the current tool center point (TCP), J6 rotates around the tool axis.
- That rotation changes orientation, but not end-effector position in Cartesian XYZ.
- For position-only IK, J6 is underconstrained and may remain unchanged.
- For pose-locked IK, J6 is actively used to maintain orientation quality.

### Cartesian Policy
- Cartesian control exposes two explicit modes:
  - `pose_lock` (default): hold current tool orientation while moving in XYZ.
  - `position_only`: prioritize XYZ only and allow orientation drift.
- Direct Cartesian moves are generated as linear trajectories with quintic minimum-jerk timing.
- IK is solved sample-by-sample (warm-started), and each sample carries joint velocity for firmware interpolation.
- Default quality gates:
  - position residual target: <= 1.5 mm
  - orientation residual target (pose lock): <= 1.5 deg
  - straight-line tracking target: <= 2 mm max deviation
- If the selected mode cannot satisfy quality gates, move is rejected with diagnostics.

### IK Runtime Hierarchy (V3)
- Runtime solver order:
  1. Analytical-first branch generation (`AnalyticalPieperIK`)
  2. Continuity/branch-lock ranking (`ContinuityPolicy`)
  3. Numerical constrained refinement (`InverseKinematics`)
  4. Numeric-only fallback when analytical candidates are not valid
- Branch labels are deterministic:
  - `SL|SR` = shoulder side
  - `EU|ED` = elbow branch
  - `WF|WN` = wrist flip / non-flip
- Branch locking is enabled by default for trajectory continuity and avoids branch jumps unless needed.
- Singularity handling includes:
  - spectrum-based damping scaling
  - wrist singular bypass near `sin(J5) -> 0`
  - explicit singularity flags in `IKResult`.
- Collision handling:
  - optional capsule-based self-collision checks are available in the IK loop.
  - when enabled, colliding solutions are rejected with `failureCategory='collision'`.
- Boundary handling:
  - reachability probe now returns `boundaryDistanceM`.
  - damping and step aggressiveness are reduced near workspace limits.

### Quaternion-First Orientation Internals
- Internal IK error metrics use quaternion log-map error.
- Orientation drift metrics are quaternion-based (geodesic angle), not Euler subtraction.
- Euler angles remain supported as UI I/O representation for compatibility.

### Workspace Classification Policy (Strict)
- A target is not classified as `invalid_target` from a single failed attempt.
- Runtime retries in stages:
  - Stage A: base solver parameters
  - Stage B: relaxed damping/posture parameters
  - Stage C: nearby seeded retries
- `invalid_target` is emitted only when all retries fail and the best residual remains above strict threshold.
- If pose lock fails but position-only succeeds, failure is classified as `orientation_infeasible`.

### Queue-Driven Execution
- Cartesian trajectories are uploaded using `TQ` queue commands.
- Firmware executes the queue internally with Hermite interpolation.
- Manual controls (`J`, `JR`) stay unchanged.
- Queue budgeting is adaptive: planner first tries the target quality floor (25 Hz), then auto-adjusts speed/sampling to stay within firmware point limits before rejecting.
- Cartesian planning is two-stage:
  - Stage 1 (`stage1_fast`): fast endpoint feasibility target (<500 ms responsiveness).
  - Stage 2 (`stage2_refine`): strict background refinement on a worker thread.
- Execution only starts after Stage 2 returns a strict-quality trajectory.
- Runtime uses a normalized constraint contract per request:
  - source limits come from firmware config (logical frame)
  - limits are mapped once into URDF frame (including direction/offset mapping)
  - endpoint IK, interpolation IK, and worker refinement all consume the same URDF-frame limits.
- Planned trajectories are fail-closed on limits:
  - no silent post-plan clamping before queue upload
  - any out-of-limit sample is rejected as deterministic planning failure (`joint_limit`).
- Queue execution is transactional:
  - clear -> upload -> verify queue count -> run -> status query.
- Motion quality diagnostics are exposed by `MQ STAT` (`tick_jitter_us`, `queue_underrun`, `step_overrun`).
- Operator feature flags:
  - `REACT_APP_IK_ENGINE_V2` (default enabled)
  - `REACT_APP_REACHABILITY_ATLAS_V1` (default enabled)
  - `REACT_APP_MOTION_KERNEL_V2` (default enabled)
  - `REACT_APP_IK_ANALYTIC_PRIMARY_V1` (default enabled)
  - `REACT_APP_IK_RESOLVED_RATE_V1` (default enabled)
  - `REACT_APP_IK_COLLISION_CHECK_V1` (default disabled; set `1` to enable)

---

## Legacy DH Parameters (Reference Only)

The table below is historical reference from earlier implementation work.
Runtime FK/IK now uses the URDF chain directly.

The robot uses **Modified DH Convention (Craig)**:

| Joint | α (deg) | a (mm) | d (mm) | θ | Notes |
|-------|---------|--------|--------|---|-------|
| J1    | 0       | 0      | 80.0   | q₁ | Base rotation |
| J2    | -90     | 42.5   | 55.95  | q₂ | Shoulder pitch |
| J3    | -180    | 160.0  | 16.0   | q₃ | Elbow pitch |
| J4    | -90     | 38.1   | 36.4   | q₄+90° | Wrist roll |
| J5    | -90     | 10.02  | 103.23 | q₅+90° | Wrist pitch |
| J6    | 90      | 26.77  | 9.94   | q₆-90° | Tool rotation |

**Parameters:**
- **α (alpha):** Twist angle about X_{i-1}
- **a:** Link length along X_{i-1}
- **d:** Link offset along Z_i
- **θ (theta):** Joint angle about Z_i (variable)

---

## Forward Kinematics

### Algorithm

1. Convert joint angles from degrees to radians.
2. For each URDF joint, build:
   - origin transform from `xyz` and `rpy`
   - revolute transform around the joint axis (`axis` from URDF)
3. Multiply along the chain:
   - `T_base_to_ee = (T_origin_1 * R_axis_1(q1)) * ... * (T_origin_6 * R_axis_6(q6))`
4. Extract position from translation (`x, y, z`).
5. Extract orientation from the final rotation matrix.

### Per-Joint Composition

```text
T_joint_i = T_xyz(x_i, y_i, z_i) * R_rpy(roll_i, pitch_i, yaw_i) * R_axis_i(q_i)
```

### Code Example

```typescript
import { ForwardKinematics } from './kinematics/ForwardKinematics';

// Joint angles in degrees
const jointAngles = [0, 20, 30, 0, 0, 0];

// Solve FK
const result = ForwardKinematics.solve(jointAngles);

if (result.success) {
  const pos = result.endEffectorPose.position;
  console.log(`Position: X=${pos.x}m, Y=${pos.y}m, Z=${pos.z}m`);

  const rot = result.endEffectorPose.rotation;
  console.log(`Orientation: Roll=${rot.roll}, Pitch=${rot.pitch}, Yaw=${rot.yaw}`);
}
```

### Performance

- **Computation time:** < 1ms
- **Update rate:** Real-time capable (>100 Hz)
- **Dependencies:** Pure JavaScript (no external math libs)

---

## Inverse Kinematics

### Algorithm: Weighted Damped Least Squares (Normalized)

Iterative solver in URDF chain frame:

`
Delta q = (J^T J + (lambda^2 * s + w_posture * s) I)^(-1) * (J^T e + w_posture * s * (q_center - q))
`

Where:
- J = analytic geometric Jacobian (units per degree)
- e = 6D pose error (position + orientation)
- lambda = damping weight
- w_posture = posture regularization weight
- s = normalization scale, s = max(trace(J^T J) / n, eps)

**Iteration Steps:**
1. Compute FK at current joint angles.
2. Compute weighted pose error.
3. Build analytic Jacobian from URDF chain.
4. Solve normalized DLS update.
5. Clamp per-iteration step and joint limits.
6. Accept/reject step with line search and adaptive damping.
7. Repeat until strict residual gates are met or retries are exhausted.

### Configuration

```typescript
const ikConfig = {
  maxIterations: 100,     // Maximum iterations before giving up
  tolerance: 0.001,       // Position error tolerance (meters)
  dampingFactor: 0.01,    // Damping λ (higher = more stable, slower)
  jointLimits: {          // Enforce joint limits
    min: [-2.8, -1.3, -2.1, -2.5, -2.5, -6.28],
    max: [2.8, 1.3, 2.1, 2.5, 2.5, 6.28]
  }
};
```

### Code Example

```typescript
import { InverseKinematics } from './kinematics/InverseKinematics';

const ikSolver = new InverseKinematics();

// Target position in meters
const targetPosition = {
  x: 0.20,  // 200mm forward
  y: 0.10,  // 100mm left
  z: 0.25   // 250mm up
};

// Initial guess (use current position for better convergence)
const initialGuess = [0, 20, 30, 0, 0, 0];

// Solve IK
const result = ikSolver.solvePosition(targetPosition, initialGuess);

if (result.success) {
  console.log('Joint angles:', result.jointAngles);
  console.log('Converged in', result.iterations, 'iterations');
  console.log('Final error:', result.residualError * 1000, 'mm');
} else {
  console.error('IK failed:', result.error);
}
```

### When to Use Position vs Pose IK

**Position-Only IK** (`solvePosition`):
- Faster convergence
- Only cares about XYZ location
- Orientation is free to vary
- **Use for:** Pick-and-place, drawing, general positioning

**Full Pose IK** (`solvePose`):
- Slower convergence
- Controls both position and orientation
- More constraints = harder to solve
- **Use for:** Precise tool alignment, assembly tasks

### Performance

- **Position residual target:** <= 1.5 mm
- **Pose-lock orientation target:** <= 1.5 deg
- **Classification policy:** staged retries before `invalid_target`
- **Cartesian execution:** queue-budgeted sampling with strict final residual gate

---

## Workspace Analysis

### Reachable Workspace

Approximate workspace bounds (from base origin):

| Axis | Minimum | Maximum | Notes |
|------|---------|---------|-------|
| X    | -300mm  | +300mm  | Forward/back |
| Y    | -300mm  | +300mm  | Left/right |
| Z    | 0mm     | 400mm   | Height above base |

**Shape:** Roughly toroidal (donut-shaped) due to J2/J3 reach

### Singularities

Configurations where IK may fail or behave poorly:

1. **Shoulder singularity:** J2 and J3 aligned (fully extended or retracted)
2. **Elbow singularity:** J3 = 0° (straight arm)
3. **Wrist singularity:** J5 = 0° (wrist axes aligned)

**Avoidance:**
- Stay away from fully extended configurations
- Use non-zero J5 angles when possible
- Increase damping factor near singularities

---

## Testing and Validation

### Round-Trip Test

Verifies FK and IK are consistent:

```typescript
import { runTestsInConsole } from './kinematics/testKinematics';

// Run in browser console
runTestsInConsole();
```

**Expected Results:**
- Position error: < 1mm
- Joint error: < 5°
- Success rate: > 80%

### Test Cases

The test suite includes:
1. Home position (zeros)
2. Single joint movements
3. Multi-joint combinations
4. Edge cases (near limits)
5. Maximum reach positions

### Manual Validation

1. Move robot to known position (e.g., home)
2. Read XYZ from StatusBar
3. Enter same XYZ in Cartesian control
4. Click `Current -> Target` and verify marker overlaps end-effector in 3D
5. Click "Move to Position"
6. Robot should stay in same position (minimal movement)

---

## Usage in Web Application

### Automatic FK Updates

Forward kinematics runs automatically whenever joint angles change:

```typescript
// In robotStore.ts
manager.onMessage((msg) => {
  if (msg.type === 'POS') {
    set({ currentAngles: msg.data });
    get().updateCurrentPosition(); // ← Automatic FK
  }
});
```

Result displayed in StatusBar:
```
Position: X:123.4mm Y:56.7mm Z:234.5mm
```

### Cartesian Control Panel

Located in left panel above joint controls:

**Features:**
- Current XYZ display (real-time from FK)
- Target XYZ input fields (mm)
- Workspace limit indicators
- "Move to Position" button (triggers IK)
- IK status feedback (success/failure, iterations, error)
- Target marker is frame-gated by loaded firmware config and snaps to current pose after motion settles.

**Preconditions (required):**
- Connected to robot
- Motors enabled
- J2..J5 homed

**Workflow:**
1. Connect to robot and enable motors
2. Home J2..J5 (use `H ALL` for normal flow)
3. Wait until Cartesian frame is synchronized (or click `Current -> Target`)
4. Enter target X, Y, Z coordinates (mm)
5. Click "Move to Position"
6. IK solver computes joint angles
7. If successful, robot moves to target
8. If failed, error message displayed

---

## Coordinate Frame Visualization

```
         Z↑
          |
          |
    ┌─────┴─────┐
    │   Base    │
    │  (linkB)  │
    └───────────┘
         / \
        /   \
       /     \
Y ←───┘       └───→ X
```

**Base Frame (World):**
- Origin: Center of base plate
- Z-axis: Vertical (up)
- X-axis: Forward (front of robot)
- Y-axis: Left (follows right-hand rule)

---

## Troubleshooting

### IK Not Converging

**Problem:** "Failed to converge" error

**Solutions:**
1. Verify frame consistency first (`Current -> Target` overlap in 3D).
2. Retry in `Position Only` mode:
   - if it succeeds, failure is orientation constraint (`orientation_infeasible`).
3. Check failure category:
   - `joint_limit` or `singularity`: move slightly away and retry.
   - `invalid_target`: all staged retries failed above strict residual gate.
4. If queue budget error appears, move closer or reduce Cartesian speed.

### IK Converges to Wrong Solution

**Problem:** Robot takes unexpected path

**Cause:** Multiple IK solutions exist (robot is redundant for position-only)

**Solutions:**
1. Use current position as initial guess (stays close)
2. Add orientation constraints (use `solvePose` instead)
3. Manually specify preferred configuration

### Position Accuracy Low

**Problem:** FK position doesn't match real robot

**Causes:**
- URDF chain constants or mapping incorrect
- Stepper calibration wrong (`USTEPS_PER_DEG`)
- Mechanical play in joints

**Solutions:**
1. Verify URDF chain constants and logical-to-URDF mapping
2. Re-calibrate stepper ratios in `config.h`
3. Re-home robot to reset zero positions
4. Check for mechanical issues

### Workspace Limits Too Restrictive

**Problem:** Reachable positions rejected

**Solution:** Adjust limits in `CartesianControlPanel.tsx`:

```typescript
const WORKSPACE_LIMITS = {
  x: { min: -0.35, max: 0.35 },  // Expand from ±300mm to ±350mm
  y: { min: -0.35, max: 0.35 },
  z: { min: 0.0, max: 0.45 }     // Expand from 400mm to 450mm
};
```

---

## Advanced Topics

### Jacobian Matrix

The Jacobian maps joint velocities to end-effector velocities:

```
v = J · q̇
```

Where:
- `v` = [vₓ, vᵧ, vᵤ, ωₓ, ωᵧ, ωᵤ]ᵀ (twist vector)
- `J` = 6×6 Jacobian matrix
- `q̇` = [q̇₁, q̇₂, q̇₃, q̇₄, q̇₅, q̇₆]ᵀ (joint velocities)

**Computed numerically:**
```typescript
const J = ForwardKinematics.computeJacobian(jointAngles);
```

**Uses:**
- Inverse kinematics (DLS method)
- Velocity control
- Singularity detection (det(J) ≈ 0)
- Manipulability analysis

### Singularity Detection

```typescript
function isSingular(J: number[][]): boolean {
  const det = computeDeterminant(J);
  return Math.abs(det) < 1e-3;
}
```

If singular, increase damping or avoid configuration.

### Alternative IK Methods

Current implementation uses **Damped Least Squares (DLS)**.

**Other options:**
- **Analytical IK:** Faster but complex (requires geometry analysis)
- **CCD (Cyclic Coordinate Descent):** Simpler but less accurate
- **Jacobian Transpose:** Faster but slower convergence
- **Optimization-based:** Global minimum but very slow

DLS chosen for balance of speed, accuracy, and robustness.

---

## References

### Books
- Craig, J. J. (2005). *Introduction to Robotics: Mechanics and Control* (3rd ed.)
- Siciliano, B., & Khatib, O. (2016). *Springer Handbook of Robotics* (2nd ed.)
- Spong, M. W., Hutchinson, S., & Vidyasagar, M. (2006). *Robot Modeling and Control*

### Papers
- Nakamura, Y., & Hanafusa, H. (1986). "Inverse Kinematic Solutions With Singularity Robustness for Robot Manipulator Control"
- Buss, S. R. (2004). "Introduction to Inverse Kinematics with Jacobian Transpose, Pseudoinverse and Damped Least Squares methods"

### Online Resources
- [Modern Robotics (Northwestern)](http://modernrobotics.org/)
- [ROS MoveIt! Kinematics](http://docs.ros.org/en/kinetic/api/moveit_tutorials/)

---

## Appendix: URDF Kinematic Chain

```xml
linkB.000 (fixed world frame)
  ↓ (fixed joint)
linkB (base plate)
  ↓ (J1: link0_joint, revolute, Z-axis)
link0 (base wall)
  ↓ (J2: Joint2, revolute, Z-axis)
link1 (arm 1)
  ↓ (J3: Joint3, revolute, Z-axis)
link2 (arm 2 mount)
  ↓ (J4: link3_joint, revolute, Z-axis)
link3 (rotation arm)
  ↓ (J5: Joint5, revolute, Z-axis)
link4 (J6 housing)
  ↓ (J6: Joint6, continuous, Z-axis)
link5 (end-effector)
```

**Joint Limits (from URDF):**
- J1: ±2.8 rad (±160°)
- J2: ±1.3 rad (±74°)
- J3: ±2.1 rad (±120°)
- J4: ±2.5 rad (±143°)
- J5: ±2.5 rad (±143°)
- J6: Continuous (no limits)

---

**For further assistance, refer to:**
- [PHASE_2_COMPLETE.md](../PHASE_2_COMPLETE.md) - Implementation details
- [URDF.md](../URDF.md) - Robot structure definition
- [testKinematics.ts](../robot-arm-control/src/kinematics/testKinematics.ts) - Validation tests

