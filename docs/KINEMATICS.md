# Kinematics Documentation

**6DOF Robot Arm - Forward and Inverse Kinematics**

---

## Overview

This document describes the kinematics implementation for the 6DOF robot arm, including:
- Forward Kinematics (FK): Joint angles → Cartesian position
- Inverse Kinematics (IK): Cartesian position → Joint angles
- DH parameter derivation from URDF
- Usage examples and troubleshooting

---

## Coordinate Systems

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

---

## DH Parameters

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

1. Convert joint angles from degrees to radians
2. Create DH transformation matrix for each joint:
   ```
   T_i = Rot_X(α) · Trans_X(a) · Rot_Z(θ) · Trans_Z(d)
   ```
3. Multiply matrices: `T = T₁ · T₂ · T₃ · T₄ · T₅ · T₆`
4. Extract position from translation vector: `[T₁₄, T₂₄, T₃₄]`
5. Extract orientation from rotation matrix: `[T₁₁..T₃₃]`

### Transformation Matrix

```
      ┌                                          ┐
      │  cos(θ)    -sin(θ)      0           a    │
T_i = │  sin(θ)cos(α)  cos(θ)cos(α)  -sin(α)  -d·sin(α) │
      │  sin(θ)sin(α)  cos(θ)sin(α)   cos(α)   d·cos(α) │
      │     0          0          0           1    │
      └                                          ┘
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

### Algorithm: Damped Least Squares (DLS)

Iterative numerical method that solves:

```
Δq = (J^T·J + λ²I)^(-1) · J^T · e
```

Where:
- `J` = Jacobian matrix (∂FK/∂q)
- `e` = Position/pose error
- `λ` = Damping factor (singularity avoidance)
- `Δq` = Joint angle update

**Iteration Steps:**
1. Compute FK with current joint angles
2. Calculate error: `e = target - current`
3. Compute Jacobian numerically
4. Solve for `Δq` using damped pseudo-inverse
5. Update: `q ← q + Δq`
6. Check convergence or max iterations
7. Repeat from step 1

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

- **Average iterations:** 15-30
- **Computation time:** 50-200ms
- **Success rate:** ~90% within workspace
- **Convergence tolerance:** 1mm

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
4. Click "Move to Position"
5. Robot should stay in same position (minimal movement)

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

**Workflow:**
1. Enter target X, Y, Z coordinates (mm)
2. Click "Move to Position"
3. IK solver computes joint angles
4. If successful, robot moves to target
5. If failed, error message displayed

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
1. Check if target is within workspace bounds
2. Use better initial guess (current position)
3. Increase `maxIterations` to 200
4. Increase `dampingFactor` to 0.05 (more stable)
5. Relax `tolerance` to 0.002 (2mm)

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
- DH parameters incorrect
- Stepper calibration wrong (`USTEPS_PER_DEG`)
- Mechanical play in joints

**Solutions:**
1. Verify DH parameters match URDF
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
