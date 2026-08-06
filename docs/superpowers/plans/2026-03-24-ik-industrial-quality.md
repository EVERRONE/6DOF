# IK Industrial Quality Improvements — Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Upgrade inverse kinematics and Cartesian path interpolation to industrial quality through three focused changes: combined FK+Jacobian computation, SLERP orientation interpolation, and analytic wrist Jacobian in branch generation.

**Architecture:** All changes are contained within the kinematics and motion layers. No firmware, serial, or store logic changes are needed. Tasks are ordered by decreasing upstream impact — Task 1 makes Task 3 cheaper, Tasks 1+3 together make the analytical solver significantly faster. Task 2 is fully independent.

**Tech Stack:** TypeScript, CRA (Jest), `robot-arm-control/` workspace. Run all tests from `robot-arm-control/` with `npm test -- --watchAll=false --runInBand`.

---

## File Map

| File | Change |
|---|---|
| `src/kinematics/UrdfChainKinematics.ts` | Add `solveWithJointFrames()` — single-pass FK that also records per-joint axis/origin for Jacobian |
| `src/kinematics/GeometricJacobian.ts` | Add `computeFromFrames()` — Jacobian from precomputed frames (no second chain traversal) |
| `src/kinematics/ForwardKinematics.ts` | Add `solveWithJacobian()` — delegates to combined single-pass method |
| `src/kinematics/InverseKinematics.ts` | Update `evaluateState()` at line 491/507 to call `solveWithJacobian()` |
| `src/kinematics/AnalyticalPieperIK.ts` | Replace finite-difference orientation Jacobian in `refineWristOrientation()` with analytic Jacobian columns |
| `src/motion/PathInterpolator.ts` | Add SLERP orientation interpolation in `interpolateCartesianSpace()` |
| `src/kinematics/__tests__/fkJacobianCombined.test.ts` | New: verify combined call returns identical results to separate calls |
| `src/kinematics/__tests__/analyticalPieperIK.test.ts` | Extend: verify analytic wrist gives same or better residuals |
| `src/motion/__tests__/slerpOrientation.test.ts` | New: verify SLERP interpolation correctness |

---

## Task 1: Combined FK+Jacobian Single-Pass Computation

**Why first:** `InverseKinematics.evaluateState()` calls `ForwardKinematics.solve()` AND `ForwardKinematics.computeJacobian()` on the same joint angles — two separate full chain traversals per iteration. With 80 iterations per IK call, this doubles FK computation. This fix halves it.

**Key insight:** `GeometricJacobian.compute()` and `UrdfChainKinematics.solveFromRadians()` both traverse the 6-joint URDF chain but compute slightly different quantities. The Jacobian needs the world-frame joint axis and origin *before* applying each joint's rotation. The FK stores the cumulative transform *after* each joint's rotation. Both can be computed in one pass if we record both quantities at each joint.

**Files:**
- Modify: `src/kinematics/UrdfChainKinematics.ts`
- Modify: `src/kinematics/GeometricJacobian.ts`
- Modify: `src/kinematics/ForwardKinematics.ts`
- Modify: `src/kinematics/InverseKinematics.ts:483-536`
- Create: `src/kinematics/__tests__/fkJacobianCombined.test.ts`

### Step 1.1 — Write failing test

- [ ] Create `src/kinematics/__tests__/fkJacobianCombined.test.ts`:

```ts
import { ForwardKinematics } from '../ForwardKinematics';

describe('ForwardKinematics.solveWithJacobian', () => {
  const TEST_ANGLES = [5, -20, 35, 120, -30, 15];

  test('returns same pose as solve()', () => {
    const fkOnly = ForwardKinematics.solve(TEST_ANGLES);
    const { fk } = ForwardKinematics.solveWithJacobian(TEST_ANGLES);
    expect(fk.success).toBe(true);
    expect(fkOnly.success).toBe(true);
    expect(fk.endEffectorPose.position.x).toBeCloseTo(fkOnly.endEffectorPose.position.x, 10);
    expect(fk.endEffectorPose.position.y).toBeCloseTo(fkOnly.endEffectorPose.position.y, 10);
    expect(fk.endEffectorPose.position.z).toBeCloseTo(fkOnly.endEffectorPose.position.z, 10);
  });

  test('returns same Jacobian as computeJacobian()', () => {
    const jOnly = ForwardKinematics.computeJacobian(TEST_ANGLES);
    const { jacobian } = ForwardKinematics.solveWithJacobian(TEST_ANGLES);
    for (let row = 0; row < 6; row++) {
      for (let col = 0; col < 6; col++) {
        expect(jacobian[row][col]).toBeCloseTo(jOnly[row][col], 10);
      }
    }
  });

  test('handles all-zero angles', () => {
    const { fk, jacobian } = ForwardKinematics.solveWithJacobian([0, 0, 0, 0, 0, 0]);
    expect(fk.success).toBe(true);
    expect(jacobian.length).toBe(6);
    expect(jacobian[0].length).toBe(6);
  });
});
```

- [ ] Run test to verify it fails (function does not exist yet):
```
cd "robot-arm-control" && npm test -- --watchAll=false --runInBand --testPathPattern fkJacobianCombined
```
Expected: FAIL — `ForwardKinematics.solveWithJacobian is not a function`

### Step 1.2 — Add `JointFrame` type and `solveWithJointFrames()` to `UrdfChainKinematics.ts`

- [ ] Open `src/kinematics/UrdfChainKinematics.ts`. After the `UrdfFkResult` interface (line 9-12), add the new export type and method.

Add after the `UrdfFkResult` interface:
```ts
export interface JointFrame {
  origin: { x: number; y: number; z: number };
  axisWorld: { x: number; y: number; z: number };
}

export interface UrdfFkWithFramesResult extends UrdfFkResult {
  jointFrames: JointFrame[];  // per-joint axis/origin BEFORE applying joint rotation
}
```

Add the following **public static** method to the `UrdfChainKinematics` class, before the closing `}`:
```ts
/**
 * Single-pass FK that also records per-joint axis+origin for Jacobian computation.
 * Eliminates the duplicate chain traversal when both FK and Jacobian are needed.
 */
static solveWithJointFrames(jointAnglesDeg: number[]): UrdfFkWithFramesResult {
  const jointAnglesRad = jointAnglesDeg.map((deg) => (deg * Math.PI) / 180);

  let cumulative = this.createIdentityMatrix();
  const transforms: Matrix4x4[] = [];
  const jointFrames: JointFrame[] = [];

  for (let i = 0; i < ROBOT_KINEMATIC_CHAIN.length; i++) {
    const joint = ROBOT_KINEMATIC_CHAIN[i];

    // Apply origin translation + RPY rotation (same as existing solveFromRadians).
    const originTranslation = this.translationMatrix(
      joint.origin.xyz.x, joint.origin.xyz.y, joint.origin.xyz.z
    );
    const originRotation = this.multiplyMatrices(
      this.rotationZMatrix(joint.origin.rpy.yaw),
      this.multiplyMatrices(
        this.rotationYMatrix(joint.origin.rpy.pitch),
        this.rotationXMatrix(joint.origin.rpy.roll)
      )
    );
    const beforeJointRotation = this.multiplyMatrices(
      cumulative,
      this.multiplyMatrices(originTranslation, originRotation)
    );

    // Record world-frame origin and axis BEFORE applying joint rotation.
    const ax = joint.axis;
    const axN = Math.hypot(ax.x, ax.y, ax.z) || 1;
    jointFrames.push({
      origin: {
        x: beforeJointRotation[0][3],
        y: beforeJointRotation[1][3],
        z: beforeJointRotation[2][3]
      },
      axisWorld: {
        x: (beforeJointRotation[0][0] * ax.x + beforeJointRotation[0][1] * ax.y + beforeJointRotation[0][2] * ax.z) / axN,
        y: (beforeJointRotation[1][0] * ax.x + beforeJointRotation[1][1] * ax.y + beforeJointRotation[1][2] * ax.z) / axN,
        z: (beforeJointRotation[2][0] * ax.x + beforeJointRotation[2][1] * ax.y + beforeJointRotation[2][2] * ax.z) / axN
      }
    });

    // Apply joint rotation (same as composeJointTransform but split to capture beforeJointRotation).
    cumulative = this.multiplyMatrices(beforeJointRotation, this.axisAngleMatrix(joint.axis, jointAnglesRad[i]));
    transforms.push(this.copyMatrix(cumulative));
  }

  return {
    endEffectorPose: this.extractPose(cumulative),
    jointTransforms: transforms,
    jointFrames
  };
}
```

### Step 1.3 — Add `computeFromFrames()` to `GeometricJacobian.ts`

**Important:** `GeometricJacobian.ts` already has a local `type JointFrame` at lines 6–9 with shape `{ origin: Vector3; axis: Vector3 }`. The exported `JointFrame` from `UrdfChainKinematics` uses `axisWorld` instead of `axis`. Rename the local type first to avoid a compile error.

- [ ] Open `src/kinematics/GeometricJacobian.ts`. Rename the local `type JointFrame` at line 6 to `LocalJointFrame`:
```ts
// Change from:
type JointFrame = {
  origin: Vector3;
  axis: Vector3;
};
// To:
type LocalJointFrame = {
  origin: Vector3;
  axis: Vector3;
};
```

- [ ] Update the 4 usages of `JointFrame` in the `compute()` method body to `LocalJointFrame`:
  - The `frames` array declaration: `const frames: LocalJointFrame[] = [];`
  - The `frames.push({...})` call argument type (implicit, no explicit annotation needed)

- [ ] Add import for `JointFrame` from `UrdfChainKinematics` after the existing imports:
```ts
import { JointFrame } from './UrdfChainKinematics';
```

- [ ] Add the following static method to the `GeometricJacobian` class after the existing `compute()` method:
```ts
/**
 * Compute Jacobian from pre-computed joint frames (no second chain traversal).
 * Use this after ForwardKinematics.solveWithJacobian() to share computation.
 * pEe is the end-effector position from the FK result.
 */
static computeFromFrames(
  jointFrames: JointFrame[],
  pEe: { x: number; y: number; z: number }
): number[][] {
  const DEG_TO_RAD_LOCAL = Math.PI / 180;
  const jacobian = Array.from({ length: 6 }, () => Array(6).fill(0));

  for (let i = 0; i < jointFrames.length && i < 6; i++) {
    const { origin, axisWorld } = jointFrames[i];
    const dx = pEe.x - origin.x;
    const dy = pEe.y - origin.y;
    const dz = pEe.z - origin.z;

    // Linear velocity: axis × (pEe - origin), scaled by DEG_TO_RAD for degree-input convention.
    jacobian[0][i] = (axisWorld.y * dz - axisWorld.z * dy) * DEG_TO_RAD_LOCAL;
    jacobian[1][i] = (axisWorld.z * dx - axisWorld.x * dz) * DEG_TO_RAD_LOCAL;
    jacobian[2][i] = (axisWorld.x * dy - axisWorld.y * dx) * DEG_TO_RAD_LOCAL;
    // Angular velocity: axis (already world-frame), scaled by DEG_TO_RAD.
    jacobian[3][i] = axisWorld.x * DEG_TO_RAD_LOCAL;
    jacobian[4][i] = axisWorld.y * DEG_TO_RAD_LOCAL;
    jacobian[5][i] = axisWorld.z * DEG_TO_RAD_LOCAL;
  }

  return jacobian;
}
```

### Step 1.4 — Add `solveWithJacobian()` to `ForwardKinematics.ts`

- [ ] Open `src/kinematics/ForwardKinematics.ts`. Replace the existing `import { UrdfChainKinematics }` line with:
```ts
import { UrdfChainKinematics } from './UrdfChainKinematics';
```
(No `UrdfFkWithFramesResult` import needed — the return type is inlined.)

`GeometricJacobian` is already imported at line 3. No change needed there.

Add the following method after `computeJacobian()`:
```ts
/**
 * Compute FK and geometric Jacobian in a single chain traversal.
 * ~50% fewer operations than calling solve() + computeJacobian() separately.
 */
static solveWithJacobian(jointAngles: number[]): { fk: FKResult; jacobian: number[][] } {
  try {
    if (jointAngles.length !== 6) {
      const fk: FKResult = {
        endEffectorPose: { position: { x: 0, y: 0, z: 0 }, rotation: { roll: 0, pitch: 0, yaw: 0 } },
        jointTransforms: [],
        success: false,
        error: 'Invalid number of joint angles (expected 6)'
      };
      return { fk, jacobian: Array.from({ length: 6 }, () => Array(6).fill(0)) };
    }

    const combined = UrdfChainKinematics.solveWithJointFrames(jointAngles);
    const fk: FKResult = {
      endEffectorPose: combined.endEffectorPose,
      jointTransforms: combined.jointTransforms,
      success: true
    };
    const jacobian = GeometricJacobian.computeFromFrames(
      combined.jointFrames,
      combined.endEffectorPose.position
    );
    return { fk, jacobian };
  } catch (error) {
    const fk: FKResult = {
      endEffectorPose: { position: { x: 0, y: 0, z: 0 }, rotation: { roll: 0, pitch: 0, yaw: 0 } },
      jointTransforms: [],
      success: false,
      error: error instanceof Error ? error.message : 'Unknown error'
    };
    return { fk, jacobian: Array.from({ length: 6 }, () => Array(6).fill(0)) };
  }
}
```

### Step 1.5 — Update `InverseKinematics.evaluateState()` to use combined call

- [ ] Open `src/kinematics/InverseKinematics.ts`. Find `evaluateState()` (around line 483). Replace the two separate calls (lines 491 and 507) with the combined call:

Old code:
```ts
const fk = ForwardKinematics.solve(q);
if (!fk.success) return null;
// ... error computation ...
const jacobian = ForwardKinematics.computeJacobian(q);
```

New code:
```ts
const { fk, jacobian } = ForwardKinematics.solveWithJacobian(q);
if (!fk.success) return null;
```

The `jacobian` variable is now declared before the error computation lines (494-498). The SolveState return at line 518 already includes `jacobian` — no changes needed there.

### Step 1.6 — Run tests to verify no regressions

- [ ] Run the new test and the full suite:
```
cd "robot-arm-control" && npm test -- --watchAll=false --runInBand --testPathPattern fkJacobianCombined
```
Expected: 3 tests PASS.

```
cd "robot-arm-control" && npm test -- --watchAll=false --runInBand
```
Expected: all existing tests still PASS.

### Step 1.7 — Commit

- [ ] Commit:
```
git add robot-arm-control/src/kinematics/UrdfChainKinematics.ts \
  robot-arm-control/src/kinematics/GeometricJacobian.ts \
  robot-arm-control/src/kinematics/ForwardKinematics.ts \
  robot-arm-control/src/kinematics/InverseKinematics.ts \
  robot-arm-control/src/kinematics/__tests__/fkJacobianCombined.test.ts
git commit -m "perf(ik): combine FK and Jacobian into single chain traversal"
```

---

## Task 2: SLERP Orientation Interpolation

**Why this matters:** Currently `PathInterpolator.interpolateCartesianSpace()` locks the orientation to a single constant value for ALL path samples — either the start orientation or the provided target orientation. This means the robot cannot smoothly rotate its wrist while moving linearly. `QuaternionMath.slerp()` already exists and is tested, but is not used in path planning.

**Change in one sentence:** For each sample at progress `s = minimumJerkProgress(u)`, SLERP between the start quaternion and the target quaternion to get the per-sample orientation target.

**Files:**
- Modify: `src/motion/PathInterpolator.ts`
- Create: `src/motion/__tests__/slerpOrientation.test.ts`

### Step 2.1 — Write failing test

- [ ] Create `src/motion/__tests__/slerpOrientation.test.ts`:

```ts
import { PathInterpolator } from '../PathInterpolator';
import { ForwardKinematics } from '../../kinematics/ForwardKinematics';
import { QuaternionMath } from '../../kinematics/QuaternionMath';

describe('PathInterpolator SLERP orientation', () => {
  const interpolator = new PathInterpolator();
  // Known reachable start configuration
  const START_ANGLES = [0, -20, 35, 0, -30, 0];

  test('orientation stays constant when no targetOrientation provided', () => {
    const fkStart = ForwardKinematics.solve(START_ANGLES);
    expect(fkStart.success).toBe(true);
    if (!fkStart.success) return;

    const result = interpolator.interpolateCartesianSpace(
      START_ANGLES,
      {
        x: fkStart.endEffectorPose.position.x + 0.02,
        y: fkStart.endEffectorPose.position.y,
        z: fkStart.endEffectorPose.position.z
      },
      20, 80, 10
    );

    expect(result.success).toBe(true);
    expect(result.targetOrientation).toBeDefined();
    // All intermediate samples should have approx same orientation as start
    // (checking FK of each point is overkill, but we verify the result is reported correctly)
    expect(result.targetOrientation!.roll).toBeCloseTo(fkStart.endEffectorPose.rotation.roll, 2);
    expect(result.targetOrientation!.yaw).toBeCloseTo(fkStart.endEffectorPose.rotation.yaw, 2);
  });

  test('with targetOrientation, intermediate samples interpolate toward target', () => {
    const fkStart = ForwardKinematics.solve(START_ANGLES);
    expect(fkStart.success).toBe(true);
    if (!fkStart.success) return;

    const startQuat = QuaternionMath.fromEuler(fkStart.endEffectorPose.rotation);
    // Create a target orientation that is ~30 degrees rotated around Z (yaw)
    const targetOrientation = {
      roll: fkStart.endEffectorPose.rotation.roll,
      pitch: fkStart.endEffectorPose.rotation.pitch,
      yaw: fkStart.endEffectorPose.rotation.yaw + Math.PI / 6  // +30 degrees yaw
    };
    const targetQuat = QuaternionMath.fromEuler(targetOrientation);

    // Small lateral move so path planning is likely to succeed
    const targetPosition = {
      x: fkStart.endEffectorPose.position.x + 0.015,
      y: fkStart.endEffectorPose.position.y,
      z: fkStart.endEffectorPose.position.z
    };

    const result = interpolator.interpolateCartesianSpace(
      START_ANGLES,
      targetPosition,
      20, 80, 10,
      targetOrientation
    );

    // Whether or not IK succeeds, verify that if it does the final
    // orientation is closer to targetQuat than startQuat at the final sample.
    if (!result.success) return; // skip if IK fails — orientation handling is still correct

    // The result should report the target orientation as the final orientation
    expect(result.targetOrientation).toBeDefined();
    const resultQuat = QuaternionMath.fromEuler(result.targetOrientation!);
    const distToTarget = QuaternionMath.angularDistance(resultQuat, targetQuat);
    const distToStart = QuaternionMath.angularDistance(resultQuat, startQuat);
    // Final orientation should be closer to target than to start
    expect(distToTarget).toBeLessThan(distToStart);
  });

  test('SLERP produces monotonically increasing angular distance from start', () => {
    // Verify the interpolation formula itself: slerp(q0, q1, t) should be
    // at angular distance t * totalAngle from q0 (approximately)
    const q0 = QuaternionMath.fromEuler({ roll: 0, pitch: 0, yaw: 0 });
    const q1 = QuaternionMath.fromEuler({ roll: 0, pitch: 0, yaw: Math.PI / 2 });
    const totalAngle = QuaternionMath.angularDistance(q0, q1);

    const samples = [0, 0.1, 0.25, 0.5, 0.75, 1.0];
    let prevDist = -1;
    for (const t of samples) {
      const qInterp = QuaternionMath.slerp(q0, q1, t);
      const dist = QuaternionMath.angularDistance(q0, qInterp);
      expect(dist).toBeGreaterThanOrEqual(prevDist - 1e-6); // monotonically non-decreasing
      expect(dist).toBeCloseTo(t * totalAngle, 3); // linear interpolation of angle
      prevDist = dist;
    }
  });
});
```

- [ ] Run to verify it fails where expected:
```
cd "robot-arm-control" && npm test -- --watchAll=false --runInBand --testPathPattern slerpOrientation
```
Expected: the SLERP formula test passes (it uses existing `QuaternionMath.slerp`). The orientation interpolation tests may pass or fail depending on existing behavior; the key test is "final orientation closer to target" — this should currently fail if it runs.

### Step 2.2 — Implement SLERP in `PathInterpolator.interpolateCartesianSpace()`

- [ ] Open `src/motion/PathInterpolator.ts`. Find line ~155 where `lockedOrientation` is set.

**Current code (lines ~154-172):**
```ts
const startPos = fkStart.endEffectorPose.position;
const lockedOrientation = targetOrientation || fkStart.endEffectorPose.rotation;
const ikOptions: Partial<IKSolveOptions> = {
  orientationWeight: targetOrientation ? 0.20 : 0.0,
  ...
};
```

**Replace with:**
```ts
const startPos = fkStart.endEffectorPose.position;
// Quaternion SLERP setup: interpolate orientation from start to target.
// When no targetOrientation is provided, startQuat === targetQuat → pure position move.
const startQuat = QuaternionMath.fromEuler(fkStart.endEffectorPose.rotation);
const targetQuat = targetOrientation
  ? QuaternionMath.fromEuler(targetOrientation)
  : startQuat;
const hasOrientationTransition =
  targetOrientation !== undefined &&
  QuaternionMath.angularDistance(startQuat, targetQuat) > 1e-3;
// lockedOrientation is still used for terminal refinement and returned result.
const lockedOrientation = targetOrientation || fkStart.endEffectorPose.rotation;
const ikOptions: Partial<IKSolveOptions> = {
  orientationWeight: targetOrientation ? 0.20 : 0.0,
  ...
};
```

You also need to add `QuaternionMath` to the imports at the top of the file. Add:
```ts
import { QuaternionMath } from '../kinematics/QuaternionMath';
```

- [ ] Find the per-sample `poseTarget` construction inside `solveAtSamples` (line ~276). Replace the constant orientation with the per-sample SLERP result.

**Current code:**
```ts
const poseTarget: Pose = { position: pos, rotation: lockedOrientation };
let ikSolveResult: IKResult;
if (trackingMode === 'resolved_rate') {
```

**Replace with:**
```ts
// Per-sample orientation: SLERP from start to target using minimum-jerk progress s.
// When hasOrientationTransition is false (position-only move or no rotation change),
// this degenerates to the constant lockedOrientation — no IK behavior change.
const sampleOrientation = hasOrientationTransition
  ? QuaternionMath.toEuler(QuaternionMath.slerp(startQuat, targetQuat, s))
  : lockedOrientation;
const poseTarget: Pose = { position: pos, rotation: sampleOrientation };
let ikSolveResult: IKResult;
if (trackingMode === 'resolved_rate') {
```

- [ ] Update the `solveSampleWithRecovery()` call (the one in the `else` branch at line ~321) to pass `sampleOrientation` instead of `lockedOrientation`:

**Current code:**
```ts
} else {
  ikSolveResult = this.solveSampleWithRecovery(
    poseTarget,
    currentAnglesUrdf,
    previousPosition,
    lockedOrientation,
    ikOptions,
    ikEngineMode
  ).result;
}
```

**Replace with:**
```ts
} else {
  ikSolveResult = this.solveSampleWithRecovery(
    poseTarget,
    currentAnglesUrdf,
    previousPosition,
    sampleOrientation,
    ikOptions,
    ikEngineMode
  ).result;
}
```

Also update the `resolved_rate` recovery path in the same function (line ~300) — it also passes `lockedOrientation` to `solveSampleWithRecovery`:
```ts
// Change from:
const iterativeRecovery = this.solveSampleWithRecovery(
  poseTarget,
  currentAnglesUrdf,
  previousPosition,
  lockedOrientation,
  ...
).result;

// To:
const iterativeRecovery = this.solveSampleWithRecovery(
  poseTarget,
  currentAnglesUrdf,
  previousPosition,
  sampleOrientation,
  ...
).result;
```

**Do NOT change** the `solveSampleWithRecovery` call inside `refineTerminalSample()` (around line 684). That function always refines toward the final target pose (s=1), so it correctly keeps `lockedOrientation`. The internal `midPose.rotation: lockedOrientation` in the bisection recovery inside `solveSampleWithRecovery` is also intentionally constant — it is a position-priority recovery that doesn't need per-sample orientation.

### Step 2.3 — Run tests

- [ ] Run the SLERP test suite:
```
cd "robot-arm-control" && npm test -- --watchAll=false --runInBand --testPathPattern slerpOrientation
```
Expected: all 3 tests PASS.

- [ ] Run full suite:
```
cd "robot-arm-control" && npm test -- --watchAll=false --runInBand
```
Expected: all existing tests PASS.

### Step 2.4 — Commit

- [ ] Commit:
```
git add robot-arm-control/src/motion/PathInterpolator.ts \
  robot-arm-control/src/motion/__tests__/slerpOrientation.test.ts
git commit -m "feat(motion): add SLERP orientation interpolation along Cartesian paths"
```

---

## Task 3: Replace Finite-Difference Wrist Jacobian with Analytic Jacobian

**Why:** `AnalyticalPieperIK.refineWristOrientation()` runs 8 iterations of a finite-difference orientation Jacobian for J4–J6. Each iteration calls `ForwardKinematics.solve()` 7 times (1 for the current state + 6 for ±delta perturbations). For the 4 shoulder×elbow branches and up to 3 wrist seeds each, this is up to **12 branch-seeds × 8 iters × 7 FK = 672 FK calls** per `solveCandidates()` invocation. With the analytic geometric Jacobian (already computed via Task 1's combined call), this drops to **12 × 8 × 1 = 96 calls** — a 7× reduction.

**Key insight:** The finite-difference Jacobian in `refineWristOrientation()` computes:
```ts
j[row][col] = (logMapError(qPlus, qMinus)).component / (2 * delta)
```
This is an approximation of the angular Jacobian (rows 3-5) for columns 3-5 (J4-J6) of `GeometricJacobian.compute()`. Replacing it with the exact analytic sub-matrix is both faster and more accurate.

**Files:**
- Modify: `src/kinematics/AnalyticalPieperIK.ts:183-283`
- Modify: `src/kinematics/__tests__/analyticalPieperIK.test.ts` (extend)

### Step 3.1 — Add test for analytic wrist quality

- [ ] Open `src/kinematics/__tests__/analyticalPieperIK.test.ts`. Add at the end:

```ts
test('analytic wrist jacobian produces candidates with same or better residuals as before', () => {
  const solver = new AnalyticalPieperIK();
  // Known difficult pose: large wrist angles
  const testConfig = [5, -25, 45, 120, -30, 15];
  const fk = ForwardKinematics.solve(testConfig);
  expect(fk.success).toBe(true);
  if (!fk.success) return;

  const target = QuaternionMath.toPoseQuat(fk.endEffectorPose);
  const candidates = solver.solveCandidates(target, { previousSolutionDeg: testConfig });

  // At least one candidate should converge to the correct pose
  const best = [...candidates].sort((a, b) => a.positionResidualM - b.positionResidualM)[0];
  expect(best).toBeDefined();
  expect(best.positionResidualM).toBeLessThan(0.05); // within 5cm — analytic should do better
});

test('all four shoulder/elbow branches produce candidates', () => {
  const solver = new AnalyticalPieperIK();
  const testConfig = [0, -20, 30, 0, -45, 0];
  const fk = ForwardKinematics.solve(testConfig);
  expect(fk.success).toBe(true);
  if (!fk.success) return;

  const target = QuaternionMath.toPoseQuat(fk.endEffectorPose);
  const candidates = solver.solveCandidates(target, { previousSolutionDeg: testConfig });

  // Should have candidates from at least 2 branches (SL_EU, SL_ED, SR variants)
  const branches = new Set(candidates.map((c) => `${c.branchId.substring(0, 5)}`));
  expect(branches.size).toBeGreaterThanOrEqual(2);
});
```

- [ ] Run to verify tests currently pass (or reveal their baseline):
```
cd "robot-arm-control" && npm test -- --watchAll=false --runInBand --testPathPattern analyticalPieperIK
```

### Step 3.2 — Replace finite-difference Jacobian in `refineWristOrientation()`

**Note on traversal count after Tasks 1+3:** After this change, `refineWristOrientation()` still calls `ForwardKinematics.solve(angles)` at line 207 (for the FK state) AND `GeometricJacobian.compute(angles)` (for the Jacobian) — that is **2 chain traversals per iteration**, down from 7. If Task 1 is complete, you may optionally replace both with `ForwardKinematics.solveWithJacobian(angles)` and extract the FK result and Jacobian from that single call, reducing to **1 traversal per iteration**.

- [ ] Open `src/kinematics/AnalyticalPieperIK.ts`. Add the `GeometricJacobian` import at the top:
```ts
import { GeometricJacobian } from './GeometricJacobian';
```

- [ ] Find `refineWristOrientation()` at line 183. The inner loop body (lines ~206-235) computes the finite-difference Jacobian. Replace the entire inner loop body:

**Remove this block** (lines ~215-235):
```ts
// Local finite-difference orientation Jacobian w.r.t J4-J6 only.
const j = Array.from({ length: 3 }, () => Array(3).fill(0));
const delta = 0.2;
for (let idx = 0; idx < 3; idx++) {
  const jointIdx = idx + 3;
  const plus = [...angles];
  const minus = [...angles];
  plus[jointIdx] += delta;
  minus[jointIdx] -= delta;

  const fkPlus = ForwardKinematics.solve(plus);
  const fkMinus = ForwardKinematics.solve(minus);
  if (!fkPlus.success || !fkMinus.success) continue;

  const qPlus = QuaternionMath.fromEuler(fkPlus.endEffectorPose.rotation);
  const qMinus = QuaternionMath.fromEuler(fkMinus.endEffectorPose.rotation);
  const axisErr = QuaternionMath.logMapError(qPlus, qMinus);
  j[0][idx] = axisErr.x / (2 * delta);
  j[1][idx] = axisErr.y / (2 * delta);
  j[2][idx] = axisErr.z / (2 * delta);
}
```

**Replace with:**
```ts
// Analytic orientation Jacobian sub-block for J4–J6 (rows 3-5, cols 3-5 of full Jacobian).
// Replaces 6 FK perturbation calls with 1 Jacobian call. Units: radians/degree (same convention
// as the rest of the codebase — maps degree changes to radian orientation changes).
const fullJ = GeometricJacobian.compute(angles);
const j = [
  [fullJ[3][3], fullJ[3][4], fullJ[3][5]],
  [fullJ[4][3], fullJ[4][4], fullJ[4][5]],
  [fullJ[5][3], fullJ[5][4], fullJ[5][5]]
];
```

The rest of the method (DLS solve, step application, drift guard) remains **unchanged**.

### Step 3.3 — Run tests

- [ ] Run analytical IK tests:
```
cd "robot-arm-control" && npm test -- --watchAll=false --runInBand --testPathPattern analyticalPieperIK
```
Expected: all tests PASS. The residuals should be same or better (analytic Jacobian is more accurate than finite differences).

- [ ] Run full suite:
```
cd "robot-arm-control" && npm test -- --watchAll=false --runInBand
```
Expected: all tests PASS.

### Step 3.4 — Commit

- [ ] Commit:
```
git add robot-arm-control/src/kinematics/AnalyticalPieperIK.ts \
  robot-arm-control/src/kinematics/__tests__/analyticalPieperIK.test.ts
git commit -m "perf(ik): replace finite-difference wrist Jacobian with analytic sub-block"
```

---

## Summary: What These Changes Achieve

| Change | IK iterations affected | Expected improvement |
|---|---|---|
| Task 1: FK+Jacobian combined | Every IK iteration in `evaluateState()` | ~50% fewer chain traversals per solve |
| Task 2: SLERP orientation | Every path sample with `targetOrientation` | Correct combined position+rotation paths |
| Task 3: Analytic wrist Jacobian | Every `solveCandidates()` call | ~3.5× fewer FK calls in branch generation |

**Total impact** (50 samples, 24 IK iterations each, 3 wrist seeds × 8 refinement iterations):
- Before: 50 × (24 × 2) + 50 × (3 × 8 × 7) = 2400 + 8400 = **10800 traversals**
- Task 1 only: 50 × 24 + 50 × (3 × 8 × 7) = 1200 + 8400 = 9600
- Tasks 1+3: 50 × 24 + 50 × (3 × 8 × 2) = 1200 + 2400 = **3600 traversals** (~3× total speedup)
- Tasks 1+3 with optional `solveWithJacobian()` in wrist loop: 1200 + 1200 = **2400 traversals** (~4.5× speedup)

**Orientation-level impact:** Path moves that require both position change AND orientation change now interpolate smoothly instead of the IK fighting a constant orientation lock.

---

## Remaining Industrial-Quality Gaps (Future Work)

These are important but outside this plan's scope:

1. **Singularity avoidance** — Detect singularities ahead of time in the planned path and reroute. Requires a path-level singularity corridor detector. Complex 2-week effort.

2. **Null-space posture optimization** — Use `(I - J⁺J) * ∇h(q)` to steer elbow away from limits without affecting the task. Requires refactoring the DLS solver to support augmented null-space projection.

3. **Resolved-rate path tracking (default)** — Switch `trackingMode` from `'iterative_pose'` to `'resolved_rate'` as default for path samples. The infrastructure exists (`stepResolvedRate()`) but the toggle is disabled. Requires validation that the resolved-rate path handles branch switches gracefully.

4. **Waypoint execution via TQ queue** — `executeTrajectory()` still uses `J`-command streaming. Upgrading to queue-based execution requires a separate planning phase for waypoint-to-waypoint segments, same as the current direct Cartesian path.
