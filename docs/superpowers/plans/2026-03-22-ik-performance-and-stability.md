# IK Performance & Stability Fix Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Fix the three broken symptoms of the Cartesian IK system — extreme solve latency (2 s+), frequent Cartesian control errors, and cracking/jerky robot motion — by targeting the five verified root causes identified during code investigation.

**Architecture:** Five focused, independent changes are made in priority order. Each fix targets a specific root cause found in the code: Jacobi-SVD frequency, iteration budget, wrist seed quality, branch-state leakage, and velocity spikes. No architectural reshuffling; no new files.

**Tech Stack:** React 19 + TypeScript 4.9 (CRA / react-scripts 5), Jest + React Testing Library, Zustand 5

---

## Root-Cause Index

| ID | File | Line(s) | Symptom |
|----|------|---------|---------|
| RC-1 | `InverseKinematics.ts` | 239–251 | Jacobi-SVD runs every iteration for endpoint_global → 100 sweeps × 220 iterations |
| RC-2 | `HybridIKSolver.ts` | 96 | `maxIterations: 220` — far too large for an already-seeded analytic solve |
| RC-3 | `AnalyticalPieperIK.ts` | 104–107, 179 | Wrist seeds hardcoded [0,0,0]/[180,0,180]; 20-iter Newton refine; no warm-start |
| RC-4 | `HybridIKSolver.ts` | 92, 305–340 | `lastBranchId` survives across endpoint_global move requests; contaminates next motion |
| RC-5 | `PathInterpolator.ts` | 783–795 | Interior joint velocities from finite differences can exceed firmware limit (120 °/s), causing audible cracking and firmware trajectory rejection |

---

## File Map

| File | Role in this fix |
|------|-----------------|
| `robot-arm-control/src/kinematics/InverseKinematics.ts` | Fix diagnostic stride scope (RC-1) |
| `robot-arm-control/src/kinematics/HybridIKSolver.ts` | Reduce maxIterations (RC-2); add `resetBranchState()` (RC-4) |
| `robot-arm-control/src/kinematics/AnalyticalPieperIK.ts` | Add warm-start seed, reduce wrist refine (RC-3) |
| `robot-arm-control/src/motion/PathInterpolator.ts` | Clamp interior velocities to firmware limit (RC-5) |
| `robot-arm-control/src/store/robotStore.ts` | Call `resetBranchState()` before each new Cartesian move (RC-4) |
| `robot-arm-control/src/kinematics/__tests__/solverScaling.test.ts` | Performance regression test (RC-1, RC-2) |
| `robot-arm-control/src/kinematics/__tests__/analyticalPieperIK.test.ts` | Warm-start seed test (RC-3) |
| `robot-arm-control/src/kinematics/__tests__/continuityPolicy.test.ts` | Branch reset test (RC-4) |
| `robot-arm-control/src/motion/__tests__/poseLockedInterpolation.test.ts` | Velocity clamp test (RC-5) |

---

## Task 1: Fix Jacobi-SVD Diagnostic Stride (RC-1)

**Why it's broken:** `computeDiagnosticsThisIteration` on line 250 of `InverseKinematics.ts` reads:
```ts
const computeDiagnosticsThisIteration = computeDiagnosticsDefault &&
  (!trackingLocal || iteration === 0 || iteration % diagnosticStride === 0);
```
For `endpoint_global` intent, `trackingLocal = false`, so `!trackingLocal = true` short-circuits the stride check. The Jacobi-SVD (`singularSpectrumFromJtJ` — 100 sweeps on a 6×6 matrix) runs on **every** iteration. The `diagnosticStride = 4` setting is completely unused for endpoint solves.

**Fix:** Apply the stride to **all** intents and cache the last known `minSingularValue` for non-computing iterations, so damping continues to use a reasonable estimate.

**Files:**
- Modify: `robot-arm-control/src/kinematics/InverseKinematics.ts:239–270`
- Test: `robot-arm-control/src/kinematics/__tests__/solverScaling.test.ts`

- [ ] **Step 1.1: Write the failing performance test**

Add to `robot-arm-control/src/kinematics/__tests__/solverScaling.test.ts`:

```ts
test('endpoint_global solve completes in under 200ms for a reachable target', () => {
  const solver = new InverseKinematics({ maxIterations: 80, tolerance: 0.001, dampingFactor: 0.01 });
  const seed = [0, 10, 30, 0, 0, 0];
  const fk = ForwardKinematics.solve(seed);
  expect(fk.success).toBe(true);
  if (!fk.success) return;

  const target = fk.endEffectorPose;
  const slightly_off = { ...target, position: { x: target.position.x + 0.02, y: target.position.y, z: target.position.z } };

  const t0 = performance.now();
  const result = solver.solvePoseWeighted(slightly_off, seed, {
    intent: 'endpoint_global',
    diagnosticStride: 4,
    computeDiagnostics: true,
    tolerancePositionM: 0.0015,
    toleranceOrientationRad: 0.035,
    toleranceWeighted: 0.01
  });
  const elapsed = performance.now() - t0;

  expect(result.success).toBe(true);
  expect(elapsed).toBeLessThan(200);
}, 5000);
```

Run: `npm test -- --testPathPattern=solverScaling --watchAll=false`

Expected: **FAIL** — test times out or `elapsed >= 200`

- [ ] **Step 1.2: Apply the stride fix to InverseKinematics.ts**

In `robot-arm-control/src/kinematics/InverseKinematics.ts`, inside `solvePoseWeighted`, change lines 238–251:

**Before:**
```ts
const diagnosticStride = Math.max(1, opts.diagnosticStride);
const computeDiagnosticsDefault = opts.computeDiagnostics || !trackingLocal;
// ...
for (let iteration = 0; iteration < iterationBudget; iteration++) {
  const computeDiagnosticsThisIteration = computeDiagnosticsDefault &&
    (!trackingLocal || iteration === 0 || iteration % diagnosticStride === 0);
```

**After:**
```ts
const diagnosticStride = Math.max(1, opts.diagnosticStride);
const computeDiagnosticsDefault = opts.computeDiagnostics || !trackingLocal;
// Cache last known minSingularValue to keep damping stable between stride gaps
let cachedMinSingularValue = Number.POSITIVE_INFINITY;
// ...
for (let iteration = 0; iteration < iterationBudget; iteration++) {
  const computeDiagnosticsThisIteration = computeDiagnosticsDefault &&
    (iteration === 0 || iteration % diagnosticStride === 0);
```

`evaluateState` doesn't have access to the loop cache. Instead, inject the cached value back into the state object after the call returns. In the main iteration loop (around line 253), the pattern is:

```ts
const current = this.evaluateState(targetPose, q, iteration, damping, opts, computeDiagnosticsThisIteration);
```

Immediately after that line, add the cache update/inject block:

```ts
// Cache minSingularValue so damping stays coherent when Jacobi is skipped.
if (current) {
  if (computeDiagnosticsThisIteration) {
    cachedMinSingularValue = current.metrics.minSingularValue;
  } else {
    current.metrics.minSingularValue = cachedMinSingularValue;
  }
}
```

No change is needed inside `evaluateState` itself — the cache lives entirely in the outer loop.

- [ ] **Step 1.3: Run test to verify it passes**

Run: `npm test -- --testPathPattern=solverScaling --watchAll=false`

Expected: **PASS**, `elapsed < 200`

- [ ] **Step 1.4: Run full kinematics test suite to check for regressions**

Run: `npm test -- --testPathPattern=kinematics --watchAll=false`

Expected: All tests pass

- [ ] **Step 1.5: Commit**

```bash
git add robot-arm-control/src/kinematics/InverseKinematics.ts
git add robot-arm-control/src/kinematics/__tests__/solverScaling.test.ts
git commit -m "perf(ik): apply diagnosticStride to all intents, cache minSingularValue between gaps

Jacobi-SVD (100 sweeps on J^T*J) ran on every iteration for endpoint_global
because !trackingLocal short-circuited the stride guard. With stride=4 this
cuts Jacobi-SVD work by 75% for all endpoint solves."
```

---

## Task 2: Reduce Iteration Budget (RC-2 + RC-3 partial)

> **Ordering note:** If any existing IK quality test fails after reducing `maxIterations: 80`, implement Task 3 (warm-start seed) first, then re-run Task 2's tests. The warm-start compensates for the tighter budget by providing better initial seeds.

**Why it's broken:**
- `HybridIKSolver` constructs `InverseKinematics` with `maxIterations: 220`. The analytical stage already provides a good seed, so 220 iterations are never needed — they just burn time.
- `AnalyticalPieperIK.refineWristOrientation` uses `maxIter = 20` with up to 9 FK calls per iteration (central-diff Jacobian for 3 axes + FK-after-step + drift check). That's 8 candidates × 20 × 9 = 1,440 FK calls for the analytical phase alone. Gradient descent converges in 3–6 iterations from a reasonable seed — 20 is wasteful.

**Files:**
- Modify: `robot-arm-control/src/kinematics/HybridIKSolver.ts:96`
- Modify: `robot-arm-control/src/kinematics/AnalyticalPieperIK.ts:179`
- Test: `robot-arm-control/src/kinematics/__tests__/solverScaling.test.ts`

- [ ] **Step 2.1: Add a combined timing+quality test**

Add to `solverScaling.test.ts`:

```ts
test('HybridIKSolver endpoint_global solve under 300ms for a standard reachable target', () => {
  const solver = new HybridIKSolver();
  const seed = [0, -20, 40, 0, 0, 0];
  const fk = ForwardKinematics.solve(seed);
  expect(fk.success).toBe(true);
  if (!fk.success) return;

  const target = {
    position: { x: fk.endEffectorPose.position.x + 0.03, y: fk.endEffectorPose.position.y, z: fk.endEffectorPose.position.z },
    rotation: fk.endEffectorPose.rotation
  };

  const t0 = performance.now();
  const result = solver.solvePose(target, seed, { mode: 'pose_lock', intent: 'endpoint_global', profile: 'balanced' });
  const elapsed = performance.now() - t0;

  expect(result.success).toBe(true);
  expect((result.quality?.positionResidualM ?? Infinity) * 1000).toBeLessThan(2.0);
  expect(elapsed).toBeLessThan(300);
}, 5000);
```

Import `HybridIKSolver` at the top of the test file.

Run: `npm test -- --testPathPattern=solverScaling --watchAll=false`
Expected: **FAIL** (too slow)

- [ ] **Step 2.2: Reduce maxIterations in HybridIKSolver**

In `robot-arm-control/src/kinematics/HybridIKSolver.ts`, line 96:

**Before:**
```ts
this.solver = new InverseKinematics({
  maxIterations: 220,
  tolerance: 0.001,
  dampingFactor: 0.01
});
```

**After:**
```ts
this.solver = new InverseKinematics({
  maxIterations: 80,
  tolerance: 0.001,
  dampingFactor: 0.01
});
```

- [ ] **Step 2.3: Reduce wrist refine iterations in AnalyticalPieperIK**

In `robot-arm-control/src/kinematics/AnalyticalPieperIK.ts`, line 179:

**Before:**
```ts
const maxIter = 20;
```

**After:**
```ts
const maxIter = 8;
```

- [ ] **Step 2.4: Run test to verify it passes with quality maintained**

Run: `npm test -- --testPathPattern=solverScaling --watchAll=false`

Expected: **PASS** — both timing and quality assertions green

- [ ] **Step 2.5: Run full test suite**

Run: `npm test -- --watchAll=false`

Expected: All tests pass. If any IK quality test fails with `maxIterations: 80`, it means the seed quality is not good enough — that is addressed in Task 3.

- [ ] **Step 2.6: Commit**

```bash
git add robot-arm-control/src/kinematics/HybridIKSolver.ts
git add robot-arm-control/src/kinematics/AnalyticalPieperIK.ts
git add robot-arm-control/src/kinematics/__tests__/solverScaling.test.ts
git commit -m "perf(ik): reduce numeric iteration budget 220→80, wrist refine 20→8

Analytical stage already provides a good seed; 220 iterations is never needed.
Wrist Newton descent converges in 3-6 iterations - 20 is wasteful.
Combined with stride fix this cuts worst-case endpoint solve time by ~80%."
```

---

## Task 3: Add Warm-Start Wrist Seed from Previous Solution (RC-3)

**Why it's broken:** When the robot is mid-trajectory, consecutive IK calls should produce similar J4–J6 angles. But `AnalyticalPieperIK` always starts wrist refinement from [0,0,0] or [180,0,180] — angles with no relation to the target. With the iteration budget cut to 8, bad seeds now fail more often.

**Fix:** When `options.previousSolutionDeg` is available (always provided by `HybridIKSolver` during interpolation), extract its J4–J6 values as a third wrist seed. This seed is nearly always close to the solution, so 2–3 refine iterations suffice.

**Files:**
- Modify: `robot-arm-control/src/kinematics/AnalyticalPieperIK.ts:104–111`
- Test: `robot-arm-control/src/kinematics/__tests__/analyticalPieperIK.test.ts`

- [ ] **Step 3.1: Write a failing test showing warm-start improves quality**

Add to `robot-arm-control/src/kinematics/__tests__/analyticalPieperIK.test.ts`:

```ts
import { AnalyticalPieperIK } from '../AnalyticalPieperIK';
import { ForwardKinematics } from '../ForwardKinematics';
import { QuaternionMath } from '../QuaternionMath';

test('warm-start seed from previousSolutionDeg produces a valid wrist candidate', () => {
  const solver = new AnalyticalPieperIK();
  const knownGood = [5, -25, 45, 120, -30, 15];
  const fk = ForwardKinematics.solve(knownGood);
  expect(fk.success).toBe(true);
  if (!fk.success) return;

  const targetPose = QuaternionMath.toPoseQuat(fk.endEffectorPose);

  // Without warm-start
  const withoutWarm = solver.solveCandidates(targetPose, {});
  // With warm-start: previousSolutionDeg provides J4–J6 close to correct
  const withWarm = solver.solveCandidates(targetPose, { previousSolutionDeg: knownGood });

  const bestWithout = withoutWarm.filter(c => c.valid).sort((a, b) => a.positionResidualM - b.positionResidualM)[0];
  const bestWith = withWarm.filter(c => c.valid).sort((a, b) => a.positionResidualM - b.positionResidualM)[0];

  // Warm-start should have at least as many valid candidates
  expect(withWarm.filter(c => c.valid).length).toBeGreaterThanOrEqual(withoutWarm.filter(c => c.valid).length);

  // Warm-start best candidate should have similar or better orientation residual
  if (bestWith && bestWithout) {
    expect(bestWith.orientationResidualRad).toBeLessThanOrEqual(bestWithout.orientationResidualRad + 0.01);
  }
});
```

Run: `npm test -- --testPathPattern=analyticalPieperIK --watchAll=false`

Expected: **FAIL** (`solveCandidates` ignores `previousSolutionDeg`)

- [ ] **Step 3.2: Add warm-start seed extraction to AnalyticalPieperIK.solveCandidates**

In `robot-arm-control/src/kinematics/AnalyticalPieperIK.ts`, replace lines 104–107:

**Before:**
```ts
      const wristSeeds = [
        { q: [0, 0, 0], wristFlip: 'N' as const },
        { q: [180, 0, 180], wristFlip: 'F' as const }
      ];
```

**After:**
```ts
      const wristSeeds: Array<{ q: [number, number, number]; wristFlip: 'N' | 'F' }> = [
        { q: [0, 0, 0], wristFlip: 'N' },
        { q: [180, 0, 180], wristFlip: 'F' }
      ];

      // Warm-start: if a previous solution is available, insert its J4–J6 as a
      // third seed. This is almost always the best seed during trajectory tracking
      // and costs nothing extra — it replaces a bad cold-start with a nearby guess.
      const prevSol = options?.previousSolutionDeg;
      if (Array.isArray(prevSol) && prevSol.length === 6) {
        const prevJ5 = prevSol[4] ?? 0;
        // Always label the warm-start seed as 'N'. The refine loop corrects
        // wrist flip naturally; a wrong label only costs one extra refine iter.
        wristSeeds.unshift({ q: [prevSol[3] ?? 0, prevJ5, prevSol[5] ?? 0], wristFlip: 'N' });
      }
```

Note: `unshift` places the warm-start seed **first** so it gets refined first. The flip detection is kept simple — the refine loop will correct it anyway.

- [ ] **Step 3.3: Run test to verify it passes**

Run: `npm test -- --testPathPattern=analyticalPieperIK --watchAll=false`

Expected: **PASS**

- [ ] **Step 3.4: Run full test suite**

Run: `npm test -- --watchAll=false`

Expected: All tests pass

- [ ] **Step 3.5: Commit**

```bash
git add robot-arm-control/src/kinematics/AnalyticalPieperIK.ts
git add robot-arm-control/src/kinematics/__tests__/analyticalPieperIK.test.ts
git commit -m "feat(ik): add warm-start wrist seed from previousSolutionDeg

Hardcoded seeds [0,0,0]/[180,0,180] waste all 8 wrist refine iterations
approaching from the wrong starting point. With previousSolutionDeg the
warm-start seed is typically 2-3° from the correct answer, converging in
1-2 iterations and giving better orientation accuracy under the tighter budget."
```

---

## Task 4: Reset Branch State on New Cartesian Move (RC-4)

**Why it's broken:** `HybridIKSolver.lastBranchId` is instance state updated whenever `tracking_local` or `resolved_rate` intent is used. After completing one Cartesian move, `lastBranchId` holds the branch of the final interpolation step. The next move starts with the wrong branch preference, causing the first IK call to pick an incorrect branch, which can cause a sudden joint-angle jump — audible as a crack.

**Fix:** Add `resetBranchState()` to `HybridIKSolver`. Call it from `robotStore.ts` at the start of each `moveToPosition()` call, before the Stage 1 planner runs.

**Files:**
- Modify: `robot-arm-control/src/kinematics/HybridIKSolver.ts` (add method)
- Modify: `robot-arm-control/src/store/robotStore.ts` (call method)
- Test: `robot-arm-control/src/kinematics/__tests__/continuityPolicy.test.ts`

- [ ] **Step 4.1: Write a failing test for branch state isolation**

Add to `robot-arm-control/src/kinematics/__tests__/continuityPolicy.test.ts`:

```ts
import { HybridIKSolver } from '../HybridIKSolver';
import { ForwardKinematics } from '../ForwardKinematics';

test('resetBranchState() clears lastBranchId so next endpoint solve starts fresh', () => {
  const solver = new HybridIKSolver();

  // Simulate a tracking_local solve that sets lastBranchId
  const seedA = [0, -20, 40, 90, -45, 0];
  const fkA = ForwardKinematics.solve(seedA);
  expect(fkA.success).toBe(true);
  if (!fkA.success) return;

  solver.solvePose(fkA.endEffectorPose, seedA, {
    mode: 'pose_lock',
    intent: 'tracking_local'
  });

  // The reset method must exist and be callable
  expect(() => solver.resetBranchState()).not.toThrow();

  // After reset, a solve at a completely different target should succeed
  const seedB = [15, -30, 55, -90, 30, 0];
  const fkB = ForwardKinematics.solve(seedB);
  expect(fkB.success).toBe(true);
  if (!fkB.success) return;

  const resultAfterReset = solver.solvePose(fkB.endEffectorPose, seedB, {
    mode: 'pose_lock',
    intent: 'endpoint_global',
    profile: 'balanced'
  });
  expect(resultAfterReset.success).toBe(true);
});
```

Run: `npm test -- --testPathPattern=continuityPolicy --watchAll=false`

Expected: **FAIL** — `solver.resetBranchState is not a function`

- [ ] **Step 4.2: Add resetBranchState() to HybridIKSolver**

In `robot-arm-control/src/kinematics/HybridIKSolver.ts`, add after the constructor (after line ~104):

```ts
  /**
   * Clear persisted branch state before a new endpoint_global move starts.
   * Call this from robotStore before each moveToPosition() to prevent branch
   * contamination from the previous motion's tracking_local interpolation.
   */
  resetBranchState(): void {
    this.lastBranchId = undefined;
  }
```

- [ ] **Step 4.3: Run test to verify it passes**

Run: `npm test -- --testPathPattern=continuityPolicy --watchAll=false`

Expected: **PASS**

- [ ] **Step 4.4: Call resetBranchState() in robotStore.ts before Stage 1 planning**

In `robot-arm-control/src/store/robotStore.ts` around line 1480, find the start of the `moveToPosition` request setup. Add the reset call right after `stopPlannerWorker()` — before any solver or planning code runs.

**Before (lines 1480–1483):**
```ts
    const requestId = ++activePlannerRequestId;
    stopPlannerWorker();
    const planningStartedAt = performance.now();
    const plannerDiagnostics: CartesianPlannerDiagnosticEvent[] = [];
```

**After:**
```ts
    const requestId = ++activePlannerRequestId;
    stopPlannerWorker();
    // Reset branch lock so this move starts fresh, not contaminated by the
    // previous motion's tracking_local interpolation phase.
    hybridEndpointSolver.resetBranchState();
    const planningStartedAt = performance.now();
    const plannerDiagnostics: CartesianPlannerDiagnosticEvent[] = [];
```

`hybridEndpointSolver` is the module-level singleton defined at the top of the store file (around line 239) — it is in scope here.

- [ ] **Step 4.5: Guard auto-release against unconverged results**

The auto-release at `HybridIKSolver.ts:327` sets `success: true` on `bestFailure.result` without checking if the IK actually converged — a candidate rejected only for branch mismatch could still have very high residuals if the IK didn't converge before being classified.

Find lines 326–341 in `robot-arm-control/src/kinematics/HybridIKSolver.ts`:

**Before:**
```ts
    // EDGE-12: Auto-release branch lock when preferred branch becomes unreachable.
    if (!bestSuccess && bestFailure?.result.failureCategory === 'branch_discontinuity') {
      const released: IKResult = {
        ...bestFailure.result,
        success: true,
```

**After:**
```ts
    // EDGE-12: Auto-release branch lock when preferred branch becomes unreachable.
    // Only auto-release if the candidate's IK actually converged to a reasonable pose.
    // A branch_discontinuity rejection with high residuals must not become a false success.
    const releasedPos = bestFailure?.result.quality?.positionResidualM ?? Number.POSITIVE_INFINITY;
    if (!bestSuccess && bestFailure?.result.failureCategory === 'branch_discontinuity' && releasedPos <= 0.015) {
      const released: IKResult = {
        ...bestFailure.result,
        success: true,
```

(0.015 m = 10× the standard 1.5 mm tolerance — anything larger is a genuine IK failure, not a branch mismatch.)

- [ ] **Step 4.6: Run full test suite**

Run: `npm test -- --watchAll=false`

Expected: All tests pass

- [ ] **Step 4.7: Commit**

```bash
git add robot-arm-control/src/kinematics/HybridIKSolver.ts
git add robot-arm-control/src/store/robotStore.ts
git add robot-arm-control/src/kinematics/__tests__/continuityPolicy.test.ts
git commit -m "fix(ik): reset branch state at the start of each new Cartesian move

lastBranchId persisted across moves (EDGE-13). After one trajectory's
tracking_local interpolation, the next endpoint_global solve inherited
the wrong branch, causing an initial joint-angle discontinuity (crack).
resetBranchState() is now called from moveToPosition() before Stage 1."
```

---

## Task 5: Clamp Interior Joint Velocities to Firmware Limit (RC-5)

**Why it's broken:** `PathInterpolator.populateJointVelocities` uses finite differences over non-uniform time intervals. When Cartesian interpolation produces unevenly-spaced points (adaptive refinement), or when IK solutions jump between branches, interior velocity estimates can briefly exceed 120 °/s — the firmware's `STREAM_MAX_SPEED_DEG_S`. The firmware either rejects the entire trajectory queue or executes the spike causing the audible crack.

**Fix:** After computing velocities by finite differences, clamp all interior velocities to `± MAX_JOINT_VEL_DEG_S = 120`. Endpoints stay forced to zero (existing behavior).

**Files:**
- Modify: `robot-arm-control/src/motion/PathInterpolator.ts:776–803`
- Test: `robot-arm-control/src/motion/__tests__/poseLockedInterpolation.test.ts`

- [ ] **Step 5.1: Write a failing test for velocity clamping**

Add to `robot-arm-control/src/motion/__tests__/poseLockedInterpolation.test.ts`:

```ts
test('populateJointVelocities clamps interior velocities to 120 deg/s', () => {
  // Access the private method through a subclass for testing purposes
  class TestInterpolator extends PathInterpolator {
    public exposedPopulate(points: TrajectoryPoint[]): void {
      (this as any).populateJointVelocities(points);
    }
  }

  const interp = new TestInterpolator();

  // Create two points with a very short time gap that would produce a
  // huge finite-difference velocity (simulating a branch switch jump)
  const points: TrajectoryPoint[] = [
    { time: 0.0, jointAngles: [0, 0, 0, 0, 0, 0], velocity: Array(6).fill(0), quality: {} as any },
    { time: 0.001, jointAngles: [0, 0, 0, 90, 0, 0], velocity: Array(6).fill(0), quality: {} as any }, // 90° in 1ms = 90000 °/s!
    { time: 1.0, jointAngles: [0, 0, 0, 90, 0, 0], velocity: Array(6).fill(0), quality: {} as any }
  ];

  interp.exposedPopulate(points);

  // Interior point (index 1) velocity for J4 should be clamped to ±120
  const interiorVel = points[1].velocity[3];
  expect(Math.abs(interiorVel ?? 0)).toBeLessThanOrEqual(120);
  // Endpoints must remain zero
  expect(points[0].velocity).toEqual(Array(6).fill(0));
  expect(points[2].velocity).toEqual(Array(6).fill(0));
});
```

Import `PathInterpolator` and `TrajectoryPoint` at the top of the test file.

Run: `npm test -- --testPathPattern=poseLockedInterpolation --watchAll=false`

Expected: **FAIL** — `interiorVel` is ~90000, not ≤120

- [ ] **Step 5.2: Add velocity clamping in populateJointVelocities**

In `robot-arm-control/src/motion/PathInterpolator.ts`, replace lines 776–803:

**Before:**
```ts
  private populateJointVelocities(points: TrajectoryPoint[]): void {
    if (points.length === 0) return;
    if (points.length === 1) {
      points[0].velocity = Array(6).fill(0);
      return;
    }

    for (let i = 0; i < points.length; i++) {
      const velocity = Array(6).fill(0);
      for (let j = 0; j < 6; j++) {
        if (i === 0) {
          const dt = Math.max(1e-6, points[i + 1].time - points[i].time);
          velocity[j] = (points[i + 1].jointAngles[j] - points[i].jointAngles[j]) / dt;
        } else if (i === points.length - 1) {
          const dt = Math.max(1e-6, points[i].time - points[i - 1].time);
          velocity[j] = (points[i].jointAngles[j] - points[i - 1].jointAngles[j]) / dt;
        } else {
          const dt = Math.max(1e-6, points[i + 1].time - points[i - 1].time);
          velocity[j] = (points[i + 1].jointAngles[j] - points[i - 1].jointAngles[j]) / dt;
        }
      }
      points[i].velocity = velocity;
    }

    // Cartesian direct move starts/stops at rest by default.
    points[0].velocity = Array(6).fill(0);
    points[points.length - 1].velocity = Array(6).fill(0);
  }
```

**After:**
```ts
  // Matches firmware STREAM_MAX_SPEED_DEG_S = 120.0 in config.h.
  // Velocities above this cause firmware trajectory rejection or audible cracking.
  private static readonly MAX_JOINT_VEL_DEG_S = 120.0;

  private populateJointVelocities(points: TrajectoryPoint[]): void {
    if (points.length === 0) return;
    if (points.length === 1) {
      points[0].velocity = Array(6).fill(0);
      return;
    }

    const maxVel = PathInterpolator.MAX_JOINT_VEL_DEG_S;

    for (let i = 0; i < points.length; i++) {
      const velocity = Array(6).fill(0);
      for (let j = 0; j < 6; j++) {
        let raw: number;
        if (i === 0) {
          const dt = Math.max(1e-6, points[i + 1].time - points[i].time);
          raw = (points[i + 1].jointAngles[j] - points[i].jointAngles[j]) / dt;
        } else if (i === points.length - 1) {
          const dt = Math.max(1e-6, points[i].time - points[i - 1].time);
          raw = (points[i].jointAngles[j] - points[i - 1].jointAngles[j]) / dt;
        } else {
          const dt = Math.max(1e-6, points[i + 1].time - points[i - 1].time);
          raw = (points[i + 1].jointAngles[j] - points[i - 1].jointAngles[j]) / dt;
        }
        // Clamp interior velocities. Endpoints are forced to zero below.
        velocity[j] = Math.max(-maxVel, Math.min(maxVel, raw));
      }
      points[i].velocity = velocity;
    }

    // Cartesian direct move starts/stops at rest by default.
    points[0].velocity = Array(6).fill(0);
    points[points.length - 1].velocity = Array(6).fill(0);
  }
```

- [ ] **Step 5.3: Run test to verify it passes**

Run: `npm test -- --testPathPattern=poseLockedInterpolation --watchAll=false`

Expected: **PASS**

- [ ] **Step 5.4: Run full test suite**

Run: `npm test -- --watchAll=false`

Expected: All tests pass

- [ ] **Step 5.5: Fix queue verification off-by-one (Bug #7)**

The check at `TrajectoryExecutionService.ts:119` passes silently when `queuedCount === 0`, meaning an upload that the firmware completely discarded is not caught.

**Before (line 119):**
```ts
      if (Number.isFinite(queuedCount) && queuedCount > 0 && queuedCount < points.length) {
        throw new Error(
          `Trajectory queue verification failed: uploaded ${points.length}, device reports ${queuedCount}`
        );
      }
```

**After:**
```ts
      if (queuedCount !== points.length) {
        throw new Error(
          `Trajectory queue verification failed: uploaded ${points.length}, device reports ${queuedCount}`
        );
      }
```

Also add a test to `robot-arm-control/src/services/cartesian/__tests__/TrajectoryExecutionService.test.ts`:

```ts
test('throws when device reports zero queued points after upload', async () => {
  const points = makeDummyPoints(3);
  const serialManager = {
    clearTrajectoryQueue: jest.fn(async () => {}),
    enqueueTrajectoryPoint: jest.fn(async () => {}),
    queryTrajectoryQueue: jest.fn(async () => ({
      type: 'TQ_STAT',
      data: { count: 0, running: false }  // device reports 0 - all dropped!
    })),
    runTrajectoryQueue: jest.fn(async () => {})
  };

  await expect(
    TrajectoryExecutionService.uploadAndRunTrajectoryQueue(serialManager as any, points)
  ).rejects.toThrow('Trajectory queue verification failed');
  expect(serialManager.runTrajectoryQueue).not.toHaveBeenCalled();
});
```

Where `makeDummyPoints` is already defined in the test file or can be imported from the existing test helper.

- [ ] **Step 5.6: Run full test suite**

Run: `npm test -- --watchAll=false`

Expected: All tests pass

- [ ] **Step 5.7: Commit**

```bash
git add robot-arm-control/src/motion/PathInterpolator.ts
git add robot-arm-control/src/motion/__tests__/poseLockedInterpolation.test.ts
git add robot-arm-control/src/services/cartesian/TrajectoryExecutionService.ts
git add robot-arm-control/src/services/cartesian/__tests__/TrajectoryExecutionService.test.ts
git commit -m "fix(trajectory): clamp velocities to 120 deg/s and fix queue verification gap

- PathInterpolator: clamp interior joint velocities to STREAM_MAX_SPEED_DEG_S=120
  to prevent firmware rejection and audible cracking from velocity spikes
- TrajectoryExecutionService: fix queue count verification to catch queuedCount=0,
  which previously passed silently and allowed empty trajectory execution"
```

---

## Task 6: Integration Smoke Test (All Fixes)

After all five fixes are in, run the complete test suite and verify no regressions.

**Files:**
- None (no new code)

- [ ] **Step 6.1: Run the full test suite**

From `robot-arm-control/`:

```bash
npm test -- --watchAll=false
```

Expected: All tests pass (or the same tests that were failing before this work still fail — no new failures).

- [ ] **Step 6.2: Check build compiles without TypeScript errors**

```bash
npm run build
```

Expected: Build succeeds with no TypeScript errors.

- [ ] **Step 6.3: Record baseline performance measurements**

Optionally, run the timing tests and note the elapsed time in a comment in `solverScaling.test.ts` for future comparison:

```
// Baseline after this fix: ~80-120ms for a typical endpoint_global solve
// Before fix: ~800-2000ms
```

---

## Expected Outcomes After All Tasks

| Symptom | Root cause fixed | Expected result |
|---------|-----------------|-----------------|
| Extreme slowness (2s+) | RC-1 (Jacobi stride), RC-2 (max iterations), RC-3 (wrist refine) | Endpoint solve: ~80–150ms vs ~2000ms |
| Cartesian control errors | RC-3 (warm-start seed), RC-4 (branch reset) | Significantly fewer IK failures; better convergence from good seeds |
| Cracking / jerky motion | RC-4 (branch reset at move start), RC-5 (velocity clamp) | Smooth transition between moves; no firmware velocity spikes |

---

## Notes for the Implementor

- **Test runner:** CRA uses react-scripts. Run all tests with `npm test -- --watchAll=false` from `robot-arm-control/`. Do NOT use `npx jest` directly.
- **`performance.now()`** is available in the Jest JSDOM environment — no polyfill needed.
- **Task order matters for Task 2:** If any existing IK quality test fails after lowering `maxIterations: 80`, implement Task 3 (warm-start seed) first, then re-run Task 2's tests. The warm-start compensates for the tighter budget.
- **Branch state**: `lastBranchId` is only updated and read when `useInstanceBranch = true` (tracking_local / resolved_rate). The reset in Task 4 ensures each new user-initiated move starts without inherited branch state from prior interpolation phases.
- **Velocity clamp constant**: `STREAM_MAX_SPEED_DEG_S = 120.0` is defined in firmware `config.h` and also in `robotStore.ts`. The constant added to `PathInterpolator` must stay in sync if `firmwareConfig` changes. A follow-up improvement would be to inject the firmware limit from the store rather than hardcoding it.
