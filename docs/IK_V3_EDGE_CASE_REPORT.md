# IK V3 Edge Case & Bug Report

**Date:** 2026-02-14
**Scope:** All new and modified files in `robot-arm-control/src/kinematics/`

---

## Critical Bugs

### BUG-01: `wrapDeg` infinite loop on NaN / extreme values

**File:** `AnalyticalPieperIK.ts:27-31`
```ts
const wrapDeg = (value: number): number => {
  let wrapped = value;
  while (wrapped > 180) wrapped -= 360;
  while (wrapped < -180) wrapped += 360;
  return wrapped;
};
```

If `value` is `NaN`, `Infinity`, or `-Infinity`, these while-loops never terminate and hang the browser tab. This can happen when `Math.atan2(0, 0)` feeds into downstream calculations or when the planar solver produces degenerate results.

**Fix:** Use modular arithmetic: `((value % 360) + 540) % 360 - 180`, or guard with `if (!Number.isFinite(value)) return 0`.

---

### BUG-02: `SingularityHandler.detectFlags` uses joint angles instead of Cartesian position for shoulder singularity

**File:** `SingularityHandler.ts:20-22`
```ts
const radial = Math.hypot(jointAnglesDeg[0] || 0, jointAnglesDeg[1] || 0);
if (radial < 1e-4) {
  flags.push('shoulder_axis_ambiguity');
}
```

This computes `hypot(J1_angle, J2_angle)` which is the norm of two joint angles in degrees — it has nothing to do with the wrist center's radial distance from the J1 axis. A shoulder singularity occurs when the **wrist center XY position** is near the J1 axis, not when J1 and J2 are both near zero. J1=0, J2=0 is a perfectly normal configuration with the arm extended forward.

**Impact:** False positives (flags normal configs as singular) and false negatives (misses actual shoulder singularities when angles are nonzero but the wrist center crosses the Z axis).

---

### BUG-03: `boundaryDistance` computed on clamped position, not actual position

**File:** `reachabilityAtlas.ts:107-108`
```ts
const clamped = clampToBounds(position, atlasModel);
const boundaryDistanceM = boundaryDistance(clamped, atlasModel);
```

When the position is **inside** bounds, `clamped === position` and this is correct. But the `boundaryDistanceM` for the inside-bounds path uses the clamped value, which will always be >= 0 even for boundary-adjacent points. This is fine.

However, when the position is **outside** bounds, the returned `boundaryDistanceM` is negative (correct), but the `closestReachablePoint` is just the AABB-clamped point — not the actual closest reachable point on the workspace surface. A corner of the bounding box is not necessarily reachable. This is misleading metadata that could cause the solver to aim for an unreachable point.

---

### BUG-04: `applyWristBypass` on successful solutions corrupts valid results

**File:** `InverseKinematics.ts:269` and `InverseKinematics.ts:423`
```ts
jointAngles: SingularityHandler.applyWristBypass(current.q),
```

The wrist bypass is applied unconditionally on **every** returned solution (both success and failure paths). `applyWristBypass` checks `sin(J5) < 0.015` and then sets `J4 = J4+J6, J6 = 0`.

Problem: the bypass threshold (`sin(J5) < 0.015`, i.e. J5 within ~0.86°) is checked **after** the solver converged. But the converged solution already satisfies the tolerance. Mutating J4 and J6 post-convergence **invalidates the FK** — the end-effector pose changes because J4+J6 is only invariant exactly at the singularity (sin(J5)=0), not at sin(J5)=0.014. This introduces up to ~1.4° of orientation error on otherwise valid solutions.

**Fix:** Either don't apply the bypass to successful solutions, or re-verify FK after bypass and reject if tolerance is violated.

---

### BUG-05: mathjs SVD result extraction is fragile and uses `any` casts

**File:** `InverseKinematics.ts:622, 654-680`

The `extractSingularValues` method probes for `.s`, `.S`, `.s._data`, `.S._data` with `any` types. This is fragile because:
- mathjs version updates could change the structure
- The fallback path (line 642-651) computes `sqrt(diag(J^T J))` which is NOT singular values — it's the diagonal of J^T J square-rooted, which only equals singular values if J^T J is diagonal (i.e., never for a real robot Jacobian)
- If `math.svd()` throws silently or returns an unexpected shape, the fallback gives wrong singular values, corrupting the singularity detection and damping policy

---

## Medium Severity Issues

### EDGE-06: Analytical IK shoulder flip produces configurations 180° apart with no transition check

**File:** `AnalyticalPieperIK.ts:78`
```ts
const q1Deg = wrapDeg(baseYaw + (branch.shoulder === 'R' ? 180 : 0));
```

The shoulder-right branch simply adds 180° to J1. But it doesn't negate J2/J3 to compensate — the same `solvePlanarShoulderElbow` is called with the same (x, y, z). This means the shoulder-right branch has the correct J1 for a rear approach but **wrong J2/J3** because the planar geometry should be solved from the opposite side. The elbow solver doesn't know which shoulder configuration it's solving for — it always uses `hypot(x,y) - 0.045` as the radial distance.

**Impact:** Shoulder-right candidates are geometrically incorrect. They will always have high residuals and be filtered out by the numeric refinement, wasting compute cycles. In rare cases they could converge to an unexpected configuration.

---

### EDGE-07: Wrist refinement uses finite-difference Jacobian but doesn't account for position coupling

**File:** `AnalyticalPieperIK.ts:195-215`

The wrist refinement loop computes a 3x3 orientation-only Jacobian for J4-J6 using finite differences. But changing J4-J6 also moves the end-effector **position** (the wrist is not a perfect spherical wrist in URDF — there are offsets). The refinement only minimizes orientation error and can drift the position away from the target.

After refinement, position error is checked at line 97-101, but by then the damage is done — the position could be several mm off from what the planar solver computed.

---

### EDGE-08: `ContinuityPolicy.rankCandidates` double-sorts with `AnalyticalPieperIK.solveCandidates`

**File:** `HybridIKSolver.ts:157-172`

Candidates are sorted inside `solveCandidates` (line 129-143) and then re-sorted by `ContinuityPolicy.rankCandidates` (line 168-172) with a very similar but not identical comparator. The two sorting passes use different weight factors for residuals (1000/50 vs 1000/50) and different tie-breaking. This redundant double-sort is wasteful and could produce unexpected ordering if the comparators disagree on edge cases.

---

### EDGE-09: `CollisionModel` returns `collisionFree: true` when FK fails

**File:** `CollisionModel.ts:80-86`

If FK fails (e.g., NaN angles), the collision check returns `collisionFree: true`. This means invalid configurations silently pass the collision gate. It should return `collisionFree: false` — if we can't compute link positions, we can't guarantee no collision.

---

### EDGE-10: `stepResolvedRate` always returns `success: true`

**File:** `InverseKinematics.ts:178-191`

The resolved-rate step always reports success regardless of residual magnitude. Even if the error is huge (target far outside workspace), the caller sees `success: true`. This could mislead trajectory planners into thinking the arm is tracking correctly when it's actually saturated.

---

### EDGE-11: Limit barrier gradient explodes at joint limits

**File:** `InverseKinematics.ts:526-535`
```ts
const marginMin = Math.max(eps, q[i] - this.minDeg[i]);
const marginMax = Math.max(eps, this.maxDeg[i] - q[i]);
gradient[i] = (1 / (marginMin * marginMin)) - (1 / (marginMax * marginMax));
```

When a joint is exactly at its limit, `marginMin` or `marginMax` = `eps` = 1e-6, giving gradient values of `1/(1e-12) = 1e12`. Even multiplied by `limitWeight * normalization` (where limitWeight is 0.2-0.45), this can produce enormous gradient terms that overwhelm the actual task-space error in the linear system, causing the solver to "bounce off" limits aggressively and oscillate.

The barrier function is not normalized relative to the range of each joint. A joint with a ±360° range gets the same barrier as one with ±30°.

---

### EDGE-12: Branch lock deadlock — all candidates rejected, solver stuck

**File:** `HybridIKSolver.ts:219-242`

When `branchLockEnabled=true` and the preferred branch becomes unreachable (e.g., approaching a singularity that requires branch switch), **every** successful candidate is rejected as `branch_discontinuity`. The solver returns the best failure, which is a valid-but-rejected solution. The caller has no way to automatically release the lock.

Over a trajectory, this means the arm freezes at the last valid position and the UI shows "Branch switch rejected" indefinitely. There's no automatic fallback or hysteresis to release the lock when the preferred branch is provably unreachable for N consecutive frames.

---

### EDGE-13: `lastBranchId` state leak across unrelated solve calls

**File:** `HybridIKSolver.ts:92, 279, 284, 292, 296`

`lastBranchId` is instance state on `HybridIKSolver` that persists across calls. If the solver is shared (singleton pattern via Zustand store), a `solvePosition` call for a waypoint preview will update `lastBranchId`, affecting the next `solvePose` call for actual tracking. There's no isolation between different use-contexts.

---

### EDGE-14: `QuaternionMath.angularDistance` returns 2*acos(|dot|) which can exceed π

**File:** `QuaternionMath.ts:172-177`
```ts
const dot = clamp(Math.abs(qa.w * qb.w + qa.x * qb.x + qa.y * qb.y + qa.z * qb.z), -1, 1);
return 2 * Math.acos(dot);
```

The `Math.abs()` on the dot product ensures we take the shortest path, but `clamp(..., -1, 1)` is then applied. Since `Math.abs` already makes it >= 0, the clamp range should be `[0, 1]`. As written, if `dot` is slightly > 1 due to float precision (which `Math.abs` doesn't prevent), `clamp` handles it — but the intent is obscured and the result range is [0, π] which is correct. However, if someone removes the `Math.abs` in a refactor, the clamp to [-1, 1] would allow `2*acos(-1) = 2π` which is wrong for angular distance.

---

## Low Severity / Code Quality

### EDGE-15: `math` from mathjs imported but only used for SVD — large bundle impact

**File:** `InverseKinematics.ts:2, 59`
```ts
import { all, create } from 'mathjs';
const math = create(all, {});
```

`create(all, {})` imports the **entire** mathjs library (~170KB minified). Only `math.svd()` is used. This significantly bloats the client bundle for a web app. Should use `import { svd } from 'mathjs'` or a lightweight SVD implementation.

---

### EDGE-16: `SingularityHandler.stabilizeWristStep` uses average of previous and next J5

**File:** `SingularityHandler.ts:62`
```ts
const nearSingularity = Math.abs(Math.sin(((prevJ5 + nextJ5) * 0.5) * DEG_TO_RAD)) < 0.03;
```

Averaging the previous and proposed J5 before checking singularity means a large step through the singularity (e.g., prev=5°, next=-5°, avg=0°) triggers stabilization, but a step that lands near the singularity from far away (prev=30°, next=0.5°, avg=15.25°) does not. This is backwards — the second case is more dangerous.

---

### EDGE-17: No `previousSolutionDeg` null propagation guard

**File:** `HybridIKSolver.ts:153`
```ts
const continuityPrevious = params.overrideOptions?.previousSolutionDeg || initialGuess;
```

`previousSolutionDeg` type is `number[] | null`. If it's explicitly `null`, the `||` operator falls through to `initialGuess`, which is correct. But if it's an **empty array** `[]`, it's truthy and gets used, causing `jointDistance` to compute over zero elements, returning 0 for all candidates — breaking the continuity ranking.

---

### EDGE-18: Collision model capsule radii are hardcoded constants

**File:** `CollisionModel.ts:97`
```ts
radius: i < 2 ? 0.03 : 0.022,
```

The 30mm and 22mm radii are arbitrary approximations not derived from the actual STL mesh geometry. If the real links are wider (e.g., at motor housings or cable routing), collisions will be missed. If they're narrower, valid configurations will be rejected.

---

### EDGE-19: `shouldCheckPair` skips base-to-endeffector (L0-L5) but not L0-L4 or L1-L5

**File:** `CollisionModel.ts:69-75`
```ts
if (Math.abs(i - j) <= 1) return false;
if (i === 0 && j === 5) return false;
```

The special case for L0-L5 is asymmetric and arbitrary. For a 6-link chain, the physically meaningful skip pairs should be based on the kinematic structure, not hard-coded indices. Missing: L0 vs L4 could matter at extreme reach-back configurations.

---

### EDGE-20: `AnalyticalPieperIK.solvePlanarShoulderElbow` uses empirical link lengths that may not match URDF

**File:** `AnalyticalPieperIK.ts:155-158`
```ts
const radial = Math.max(0.02, Math.hypot(x, y) - 0.045);
const vertical = z - 0.136;
const l1 = 0.16;
const l2 = 0.145;
```

These constants (`0.045`, `0.136`, `0.16`, `0.145`) are "empirical geometric model" approximations. The actual URDF has complex 3D offsets between joints (e.g., J2 has offset `(-0.0375, 0.02, 0.05595)` with `-π/2` roll). The simplified 2R model doesn't account for:
- The lateral Y offset of J2
- The Z offset between J3 and J4 (`d=0.0364`)
- The wrist center offset from J5/J6

This means analytical seeds will always have mm-level position error before numeric refinement, reducing the benefit of the analytical approach. Worse, near the workspace boundary where the 2R model is least accurate, the seeds may be outside the basin of convergence for the numeric refiner.

---

## Summary Table

| ID | Severity | File | Description |
|----|----------|------|-------------|
| BUG-01 | **CRITICAL** | AnalyticalPieperIK.ts | `wrapDeg` infinite loop on NaN/Infinity |
| BUG-02 | **CRITICAL** | SingularityHandler.ts | Shoulder singularity detection uses angles not positions |
| BUG-03 | **MEDIUM** | reachabilityAtlas.ts | `closestReachablePoint` is AABB corner, not workspace surface |
| BUG-04 | **CRITICAL** | InverseKinematics.ts | Wrist bypass on converged solutions invalidates FK |
| BUG-05 | **HIGH** | InverseKinematics.ts | mathjs SVD extraction fragile; fallback computes wrong values |
| EDGE-06 | **MEDIUM** | AnalyticalPieperIK.ts | Shoulder-right branch doesn't adjust planar geometry |
| EDGE-07 | **MEDIUM** | AnalyticalPieperIK.ts | Wrist refinement ignores position coupling |
| EDGE-08 | **LOW** | HybridIKSolver.ts | Redundant double-sort of candidates |
| EDGE-09 | **MEDIUM** | CollisionModel.ts | FK failure returns collisionFree=true |
| EDGE-10 | **MEDIUM** | InverseKinematics.ts | Resolved-rate always returns success=true |
| EDGE-11 | **HIGH** | InverseKinematics.ts | Limit barrier gradient explodes at boundaries |
| EDGE-12 | **HIGH** | HybridIKSolver.ts | Branch lock deadlock with no auto-release |
| EDGE-13 | **MEDIUM** | HybridIKSolver.ts | `lastBranchId` state leaks across contexts |
| EDGE-14 | **LOW** | QuaternionMath.ts | Angular distance clamp range slightly misleading |
| EDGE-15 | **LOW** | InverseKinematics.ts | Full mathjs import bloats bundle ~170KB |
| EDGE-16 | **LOW** | SingularityHandler.ts | J5 average for singularity check is backwards |
| EDGE-17 | **LOW** | HybridIKSolver.ts | Empty array `previousSolutionDeg` breaks continuity |
| EDGE-18 | **LOW** | CollisionModel.ts | Capsule radii are hardcoded, not from mesh |
| EDGE-19 | **LOW** | CollisionModel.ts | Skip-pair logic is incomplete |
| EDGE-20 | **MEDIUM** | AnalyticalPieperIK.ts | 2R link lengths don't match actual URDF geometry |
