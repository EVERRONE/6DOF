# Inverse Kinematics System Assessment Report

**Date:** 2026-02-14
**Scope:** Full IK pipeline in `robot-arm-control/src/kinematics/`

---

## Architecture Overview

The system has a layered design:

| Layer | File | Role |
|-------|------|------|
| FK engine | `UrdfChainKinematics.ts` | URDF-based forward kinematics |
| Jacobian | `GeometricJacobian.ts` | Analytic geometric Jacobian |
| Core solver | `InverseKinematics.ts` | Damped Least Squares (LM-style) |
| Multi-seed wrapper | `HybridIKSolver.ts` | Multi-seed, multi-stage strategy |
| Workspace | `reachabilityAtlas.ts` | Voxel-based reachability lookup |
| Diagnostics | `diagnostics.ts`, `qualityMetrics.ts` | Sensitivity & residual analysis |

---

## What's Good (Solid Foundations)

1. **Geometric Jacobian** — Properly computed from URDF chain frames with correct cross-product formulation. This is the right approach.

2. **Adaptive damping** — The LM-style adaptive damping (grow on reject, shrink on accept) with line-search backtracking is solid.

3. **Orientation error via quaternions** — Using axis-angle extraction from quaternion error avoids gimbal lock in the error metric.

4. **Multi-seed strategy** — `HybridIKSolver` generates analytic 2R seeds (elbow-up/down), wrist-bias perturbations, and shoulder-flip variants. This covers the most common IK branches.

5. **Staged relaxation** — Three stages (base / relaxed / robust) that progressively loosen damping and step bounds.

6. **Rich diagnostics** — Condition number, singular value spectrum, joint participation, failure categorisation.

7. **Reachability atlas** — Pre-computed voxel map for fast feasibility checks.

---

## What's Missing / Weak (vs. Industrial IK)

### 1. No Closed-Form Analytical IK for the Spherical Wrist

**Severity: HIGH**

The robot has a 3R spherical wrist (J4-J5-J6). Industrial 6-axis robots (UR, KUKA, Fanuc, ABB) exploit the **Pieper decomposition**: decouple position (J1-J3 solve the wrist center) from orientation (J4-J6 solve via closed-form Euler angle extraction from `R_wrist = R_0_3^T * R_target`).

Currently only a 2R planar approximation is used for seeds (`HybridIKSolver.ts:304-341`) and then everything is iterated numerically. The wrist joints (J4-J6) are initialised from `initialGuess` and must be found entirely by the numerical solver. This means:

- **8 analytical solutions** (2 shoulder × 2 elbow × 2 wrist flip) are reduced to ~4 rough seeds.
- The wrist orientation convergence depends entirely on the numerical solver groping its way there.
- Near wrist singularities (J5 ≈ 0° or 180°), the numerical solver will oscillate.

**Industrial approach:** Full Pieper decoupling gives an exact wrist-center position from J1-J3, then exact wrist orientation from J4-J6 via `R_wrist = R_0_3⁻¹ * R_desired`. All 8 solutions are enumerated in < 1 ms.

---

### 2. No Singularity Handling Strategy

**Severity: HIGH**

The solver detects singularities (`InverseKinematics.ts:185-189`) but only sets a failure category. There is no:

- **Singularity avoidance** via gradient projection in the null space.
- **Singularity-robust damping** (Nakamura-Hanafusa variable damping that scales λ with proximity to singular configurations).
- **Wrist singularity bypass** (when J5 ≈ 0°, J4+J6 are coupled — industrial controllers lock one and solve the other).
- **Shoulder singularity handling** (when the wrist center passes through the J1 axis).

Currently the solver just increases damping uniformly, which slows convergence everywhere rather than applying targeted damping only in the degenerate direction.

---

### 3. No Null-Space Optimisation

**Severity: MEDIUM-HIGH**

A 6-DOF arm at a non-singular pose has exactly 0 redundancy, but null-space projection is still needed for:

- **Joint limit avoidance** via gradient projection `(I - J⁺J) * ∇h(q)` where h is a joint-centering or limit-avoidance potential.
- **Weighted Damped Least Squares (WDLS)** with per-joint weights that increase near limits.
- Currently joint limits are handled by hard clamping (`InverseKinematics.ts:584-598`), which causes the solver to "stick" against limits and fight itself.

The posture regularisation at `InverseKinematics.ts:355-358` is in the right direction but with weight `1e-7` it is essentially invisible.

---

### 4. No Configuration-Space Continuity (Solution Selection)

**Severity: MEDIUM-HIGH**

For trajectory tracking, the solver should guarantee **C-space continuity** — selecting the IK solution closest to the previous configuration. Currently:

- `HybridIKSolver` picks the lowest-cost solution by a scoring function, but there is no explicit penalty for branch switching mid-trajectory.
- No detection of when the robot would need to pass through a singularity to reach a different branch.
- No path planning in joint space to verify the transition is smooth.

Industrial controllers maintain a "preferred branch" and only switch when explicitly commanded or when the current branch becomes unreachable.

---

### 5. Hand-Rolled Linear Algebra

**Severity: MEDIUM**

Everything is manual dense matrix operations:

- Gaussian elimination for 6×6 system (`InverseKinematics.ts:364-409`).
- Jacobi eigenvalue iteration for SVD (`InverseKinematics.ts:487-546`).
- Triple-nested matrix multiply everywhere.

Issues:

- No numerical stability guarantees (only basic partial pivoting).
- SVD via Jacobi eigenvalue method on J^T·J **squares the condition number** (loses half the significant digits).
- A proper SVD decomposition (Golub-Kahan bidiagonalisation) would give both the damped pseudoinverse and the singular values in one pass.

---

### 6. No Velocity/Acceleration-Level IK (Resolved Motion Rate Control)

**Severity: MEDIUM**

The solver only does position-level IK (find q given X). For real-time trajectory tracking, industrial systems use **velocity-level IK**:

```
dq = J⁺ · dx + (I - J⁺J) · q̇₀
```

Computing joint velocities directly from Cartesian velocities. This:

- Naturally handles continuous motion.
- Avoids the repeated convergence overhead.
- Enables singularity-robust damping at the velocity level.
- Feeds directly into the motion controller.

Currently every frame re-solves from scratch with `tracking_local` intent, which is wasteful.

---

### 7. No Workspace Boundary Awareness During Solve

**Severity: MEDIUM**

The reachability atlas is a separate lookup step, not integrated into the solver. An industrial solver would:

- Detect when a target approaches the workspace boundary.
- Automatically reduce step size / increase damping near workspace edges.
- Return a "closest reachable point" when the target is unreachable.
- Provide a distance-to-boundary metric.

---

### 8. Euler Angle Representation for Orientation

**Severity: LOW-MEDIUM**

The `Rotation3` type uses Euler angles throughout. While the orientation error is correctly computed via quaternions, the FK output and target specification use Euler angles. This means:

- **Gimbal lock** at pitch = ±90° in the rotation matrix → Euler extraction (`UrdfChainKinematics.ts:111-129`).
- Interpolation between orientations is wrong (SLERP should be used, not linear Euler interpolation).
- The `orientationDriftRad` function in `qualityMetrics.ts:35-40` computes Euler angle differences which is geometrically incorrect.

---

### 9. No Self-Collision Detection

**Severity: MEDIUM** (for real hardware)

No check for the arm colliding with itself or with the base. Industrial controllers run a simplified collision model (swept spheres or capsules per link) inside the IK loop and reject solutions that collide.

---

### 10. No Warm-Start Jacobian Reuse

**Severity: LOW**

The Jacobian is computed twice per iteration in `evaluateState` — once for diagnostics and once for the actual solve step (`InverseKinematics.ts:177` recomputes what line 310 already computed). In real-time tracking at 100 Hz+, this doubles the FK/Jacobian cost.

---

## Priority Roadmap to Industrial-Grade

| Priority | Item | Impact |
|----------|------|--------|
| **P0** | Closed-form Pieper IK (analytical J1-J3 + J4-J6 decoupling) | 10× faster, all 8 branches, no convergence risk |
| **P0** | Proper singularity-robust damping (Nakamura-Hanafusa) | Eliminates oscillation near singularities |
| **P1** | Use analytical IK as primary, numerical as fallback/refinement | Best of both worlds |
| **P1** | Velocity-level resolved-rate IK for trajectory tracking | Smooth real-time motion |
| **P1** | Configuration continuity / branch locking | No jumps during trajectories |
| **P2** | Replace hand-rolled SVD with proper Golub-Kahan or use math.js/ndarray | Numerical stability |
| **P2** | Quaternion / rotation-matrix representation throughout (drop Euler) | Eliminates gimbal lock artefacts |
| **P2** | Self-collision checking (capsule model) | Hardware safety |
| **P3** | Null-space joint limit avoidance | Smoother limit behaviour |
| **P3** | Workspace boundary integration in solver | Graceful degradation |

---

## Summary

The single biggest improvement would be implementing the **Pieper closed-form solution** for this specific robot geometry. Since the robot has a spherical wrist (J4-J5-J6 axes intersect at a point), this is a textbook decomposition that eliminates the core numerical IK entirely and gives all 8 solutions analytically in microseconds.

The current numerical solver is a competent DLS/LM implementation with good diagnostics, but it is fundamentally limited by being purely iterative for a problem that has closed-form solutions. Layering analytical IK as the primary solver with the existing numerical solver as a refinement pass would bring this system to industrial parity.

---

## Remediation Closure Table (IK V3)

| Finding | Implementation in V3 | Status |
|---|---|---|
| 1. Analytical IK missing | Added `AnalyticalPieperIK.ts` with analytical-first branch generation and deterministic branch IDs | Implemented |
| 2. Singularity handling weak | Added `SingularityHandler.ts`, directional damping scaling, wrist bypass logic, singularity flags in results | Implemented |
| 3. Null-space / limit behavior weak | Added joint-limit barrier gradient in `InverseKinematics.solveDampedLeastSquares` (soft avoidance before hard clamp) | Implemented |
| 4. Configuration continuity missing | Added `ContinuityPolicy.ts`, preferred-branch ranking, branch-lock rejection (`branch_discontinuity`) | Implemented |
| 5. Hand-rolled spectrum path | Replaced primary singular spectrum path with `mathjs` SVD; kept linear fallback only for contingency | Implemented |
| 6. Velocity-level tracking missing | Added resolved-rate step (`stepResolvedRate`) and optional resolved-rate interpolation mode in Cartesian path sampling | Implemented |
| 7. Boundary awareness missing | Extended reachability probe with `boundaryDistanceM` and closest-point projection metadata; fed boundary distance into damping policy | Implemented |
| 8. Euler-heavy orientation metrics | Added `QuaternionMath.ts`; quaternion log-map for IK error and quaternion geodesic drift metric in `qualityMetrics.ts` | Implemented |
| 9. No self-collision checks | Added `CollisionModel.ts` (capsule model) and optional collision rejection in IK pipeline | Implemented |
| 10. Jacobian recompute overhead | `evaluateState` now computes Jacobian once and reuses it for diagnostics + solve step | Implemented |

### Notes
- `AnalyticalPieperIK` is analytical-first orchestration with deterministic wrist branch expansion plus local wrist refinement for robustness on current geometry.
- UI-facing Euler orientation remains for compatibility, but internal IK orientation error and drift metrics are quaternion-based.
