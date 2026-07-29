# Kinematics Documentation

**6DOF Robot Arm — Forward and Inverse Kinematics**

---

## Overview

This document describes the kinematics implementation for the 6DOF robot arm:

- Forward Kinematics (FK): joint angles → Cartesian pose
- Inverse Kinematics (IK): Cartesian pose → joint angles
- How the kinematic chain is derived from the URDF
- Usage, validation and troubleshooting

### Files

| File | Responsibility |
|------|----------------|
| `src/kinematics/robotModel.ts` | The kinematic chain and joint limits. Single source of truth. |
| `src/kinematics/linalg.ts` | 4×4 transforms, rotation utilities, Cholesky solver. |
| `src/kinematics/ForwardKinematics.ts` | FK and the analytic geometric Jacobian. |
| `src/kinematics/InverseKinematics.ts` | Damped least squares IK, workspace bounds. |
| `src/kinematics/kinematics.test.ts` | Test suite, including FK-vs-viewer agreement. |

---

## Unit convention

Getting this wrong silently was the cause of several past bugs, so it is stated
explicitly and enforced by the function names:

| Where | Angles | Lengths |
|-------|--------|---------|
| Public API (`solve`, `solvePosition`, `solvePose`) | **degrees** | metres |
| Everything internal (`solveRad`, `jacobianRad`, the solver loop) | **radians** | metres |
| Jacobian entries | per **radian** | metres |

Functions that cross the boundary say so in their doc comment. There is no
implicit conversion anywhere else.

---

## Coordinate systems

### Joint space

- Representation: 6 joint angles `[J1, J2, J3, J4, J5, J6]`
- Units: degrees at the API boundary
- Range: `JOINT_LIMITS_DEG` in `robotModel.ts`, mirroring `firmware/config.h`

### Cartesian space

- Representation: position (X, Y, Z) + orientation (roll, pitch, yaw)
- Units: metres, radians
- Origin: robot base centre (`linkB`)
- Z is vertical (up), X forward, Y follows the right-hand rule

Orientation uses the **URDF fixed-axis rpy convention**:

```
R = Rz(yaw) · Ry(pitch) · Rx(roll)
```

which is what `THREE.Euler` expresses with order `'ZYX'`. The 3D viewer uses
that same order, so the viewer and the kinematics agree by construction.

---

## The kinematic chain

**No DH parameters are used.** The transforms are composed straight from the
URDF joint origins. For each joint *i*:

```
T_i = Translate(origin.xyz) · Rz(yaw) · Ry(pitch) · Rx(roll) · Rz(q_i)
      └────────── fixed offset from the parent frame ──────────┘  └ joint ┘
```

and the pose of frame *i* in base coordinates is the running product
`T_0→i = T_1 · T_2 · … · T_i`. Every joint of this arm rotates about its own
local Z axis (URDF `axis="0 0 1"`), which is why no per-joint axis handling is
needed.

### Why not DH

A DH table cannot be read off URDF origins component by component; converting
between the two requires a proper frame-alignment algorithm. An earlier version
of this code guessed at it (`a = sqrt(x² + y²)`, `d = z`, taking a Y component
as a link length) and produced a chain that disagreed with the actual robot by
**330–500 mm** — on an arm whose total reach is about 370 mm. IK therefore
solved for a robot that did not exist.

Composing the URDF transforms directly is exact, shorter, and cannot drift out
of sync with the 3D viewer. The test suite asserts that agreement to within
1e-9 m over 500 random poses.

### Chain

```
linkB (base plate)
  ↓ J1  link0_joint   xyz=( 0.0,      0.0,     0.08   )  rpy=( 0,       0,      0      )
link0 (base wall)
  ↓ J2  Joint2        xyz=(-0.0375,   0.02,    0.05595)  rpy=(-1.5708,  0,      0      )
link1 (arm 1)
  ↓ J3  Joint3        xyz=( 0.00027, -0.16,    0.016  )  rpy=(-3.14159, 0,      0      )
link2 (arm 2 mount)
  ↓ J4  link3_joint   xyz=(-0.035,    0.0151,  0.0364 )  rpy=(-1.5708,  0,      1.5708 )
link3 (rotation arm)
  ↓ J5  Joint5        xyz=( 0.0,     -0.01002, 0.10323)  rpy=(-1.5708,  1.5708, 0      )
link4 (J6 housing)
  ↓ J6  Joint6        xyz=(-0.02677,  0.0,     0.00994)  rpy=( 1.5708,  0,     -1.5708 )
link5 (end effector)
```

### Tool centre point

`TOOL_OFFSET` in `robotModel.ts` is the TCP expressed in frame 6. It is
currently `(0, 0, 0)`, which puts the TCP at the origin of frame 6 — exactly
where the 3D viewer attaches its end-effector axes marker. Set it once the real
gripper geometry is known, and FK, IK and the viewer all follow.

---

## Joint limits

The limits come from `firmware/config.h` (`JOINT_MIN` / `JOINT_MAX`), because
those are the mechanically valid hard stops and the firmware enforces them
regardless of what the app asks for.

| Joint | Min | Max | Reduction |
|-------|-----|-----|-----------|
| J1 | −40° | 30° | 6.3 : 1 |
| J2 | 0° | 60° | 25 : 1 |
| J3 | 0° | 70° | 6.3 : 1 |
| J4 | 0° | 274° | 3.7 : 1 |
| J5 | 0° | 280° | 2 : 1 |
| J6 | −360° | 360° | 1 : 1 |

> **Keep these in step with `firmware/config.h`.** If the solver works in a
> different box than the firmware, the firmware silently clamps the solution and
> the arm ends up somewhere the solver never asked for. `kinematics.test.ts`
> asserts the exact values so a drift fails the suite.

The URDF's own limits (±160/±74/±120/±143/±143) are **not** used: they are
symmetric placeholders, and the documented post-homing pose `J5 = 220°` falls
outside them.

Note also that J6 must never be given a zero-width range. The URDF marks it
`continuous` with a `limit` block of `{lower: 0, upper: 0}`; an earlier version
accepted that literally and froze the joint, costing the solver a degree of
freedom.

---

## Forward kinematics

### Usage

```typescript
import { ForwardKinematics } from './kinematics/ForwardKinematics';

// Joint angles in DEGREES
const result = ForwardKinematics.solve([0, 20, 30, 0, 0, 0]);

if (result.success) {
  const { position, rotation } = result.endEffectorPose;
  console.log(`X=${position.x} Y=${position.y} Z=${position.z} m`);
  console.log(`roll=${rotation.roll} pitch=${rotation.pitch} yaw=${rotation.yaw} rad`);
  // result.jointTransforms[i] is the 4x4 pose of joint frame i
}
```

Convenience entry points:

```typescript
ForwardKinematics.position(anglesDeg);      // Vector3, TCP position only
ForwardKinematics.jointOrigins(anglesDeg);  // every frame origin + the TCP
ForwardKinematics.reachAt(anglesDeg);       // distance from the base origin
ForwardKinematics.toolToBase(anglesDeg, p); // tool coords -> base coords
ForwardKinematics.solveRad(anglesRad);      // radians, no error wrapping
```

`solve` validates its input and returns `success: false` with a reason for a
wrong array length or a non-finite value, rather than quietly returning the
origin.

### Performance

Pure JavaScript, no external math libraries. One FK evaluation is six 4×4
multiplications; the suite runs several thousand per second in jsdom.

---

## The Jacobian

`ForwardKinematics.jacobianRad(q)` returns the **analytic geometric Jacobian**,
a 6×6 matrix in base coordinates:

- rows 0–2: joint velocity → linear TCP velocity (m/rad)
- rows 3–5: joint velocity → angular TCP velocity (rad/rad)

For a revolute joint with world-frame axis `z_i` through point `p_i`:

```
linear  column i = z_i × (p_tcp − p_i)
angular column i = z_i
```

Both `z_i` and `p_i` are read straight off the joint frame, because `Rz(q_i)`
changes neither the frame's Z axis nor its origin.

This costs a single FK pass instead of the seven a numerical difference needs,
and carries no truncation noise. The test suite checks it against a central
difference to within 1e-7.

> The angular rows are a genuine angular velocity, not a rate of change of Euler
> angles. Those are different quantities, and Euler rates are singular at
> gimbal lock.

---

## Inverse kinematics

### Method

Damped least squares with Levenberg-Marquardt damping. Each iteration solves

```
(JᵀJ + λ²I) Δq = Jᵀe
```

with four properties that matter:

1. **Scale-aware damping.** `λ² = dampingRel · max(diag(JᵀJ)) + floor`, so the
   damping is always proportional to the Jacobian's own magnitude. A fixed λ²
   cannot work: with the Jacobian expressed per degree, a λ² tuned for radians
   ended up 7× larger than the largest signal term and the steps collapsed to
   nothing.
2. **Cholesky factorisation** of the normal equations, with failure reported so
   the caller can raise the damping and retry. `JᵀJ + λ²I` is symmetric positive
   definite for λ > 0 but is *not* diagonally dominant here, so an iterative
   Gauss-Seidel sweep is not guaranteed to converge on it.
3. **Joint limits inside the solve.** A joint that the step would push further
   past a limit is locked out and the system re-solved so the remaining joints
   compensate. Clamping the step afterwards instead just discards the
   correction and stalls the solver against the limit.
4. **Adaptive trust region.** A step that increases the cost is rejected and
   retried with more damping; a step that works reduces the damping. Step size
   is capped at `maxStepRad` because a Jacobian is only a local linearisation.

Orientation error is the matrix logarithm of `R_target · R_currentᵀ` — the
rotation that still has to happen, as an axis-angle vector. Differencing Euler
triples, as an earlier version did, wraps at ±π and breaks down at gimbal lock.

If a seed does not converge, the solver restarts from the next one:
the caller's pose first (so nearby solutions win and the arm does not
reconfigure unnecessarily), then the home pose, the mid-range pose, and
deterministic low-discrepancy samples. Deterministic means a given target
always yields the same solution.

### Configuration

```typescript
import { InverseKinematics, DEFAULT_IK_OPTIONS } from './kinematics/InverseKinematics';

const ikSolver = new InverseKinematics({
  maxIterations: 150,          // per seed attempt
  positionTolerance: 0.0005,   // 0.5 mm
  orientationTolerance: 0.0087,// 0.5 deg, pose solves only
  orientationScale: 0.05,      // 1 rad counts like 50 mm when minimising both
  maxStepRad: 0.35,            // ~20 deg per iteration
  maxSeeds: 6
});
```

There is no `dampingFactor` to tune — the damping is adaptive.

### Usage

```typescript
// Position only, orientation free
const result = ikSolver.solvePosition(
  { x: -0.12, y: -0.01, z: 0.22 },  // metres
  currentAnglesDeg                   // seed, degrees
);

if (result.success) {
  console.log('Joint angles (deg):', result.jointAngles);
  console.log('Iterations:', result.iterations);
  console.log('Residual:', result.residualError * 1000, 'mm');
} else {
  // jointAngles still holds the best pose found, always inside the limits
  console.error(result.error);
}

// Position and orientation
const posed = ikSolver.solvePose(
  { position: { x: -0.12, y: -0.01, z: 0.22 }, rotation: { roll: 0, pitch: 0, yaw: 0 } },
  currentAnglesDeg
);
```

`result.jointAngles` is in **degrees** and is **always inside the mechanical
limits**, whether or not the solve converged.

### Position vs pose

**`solvePosition`** — converges more easily, leaves orientation free. Use for
pick-and-place, drawing, general positioning.

**`solvePose`** — constrains all six degrees of freedom. Use for tool
alignment. Harder to satisfy on this arm: with J2 limited to 0–60° and J3 to
0–70°, not every orientation is attainable at every point.

### Measured behaviour

Over 400 targets generated by FK from random poses inside the joint limits, so
every target is reachable by construction:

| Metric | Result |
|--------|--------|
| Position-only convergence | > 99% |
| Full-pose convergence | > 90% |
| Worst position residual on success | < 0.5 mm |
| Solutions outside joint limits | 0 |

These are asserted in `kinematics.test.ts`, so a regression fails the suite.

---

## Workspace

`computeWorkspaceBounds()` samples the joint space and returns the axis-aligned
outer bound of the reachable TCP positions. Measured with the firmware joint
limits:

| Axis | Min | Max |
|------|-----|-----|
| X | −203 mm | +70 mm |
| Y | −115 mm | +138 mm |
| Z | +146 mm | +384 mm |

The workspace is small and strongly off-centre because J1 only travels −40…30°
and J2 only 0…60°. `CartesianControlPanel` derives its input ranges from this
function rather than hardcoding them.

**The bounding box is an outer bound, not the reachable set.** A point inside
the box can still be unreachable. The IK result is the authority.

### Singularities

Configurations where the Jacobian loses rank:

1. **Shoulder** — J2 and J3 aligned (fully extended or fully retracted)
2. **Elbow** — arm straight
3. **Wrist** — J4 and J6 axes aligned (J5 at either end of its travel)

The adaptive damping handles these: near a singularity the cost stops improving,
the damping rises, and the step shortens instead of exploding. The solver will
not produce a wild joint jump at a singularity, though it may not reach a target
that requires passing exactly through one.

---

## Validation

```bash
cd robot-arm-control
CI=true npx react-scripts test --watchAll=false --testPathPattern=kinematics
```

The suite covers:

- **FK vs the 3D viewer** — rebuilds the chain with nested `THREE.Group` nodes
  the way `RobotModel3D` does and compares, over 500 random poses. This is the
  invariant that the old DH implementation violated, and it is the single most
  useful test in the file.
- **Jacobian vs central difference** — position and angular rows separately.
- **IK convergence rate, residuals, and limit compliance** on reachable targets.
- **Round trip** — `FK(IK(p)) ≈ p`.
- **Edge cases** — target already reached, unreachable target, non-finite input,
  seed outside the joint limits, gimbal-lock orientations.
- **Rotation utilities** — rpy round trip, and `rotationLog` near 0 and near
  180°, where `acos` loses half its precision (hence the `atan2` formulation).

### Manual validation on the arm

1. Home the robot and read the reported joint angles.
2. Read the XYZ shown in the StatusBar.
3. Type the same XYZ into the Cartesian panel and press *Move to Position*.
4. The arm should barely move; IK should report a residual well under 1 mm.

If step 4 moves the arm significantly, the reported joint angles do not match
the physical pose — check the homing bookkeeping in the firmware before
suspecting the kinematics.

---

## Usage in the web application

FK runs whenever a position report arrives:

```typescript
// robotStore.ts
manager.onMessage((msg) => {
  if (msg.type === 'POS') {
    set({ currentAngles: msg.data });
    get().updateCurrentPosition(); // FK
  }
});
```

The Cartesian panel shows the current XYZ, takes a target in mm, calls
`moveToPosition` (which runs IK and sends a `J` command), and reports the IK
outcome including the residual.

---

## Troubleshooting

### IK reports "target not reachable within tolerance"

The message includes the best residual found. Check in this order:

1. Is the target inside the workspace table above? Remember it is an outer
   bound — a point inside it can still be out of reach.
2. Is the seed sensible? Passing the current pose helps, though the solver will
   restart from other seeds on its own.
3. Is the orientation over-constraining it? Try `solvePosition` instead of
   `solvePose`.
4. Raise `positionTolerance` if 0.5 mm is stricter than the arm's mechanical
   repeatability warrants.

Raising `maxIterations` rarely helps: when a seed stops improving the solver
abandons it deliberately and restarts, which is more productive than grinding.

### FK position does not match the real arm

The kinematics agree with the URDF and the 3D viewer — the test suite proves
that. So a mismatch against the physical arm points at one of:

1. **Position bookkeeping in the firmware.** Verify the reported angles
   actually correspond to the physical pose after homing.
2. **Stepper calibration** — `USTEPS_PER_DEG` in `config.h`.
3. **The URDF itself** — if a link length in `URDF.md` is wrong, FK inherits it.
4. **Mechanical backlash**, which no amount of modelling fixes.

### IK picks an unexpected configuration

Position-only IK is redundant on a 6-joint arm — many solutions exist. Pass the
current pose as the seed so the nearest solution wins, or constrain orientation
with `solvePose`.

---

## References

### Books

- Craig, J. J. (2005). *Introduction to Robotics: Mechanics and Control* (3rd ed.)
- Siciliano, B., & Khatib, O. (2016). *Springer Handbook of Robotics* (2nd ed.)
- Lynch, K. M., & Park, F. C. (2017). *Modern Robotics: Mechanics, Planning, and Control*

### Papers

- Nakamura, Y., & Hanafusa, H. (1986). "Inverse Kinematic Solutions With Singularity Robustness for Robot Manipulator Control"
- Buss, S. R. (2004). "Introduction to Inverse Kinematics with Jacobian Transpose, Pseudoinverse and Damped Least Squares methods"
- Wampler, C. W. (1986). "Manipulator Inverse Kinematic Solutions Based on Vector Formulations and Damped Least-Squares Methods"

### Online

- [Modern Robotics (Northwestern)](http://modernrobotics.org/)
- [ROS MoveIt! kinematics](https://moveit.ai/documentation/)

---

**See also:**

- [URDF.md](../URDF.md) — robot structure definition
- [SERIAL_PROTOCOL.md](SERIAL_PROTOCOL.md) — how joint targets reach the firmware
- `firmware/config.h` — joint limits and stepper calibration
