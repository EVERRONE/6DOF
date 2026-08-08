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

### Tool frame

The step from the flange to the tool:

```
T_tcp = T_6 * Translate(xyz) * R_rpy(rpy)
```

Held in `robotModel.ts` as a `ToolFrame`, settable at runtime through
`setToolFrame` and read by FK at every call, so a tool measured mid-session takes
effect immediately — including inside the IK, which solves against this same
chain. `DEFAULT_TOOL_FRAME` is the compiled-in answer and is currently a **bare
flange**, both halves zero.

The two halves answer different questions and are worth keeping apart.

**`xyz` — where the tip is.** Without it "the position" is the flange origin, so
every wrist rotation swings the real tip through an arc nothing in the software
can see. A 100 mm tool turned 10° moves its tip 17 mm while the reported position
does not change at all. It is also what lets J6 move the TCP: on a bare flange
the TCP sits on J6's own axis, so spinning it moves nothing, which is why
`computeWorkspaceBounds` takes a single J6 sample there and the full grid once a
tool is fitted.

**`rpy` — which way it points.** The rotation is applied *after* the translation,
so it cannot move the tip; what it changes is what an orientation *means*. Ask
for an attitude and it is the tool that ends up in it — the flange goes wherever
it must so that the tool does. Without it the tool's axes are assumed to be the
flange's, so "hold the tool level" silently means "hold the flange level", which
is the same thing only if the tool was bolted on perfectly square. This is also
the only place a measured calibration can go: level the base, command the tool to
a known attitude, read the error off an inclinometer, put the difference here.

Three things follow the tool frame and had to be told about it:

| | Why |
|---|---|
| The Jacobian | Already correct — its linear columns are `axis × (p_tcp − p_joint)`, and `p_tcp` now includes the tool |
| `workspaceBounds` cache | Keyed on the tool frame's revision. It was cached on the reasoning that the joint limits are compile-time constants; a 100 mm tool moves the box by 100 mm, and a stale one rejects targets the arm can reach |
| The viewer's axes marker | Parented to frame 6 and now offset by the tool frame, or it draws the axes in one place while the panels report a position somewhere else |

**Not** told about it: the collision model, deliberately. The tool's *shape* is a
separate thing, entered separately — see [Tool geometry](#tool-geometry) below.
The frame moves where the TCP is; the boxes are bolted metal and must not move
with it, or a 150 mm tool would start being checked 300 mm out the moment
somebody calibrated one.

The Tool frame panel edits it in millimetres and degrees, remembers it in
`localStorage`, and prints the `DEFAULT_TOOL_FRAME` literal to paste back here.
Storage is a convenience, not the source of truth: a value that only exists in
one browser is one nobody can review and that differs between two machines
driving the same arm.

> Changing the tool frame invalidates anything measured against the old one. A
> held orientation was captured for a tool pointing a different way, and a
> reported position was measured to a different point. The store releases the
> tool lock and clears the IK status rather than reinterpreting them.

### Tool geometry

A different measurement of the same tool, answering a different question.

| | says | changes |
|---|---|---|
| Tool **frame** | where the tip is | what the arm **reports** |
| Tool **geometry** | how much space it takes up | what the arm **refuses** |

Held in `toolGeometry.ts` as one box — size and centre in flange coordinates,
with the flange face at zero and +Z pointing away from the arm. One box because
the tool is whatever was bolted on this morning and there is no model of it to
load; a bounding box over-reports rather than under-reports, which is the right
direction for a guard.

It hangs off **frame 6, not the TCP**. That is the trap this sits next to: the
tool frame moves the TCP, and if the box followed it, measuring a 150 mm tool
offset would push a 150 mm box out to 300 mm.

Fitting one switches the shoulder, upper arm and elbow pairs back on. They are
disabled in the generated table because the mesh model's last link is a 4 mm stub
that never reaches anything. The forearm stays off whatever is fitted — its boxes
stop about 75 mm past the flange, so the only region where they meet a tool is
the region where they overlap it by construction. The measured tables are in
[HARDWARE.md](HARDWARE.md#self-collision).

### Square with the axes

Held in `axisAlign.ts`. "Put the tool exactly straight along the axes" is not
something jogging can do: a turn jog steps by a fixed amount, so from 1.3° off it
reaches 0.3° or −3.7° and never zero, and the solver has no preference for square
over any other attitude.

Square is a finite set — the 24 attitudes in which every tool axis lies along a
frame axis, generated by screening signed permutations on the determinant (half
of the 48 ways to send three axes to three signed axes are reflections, which no
rigid body can adopt). `nearestAxisAligned` is exhaustive over the 24 and compares
by geodesic distance. Rounding Euler angles instead breaks near gimbal lock;
rounding each column independently need not stay orthogonal.

Two things it reports besides the move:

- **The distance from square**, live. The number an inclinometer would read, and
  nothing else in the app measured it.
- **Where the tool will land**, as axis labels. Square is not the same as
  upright: there are 24 of them and the nearest may not be the one in mind. No
  attitude is further than 62.7992° from the nearest — the covering radius of the
  set, found by hill-climbing rather than quoted — so the move is bounded.

It squares with the **jog frame**, so a fixture taught at an angle gets a tool
square with the fixture. In tool frame the question means nothing and it refuses.

> Below 0.06° it reports square and stops offering the move. That is half a step
> of J6 at 8.889 µsteps/deg, much the coarsest joint on this arm — nothing finer
> can be commanded, and a residual shown without that explanation reads as a
> failure rather than as the machine's resolution.

An existing tool lock is moved onto the squared attitude, using the exact target
rather than the arm's report of where it arrived. Without it the lock still holds
the crooked attitude and the next Cartesian move tips the tool back to it.

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
alignment. Harder to satisfy on this arm: J2 travels 2–86° and J3 2–104°, so not
every orientation is attainable at every point, and holding the tool costs reach
— from the parked pose, roughly 40 mm in +X against 85 mm free.

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
| X | −340 mm | +8 mm |
| Y | −340 mm | +340 mm |
| Z | +23 mm | +435 mm |

Strongly off-centre in X because the arm reaches out along −X from the parked
pose and J1's ±90° cannot bring it round. `CartesianControlPanel` derives its
input ranges from this function rather than hardcoding them.

> These replace an earlier table quoting −203…+70 in X and −115…+138 in Y, which
> was measured when J1 travelled −40…30° and J2 travelled 0…60°. Those limits
> came from a guess; the design values in `config.h` are much wider, and the
> reachable set roughly trebled with them.

**The bounding box is an outer bound, not the reachable set.** A point inside
the box can still be unreachable. The IK result is the authority.

### Singularities

Configurations where the Jacobian loses rank:

1. **Shoulder** — J2 and J3 aligned (fully extended or fully retracted)
2. **Elbow** — arm straight
3. **Wrist** — J4 and J6 axes aligned

The adaptive damping keeps a *single* solve well behaved: near a singularity the
cost stops improving, the damping rises, and the step shortens instead of
exploding. A solve lands somewhere sensible, or reports that it could not.

**Along a path, that is not enough, and the wrist case is not hypothetical on
this arm.**

> **The arm parks in its wrist singularity.** `POST_HOME_ANGLES` puts J5 at 131°,
> which is URDF zero for that joint — exactly where a spherical wrist degenerates.
> Measured at the parked pose, the orientation Jacobian's J4 and J6 columns are
> identical, both `[1, 0, 0]`, and its determinant is 2.5 × 10⁻¹⁷. The wrist
> cannot turn the tool about `wz` at all, and only **J4 + J6** is determined —
> the split between them is free.

Free is the problem. A single solve only has to land somewhere, so an arbitrary
split is harmless. Along a sampled path, where each sample is seeded from the
last, the solver will happily move 44° from J4 into J6 between two samples 2 mm
apart, because that costs nothing in tool pose and buys a 0.001 mm improvement.
The tool pose is correct at both samples and wrong everywhere between them, and
sampling the line more finely does not help — the discontinuity is in joint
space, not in the line.

How fast it decays with distance from the singularity, on a 100 mm linear move
with the tool held:

| J5, URDF | 0° | ±2° | ±5° | ±10° | ±20° | ±30° | ±40° |
|---|---|---|---|---|---|---|---|
| worst step per 2 mm | 44.1° | 20.8° | 8.3° | 4.4° | 1.9° | 1.3° | 1.0° |

### Solving it: walk the line backwards

Five things were tried. Four failed, all measured on the 100 mm move above
against a 44.1° baseline:

| Attempt | Result |
|---------|--------|
| Shorter trust region (`maxStepRad` 0.35 → 0.02) | 53.0° — worse |
| Single seed, no restarts | 44.1° — unchanged |
| Seed bias `μ‖q − anchor‖²` in the step only | 470° at μ=1e-4; no convergence at 1e-3 |
| …with the accept test minimising the same augmented cost | 45.5° at μ=1e-4; 459° at 1e-3 |
| Lock J4 near the singularity and solve on five joints | 463°, and 8 samples became unreachable |
| **Solve the line from its far end** | **0.7°** |

The seed bias fails because the weight that suppresses a free 22° swap is the
same order as the weight that stops the solver reaching the target at all — no
window between them. A weighting cannot fix a rank deficiency that is *exact*
rather than approximate.

Locking a joint fails for a more interesting reason, and it corrected the
diagnosis. With J4 held still the line becomes unrunnable: the tool tilts 25° and
drifts 4.9 mm, and every sample reports unsolved. **So the wrist reconfiguration
is not gratuitous — it is required.** Holding the tool level while J1 rotates
genuinely needs the wrist in a different configuration, and at the parked pose it
is in the wrong one. The 44° jump was real work, not wandering; what was wrong
was only *where it landed* — compressed into a single 2 mm step, because each
sample is solved greedily and that is the cheapest place to put it.

**The far end is 30° clear of the singularity and well conditioned.** Walked
backwards from there, every sample stays near its predecessor and the
reconfiguration never has to happen mid-line at all:

| line from the parked pose | forwards | backwards |
|---|---|---|
| +Y 100 mm | 44.1° | 0.7° |
| +Y 60 mm | 44.6° | 0.4° |
| −Y 80 mm | 81.4° | 0.5° |
| +Y/−Z diagonal | 25.3° | 0.7° |
| −Z 100 mm | 0.9° | 0.9° |

What it costs is that the wrist must already be in that configuration when the
line starts, and from the parked pose it is 90° away. That is reported as
`TrajectorySegment.reconfiguration`, and the caller turns the wrist there first.

**The reconfiguration is free at the tool.** At the singularity J4 and J6 turn
about the same axis, so counter-rotating them is exactly null-space motion —
measured over the whole 90° turn, the tool moves **0.0 mm and tilts 0°**, with no
self-collision anywhere along it. It is a wrist turning in place before the move
starts, which is what an industrial controller does when it inserts a
reconfiguration ahead of a linear path.

`interpolatePiece` runs the forward pass, and only if that jumps does it pay for
a second pass from the far end; the backward path is kept only if it is smooth
*and* complete. `moveAlongLine` checks the reconfiguration for self-collision
along with the path, sends it as its own move, and waits for the arm to stop
before starting the line.

**Still refused, not solved:** a line where the backward pass jumps too.
`findDiscontinuity` flags a step both more than 10× the path's median and more
than 5° absolute — relative rather than absolute, the same way MoveIt's Cartesian
interpolator treats it, because a path where *every* step is large is merely a
fast path. `executeTrajectory` refuses on `Trajectory.discontinuity`, which
aggregates the first jump across all segments.

### Measuring it: how much room is left

`singularity.ts`. Everything above is about *reacting* to a singularity — the
refusal names the joint that jumped, after the fact. This is the cause, as one
number that exists before anything is refused.

`manipulability(q)` takes the smallest singular value of the Jacobian, with the
rotation rows scaled by the arm's reach (0.37 m) so metres and radians are
comparable. Singular values rather than the determinant: the determinant is the
product of all six, so it is small both when one direction is lost and when the
arm is merely slow everywhere, and it cannot say which. Computed as an
eigendecomposition of `JᵀJ` by cyclic Jacobi (`symmetricEigen` in `linalg.ts`) —
Jacobi rather than Cholesky because the interesting input is the one that is
nearly singular, where Cholesky simply fails.

Measured on this arm:

| J5 | 131 (parked) | 133 | 136 | 141 | 151 | 161 | 91 | best anywhere |
|---|---|---|---|---|---|---|---|---|
| worst | **0.0000** | 0.0038 | 0.0095 | 0.0190 | 0.0375 | 0.0554 | 0.0722 | 0.097 |

Read it as: joint motion needed ≈ tool motion ÷ this. At 0.02 a 2 mm Cartesian
step costs about 6° of wrist; at 0.005, about 23°. `NEAR_SINGULAR = 0.02` sits
where that starts being felt — earlier than the path planner refuses anything,
deliberately, since the point is to say so first. 11.8% of poses drawn uniformly
from the joint limits fall below it.

The eigenvector for the smallest value is the joint direction that produces no
tool motion. At the parked pose it comes out as **J4 and J6 exactly opposed**,
which is the singularity stated in joint terms rather than asserted.

### Getting out of one

`planEscape(q, solver)`. There is no free escape, and the reason is worth being
plain about: a singularity is a property of the pose, not of a choice made
getting there. Every configuration that reaches the parked pose has J4 and J6
lined up, so re-solving the same pose cannot help. **The tool has to move.**

What can be kept is the tip. The escape holds position and pays entirely in
attitude, and reports the bill before anything moves. Every joint is tried and
the cheapest wins; on this arm that is reliably J5 — the joint the refusals have
been telling the operator to move by hand.

From the parked pose: **J5 moves 11.5°, the tool tips 9.7°, the tip drifts
0.11 mm**, and the measure goes 0 → 0.0218.

> Kept out of `InverseKinematics` on purpose. Both are properties of a pose
> rather than of a solve, and the solver would have to run an eigendecomposition
> every iteration to use them — for a choice four measured attempts showed it
> cannot make correctly one pose at a time.

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

---

## Work objects

Named frames that taught points are measured in. Everything else in the program
is in the robot's base frame, which makes every taught point a statement about
where a thing was *that day*: nudge the fixture five millimetres and the whole
program is wrong, with nothing to do but teach it again.

`workObject.ts` holds the frame maths, `resolveWaypoint` in `TrajectoryPlanner`
is the one place they are resolved, and everything downstream of planning works
in base coordinates and knows nothing about them.

```
p_base = R(frame.rpy) · p_frame + frame.origin
R_base = R(frame.rpy) · R_frame
```

**Orientations are relative too**, and have to be — turning a fixture 90° should
turn the attitude the tool approaches it at by 90°. Transforming only the
position would leave a re-taught program reaching the right places the wrong way
round.

### Teaching from three points

| | |
|---|---|
| p1 | the origin |
| p2 | anywhere on the +X axis |
| p3 | anywhere in the +Y half of the XY plane |

Z comes from `X̂ × (p3 − p1)`, so the frame is right-handed by construction and p3
only has to be on the correct side of the X line — its distance from it does not
matter. Y is then recovered from Z and X rather than taken from p3, which is what
makes the result orthogonal even though three touched points on a real fixture
never are.

Three points rather than six typed numbers because touching a fixture with the
tool is something an operator can do accurately, and measuring its rotation
against the robot's base with a rule is not.

Refused when p1 and p2 coincide, or when the three points are collinear.
Collinearity is judged relative to the distance from p1, so the same three points
give the same answer in millimetres and in metres.

### What follows a frame, and what does not

> **A waypoint that carries recorded joint angles does not follow its work
> object.** It is replayed from those angles exactly. That is right for a pose
> taught to clear an obstacle and wrong for a point on a fixture, and only the
> operator knows which — `dropJointAngles` forgets them so the point is solved
> from its position instead.

A waypoint naming a work object that no longer exists resolves against the base
frame. That is somewhere real and wrong, so `danglingFrames` reports it and both
deletion and planning say so loudly rather than failing silently.

Frames are remembered in `localStorage` and travel inside a saved path — only
the ones the path actually references, so a file does not accumulate every
fixture the workshop has ever had. Importing merges rather than replaces: two
paths can share a fixture, and loading the second must not delete the first
one's frames.

### Before teaching a frame

**Set the tool frame first.** The three points come from where the arm says the
tool tip is, and on a bare flange that is the flange face — so a frame taught
with an unmeasured tool is offset by however far the real tip sticks out. The
panel says so when the tool frame is still zero.
