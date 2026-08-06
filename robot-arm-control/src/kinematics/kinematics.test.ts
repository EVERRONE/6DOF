// Kinematics test suite.
//
// The headline test is "FK agrees with the 3D viewer": it rebuilds the joint
// hierarchy with THREE.js exactly the way RobotModel3D does and checks that the
// analytic chain lands in the same place. That invariant is what the previous
// DH-based implementation violated by 330-500 mm, and it is the reason IK
// pointed at a robot that did not exist.

import * as THREE from 'three';
import { ForwardKinematics } from './ForwardKinematics';
import { InverseKinematics, computeWorkspaceBounds } from './InverseKinematics';
import {
  HOME_POSE_DEG,
  JOINT_LIMITS_DEG,
  NUM_JOINTS,
  ROBOT_JOINTS,
  degToRad,
  isWithinLimitsDeg,
  URDF_DIRECTION,
  logicalToUrdfRad,
  urdfToLogicalRad
} from './robotModel';
import { matrixToRpy, rotationLog, rpyToMatrix, solveSPD, transpose3, multiply3 } from './linalg';
import { Vector3 } from './types';

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

/** Deterministic pseudo-random generator so failures are reproducible. */
function makeRng(seed: number): () => number {
  let s = seed >>> 0;
  return () => {
    s = (Math.imul(s, 1664525) + 1013904223) >>> 0;
    return s / 4294967296;
  };
}

/** Uniform random pose inside the mechanical joint limits, in degrees. */
function randomPoseDeg(rng: () => number): number[] {
  return ROBOT_JOINTS.map(j => j.limitDeg.min + rng() * (j.limitDeg.max - j.limitDeg.min));
}

function dist(a: Vector3, b: Vector3): number {
  return Math.hypot(a.x - b.x, a.y - b.y, a.z - b.z);
}

/**
 * Build the arm out of nested THREE.Group nodes the same way
 * RobotModel3D.buildModel + updatePose do, and return the world transform of
 * the last joint frame - which is where the viewer attaches its end-effector
 * marker.
 */
function viewerChain(anglesDeg: number[]): { position: Vector3; quaternion: THREE.Quaternion } {
  const root = new THREE.Group();
  let parent: THREE.Object3D = root;

  // Mirrors RobotModel3D.updatePose, including its conversion out of firmware
  // angles into the URDF's convention. The point of this whole comparison is
  // that the viewer and the solver build the same chain, so the conversion has
  // to appear on both sides or the test stops checking anything real.
  const anglesUrdf = logicalToUrdfRad(degToRad(anglesDeg));

  for (let i = 0; i < NUM_JOINTS; i++) {
    const origin = ROBOT_JOINTS[i].origin;
    const group = new THREE.Group();

    group.position.set(origin.xyz.x, origin.xyz.y, origin.xyz.z);

    // URDF rpy = fixed-axis XYZ = THREE.Euler order 'ZYX'
    const urdfQuat = new THREE.Quaternion().setFromEuler(
      new THREE.Euler(origin.rpy.roll, origin.rpy.pitch, origin.rpy.yaw, 'ZYX')
    );
    const jointQuat = new THREE.Quaternion().setFromAxisAngle(
      new THREE.Vector3(0, 0, 1),
      anglesUrdf[i]
    );
    group.setRotationFromQuaternion(
      new THREE.Quaternion().multiplyQuaternions(urdfQuat, jointQuat)
    );

    parent.add(group);
    parent = group;
  }

  root.updateMatrixWorld(true);

  const position = new THREE.Vector3();
  const quaternion = new THREE.Quaternion();
  parent.getWorldPosition(position);
  parent.getWorldQuaternion(quaternion);

  return { position: { x: position.x, y: position.y, z: position.z }, quaternion };
}

// ---------------------------------------------------------------------------
// Robot model
// ---------------------------------------------------------------------------

describe('robot model', () => {
  it('has six joints with finite, ordered limits', () => {
    expect(ROBOT_JOINTS).toHaveLength(NUM_JOINTS);
    ROBOT_JOINTS.forEach(joint => {
      expect(Number.isFinite(joint.limitDeg.min)).toBe(true);
      expect(Number.isFinite(joint.limitDeg.max)).toBe(true);
      expect(joint.limitDeg.max).toBeGreaterThan(joint.limitDeg.min);
    });
  });

  it('gives J6 a usable range instead of freezing it', () => {
    // Regression: the old chain gave the continuous joint J6 a limit of
    // {lower: 0, upper: 0}, and getJointLimits() accepted it, so the solver
    // silently lost a degree of freedom.
    const j6 = ROBOT_JOINTS[5];
    expect(j6.limitDeg.max - j6.limitDeg.min).toBeGreaterThanOrEqual(360);
  });

  it('matches the joint limits the firmware enforces', () => {
    // These must stay in step with firmware/config.h JOINT_MIN / JOINT_MAX.
    // If the solver works in a different box, the firmware clamps the solution
    // and the arm goes somewhere the solver never asked for.
    expect(JOINT_LIMITS_DEG.min).toEqual([-90, 2, 2, 2, 2, -360]);
    expect(JOINT_LIMITS_DEG.max).toEqual([90, 86, 104, 332, 222, 360]);
  });

  it('places the documented home pose inside the limits', () => {
    expect(isWithinLimitsDeg(HOME_POSE_DEG)).toBe(true);
  });
});

// ---------------------------------------------------------------------------
// Forward kinematics
// ---------------------------------------------------------------------------

describe('firmware angles vs URDF angles', () => {
  it('round-trips', () => {
    const q = degToRad([12, 33, 41.25, 100, 140, -90]);
    const back = urdfToLogicalRad(logicalToUrdfRad(q));
    back.forEach((v, i) => expect(v).toBeCloseTo(q[i], 12));
  });

  it('puts the URDF at zero when the arm is parked', () => {
    // The anchor the whole mapping rests on: after homing, the arm sits at
    // HOME_POSE_DEG and the model is at its own zero. Break this and the 3D
    // view drifts away from the machine again.
    logicalToUrdfRad(degToRad(HOME_POSE_DEG)).forEach(v =>
      expect(v).toBeCloseTo(0, 12)
    );
  });

  it('draws the parked arm upright, not collapsed on the floor', () => {
    // The symptom that started this: the viewer drew the arm flat while the real
    // one stood up, because firmware angles were fed to the URDF chain raw.
    // At the parked pose the upper arm is vertical and the forearm runs out
    // horizontally, which is a claim about millimetres rather than convention.
    const p = ForwardKinematics.position(HOME_POSE_DEG);
    const origins = ForwardKinematics.jointOrigins(HOME_POSE_DEG);

    expect(p.z).toBeGreaterThan(0.28); // TCP well above the base
    expect(Math.hypot(p.x, p.y)).toBeGreaterThan(0.15); // and reaching outward

    // J3 carries the elbow: it must be up, not down at base height.
    expect(origins[2].z).toBeGreaterThan(0.25);

    // The forearm, J4 to the TCP, stays level to within a millimetre.
    expect(Math.abs(origins[3].z - p.z)).toBeLessThan(0.001);
  });

  it('records the per-joint signs, which no test here can verify', () => {
    // Deliberately a change detector, and worth saying why.
    //
    // Every other test in this file compares FK against the viewer, and both
    // read URDF_DIRECTION, so they agree with each other whatever it says -
    // setting it to all +1 leaves this whole suite green. The offset is
    // different: the parked pose pins it, and the two tests above catch it.
    //
    // The signs can only be checked against the machine: jog one joint and see
    // whether the model turns the same way. Until each has been confirmed that
    // way, this line is the record of an assumption inherited from an earlier
    // calibration of this arm, not a verified fact. See docs/HARDWARE.md.
    expect(URDF_DIRECTION).toEqual([1, -1, 1, -1, 1, -1]);
  });

  it('sends a joint the same way the arm does', () => {
    // A positive firmware step must not move the model backwards. Checked
    // through the chain rather than on the mapping, since that is where a
    // wrong sign would actually bite.
    const base = [...HOME_POSE_DEG];
    const nudged = [...base];
    nudged[1] += 5; // J2 up from the parked pose

    const before = ForwardKinematics.position(base);
    const after = ForwardKinematics.position(nudged);
    expect(dist(before, after)).toBeGreaterThan(0.005);
  });
});

describe('forward kinematics', () => {
  it('agrees with the 3D viewer chain across the whole joint range', () => {
    const rng = makeRng(20260729);
    let worstPos = 0;
    let worstAngle = 0;

    for (let trial = 0; trial < 500; trial++) {
      const q = trial === 0 ? HOME_POSE_DEG : randomPoseDeg(rng);

      const fk = ForwardKinematics.solveRad(degToRad(q));
      const viewer = viewerChain(q);

      worstPos = Math.max(worstPos, dist(fk.position, viewer.position));

      // Compare orientation as a rotation, not as Euler triples.
      const viewerMatrix = new THREE.Matrix4().makeRotationFromQuaternion(viewer.quaternion);
      const e = viewerMatrix.elements; // column-major
      const Rviewer = [
        [e[0], e[4], e[8]],
        [e[1], e[5], e[9]],
        [e[2], e[6], e[10]]
      ];
      const angle = Math.hypot(
        ...Object.values(rotationLog(multiply3(Rviewer, transpose3(fk.rotation))))
      );
      worstAngle = Math.max(worstAngle, angle);
    }

    // Sub-micrometre and sub-microradian: pure floating point noise.
    expect(worstPos).toBeLessThan(1e-9);
    expect(worstAngle).toBeLessThan(1e-9);
  });

  it('returns the pose of every joint frame', () => {
    const result = ForwardKinematics.solve(HOME_POSE_DEG);
    expect(result.success).toBe(true);
    expect(result.jointTransforms).toHaveLength(NUM_JOINTS);

    // Frame 1 sits at the J1 origin, straight above the base.
    const frame1 = result.jointTransforms[0];
    expect(frame1[0][3]).toBeCloseTo(0, 12);
    expect(frame1[1][3]).toBeCloseTo(0, 12);
    expect(frame1[2][3]).toBeCloseTo(0.08, 12);
  });

  it('reports a plausible reach at the home pose', () => {
    const p = ForwardKinematics.position(HOME_POSE_DEG);
    const reach = Math.hypot(p.x, p.y, p.z);
    // Link lengths sum to roughly 0.4 m, so anything outside 0.05-0.45 m is a
    // sign the chain has been mis-assembled.
    expect(reach).toBeGreaterThan(0.05);
    expect(reach).toBeLessThan(0.45);
  });

  it('rejects malformed input instead of returning a silent zero pose', () => {
    expect(ForwardKinematics.solve([0, 0, 0]).success).toBe(false);
    expect(ForwardKinematics.solve([0, 0, 0, 0, 0, NaN]).success).toBe(false);
  });

  it('is deterministic', () => {
    const a = ForwardKinematics.position(HOME_POSE_DEG);
    const b = ForwardKinematics.position(HOME_POSE_DEG);
    expect(a).toEqual(b);
  });
});

// ---------------------------------------------------------------------------
// Jacobian
// ---------------------------------------------------------------------------

describe('geometric jacobian', () => {
  it('matches a central-difference approximation of the position rows', () => {
    const rng = makeRng(4242);
    const h = 1e-6;
    let worst = 0;

    for (let trial = 0; trial < 100; trial++) {
      const q = degToRad(randomPoseDeg(rng));
      const J = ForwardKinematics.jacobianRad(q);

      for (let j = 0; j < NUM_JOINTS; j++) {
        const qp = [...q];
        const qm = [...q];
        qp[j] += h;
        qm[j] -= h;

        const pp = ForwardKinematics.solveRad(qp).position;
        const pm = ForwardKinematics.solveRad(qm).position;

        worst = Math.max(
          worst,
          Math.abs(J[0][j] - (pp.x - pm.x) / (2 * h)),
          Math.abs(J[1][j] - (pp.y - pm.y) / (2 * h)),
          Math.abs(J[2][j] - (pp.z - pm.z) / (2 * h))
        );
      }
    }

    expect(worst).toBeLessThan(1e-7);
  });

  it('matches a central-difference approximation of the angular rows', () => {
    const rng = makeRng(1337);
    const h = 1e-6;
    let worst = 0;

    for (let trial = 0; trial < 100; trial++) {
      const q = degToRad(randomPoseDeg(rng));
      const J = ForwardKinematics.jacobianRad(q);

      for (let j = 0; j < NUM_JOINTS; j++) {
        const qp = [...q];
        const qm = [...q];
        qp[j] += h;
        qm[j] -= h;

        const Rp = ForwardKinematics.solveRad(qp).rotation;
        const Rm = ForwardKinematics.solveRad(qm).rotation;

        // Angular velocity from the rotation increment, not from Euler angles.
        const w = rotationLog(multiply3(Rp, transpose3(Rm)));
        worst = Math.max(
          worst,
          Math.abs(J[3][j] - w.x / (2 * h)),
          Math.abs(J[4][j] - w.y / (2 * h)),
          Math.abs(J[5][j] - w.z / (2 * h))
        );
      }
    }

    expect(worst).toBeLessThan(1e-6);
  });

  it('gives J1 a purely vertical rotation axis', () => {
    // J1 spins about the base Z axis whatever the rest of the arm does.
    const rng = makeRng(99);
    for (let trial = 0; trial < 20; trial++) {
      const J = ForwardKinematics.jacobianRad(degToRad(randomPoseDeg(rng)));
      expect(J[3][0]).toBeCloseTo(0, 12);
      expect(J[4][0]).toBeCloseTo(0, 12);
      expect(Math.abs(J[5][0])).toBeCloseTo(1, 12);
    }
  });
});

// ---------------------------------------------------------------------------
// Linear algebra
// ---------------------------------------------------------------------------

describe('linear algebra', () => {
  it('solves a symmetric positive definite system', () => {
    const A = [
      [4, 1, 0],
      [1, 3, 1],
      [0, 1, 2]
    ];
    const x = solveSPD(A, [1, 2, 3]);
    expect(x).not.toBeNull();

    const b = (A as number[][]).map(row => row.reduce((s, v, i) => s + v * (x as number[])[i], 0));
    expect(b[0]).toBeCloseTo(1, 10);
    expect(b[1]).toBeCloseTo(2, 10);
    expect(b[2]).toBeCloseTo(3, 10);
  });

  it('reports failure rather than returning garbage for a non-definite matrix', () => {
    expect(
      solveSPD(
        [
          [0, 0],
          [0, 0]
        ],
        [1, 1]
      )
    ).toBeNull();
  });

  it('round-trips rpy through a rotation matrix', () => {
    const rng = makeRng(7);
    for (let trial = 0; trial < 200; trial++) {
      const rpy = {
        roll: (rng() - 0.5) * 2 * Math.PI,
        pitch: (rng() - 0.5) * Math.PI * 0.98, // stay off gimbal lock
        yaw: (rng() - 0.5) * 2 * Math.PI
      };
      const back = matrixToRpy(rpyToMatrix(rpy));
      expect(back.roll).toBeCloseTo(rpy.roll, 9);
      expect(back.pitch).toBeCloseTo(rpy.pitch, 9);
      expect(back.yaw).toBeCloseTo(rpy.yaw, 9);
    }
  });

  it('recovers the axis and angle of a rotation, including near 180 degrees', () => {
    const cases = [
      { axis: [0, 0, 1], angle: 0.4 },
      { axis: [1, 0, 0], angle: Math.PI - 1e-7 },
      { axis: [0, 1, 0], angle: Math.PI },
      { axis: [1, 1, 1], angle: Math.PI - 1e-9 },
      { axis: [0, 0, 1], angle: 1e-10 }
    ];

    for (const { axis, angle } of cases) {
      const n = Math.hypot(axis[0], axis[1], axis[2]);
      const a = new THREE.Vector3(axis[0] / n, axis[1] / n, axis[2] / n);
      const m = new THREE.Matrix4().makeRotationAxis(a, angle);
      const e = m.elements;
      const R = [
        [e[0], e[4], e[8]],
        [e[1], e[5], e[9]],
        [e[2], e[6], e[10]]
      ];

      const w = rotationLog(R);
      const recovered = Math.hypot(w.x, w.y, w.z);
      expect(recovered).toBeCloseTo(angle, 6);

      if (angle > 1e-6) {
        // Axis direction, allowing the sign flip that is inherent at exactly pi.
        const dot = (w.x * a.x + w.y * a.y + w.z * a.z) / recovered;
        expect(Math.abs(dot)).toBeCloseTo(1, 5);
      }
    }
  });
});

// ---------------------------------------------------------------------------
// Inverse kinematics
// ---------------------------------------------------------------------------

describe('inverse kinematics - position', () => {
  const ik = new InverseKinematics();

  it('solves targets that are reachable by construction', () => {
    const rng = makeRng(20260101);
    const total = 400;
    let solved = 0;
    let worstError = 0;

    for (let trial = 0; trial < total; trial++) {
      const truth = randomPoseDeg(rng);
      const target = ForwardKinematics.position(truth);

      const result = ik.solvePosition(target, HOME_POSE_DEG);

      if (result.success) {
        solved++;
        const achieved = ForwardKinematics.position(result.jointAngles);
        worstError = Math.max(worstError, dist(achieved, target));
      }
    }

    // Every target came out of FK inside the joint limits, so every one of them
    // is reachable. The old solver managed 8% here.
    expect(solved / total).toBeGreaterThan(0.99);
    expect(worstError).toBeLessThan(ik.getConfig().positionTolerance);
  });

  it('always returns joint angles inside the mechanical limits', () => {
    const rng = makeRng(555);

    for (let trial = 0; trial < 200; trial++) {
      const target = ForwardKinematics.position(randomPoseDeg(rng));
      const result = ik.solvePosition(target, HOME_POSE_DEG);
      // True whether or not it converged - a solution outside the limits would
      // be silently clamped by the firmware.
      expect(isWithinLimitsDeg(result.jointAngles, 1e-6)).toBe(true);
    }
  });

  it('round-trips through forward kinematics', () => {
    const rng = makeRng(31415);

    for (let trial = 0; trial < 100; trial++) {
      const target = ForwardKinematics.position(randomPoseDeg(rng));
      const result = ik.solvePosition(target, HOME_POSE_DEG);
      if (!result.success) continue;

      const achieved = ForwardKinematics.position(result.jointAngles);
      expect(dist(achieved, target)).toBeLessThan(0.0005);
    }
  });

  it('holds still when the target is already the current position', () => {
    const target = ForwardKinematics.position(HOME_POSE_DEG);
    const result = ik.solvePosition(target, HOME_POSE_DEG);

    expect(result.success).toBe(true);
    result.jointAngles.forEach((angle, i) => {
      expect(angle).toBeCloseTo(HOME_POSE_DEG[i], 6);
    });
  });

  it('prefers a nearby solution over a large reconfiguration', () => {
    // A 20 mm nudge should not swing a joint by tens of degrees.
    const start = HOME_POSE_DEG;
    const p = ForwardKinematics.position(start);
    const target = { x: p.x + 0.02, y: p.y, z: p.z };

    const result = ik.solvePosition(target, start);
    expect(result.success).toBe(true);

    const maxJointChange = Math.max(
      ...result.jointAngles.map((angle, i) => Math.abs(angle - start[i]))
    );
    expect(maxJointChange).toBeLessThan(30);
  });

  it('fails cleanly and reports a residual for an unreachable target', () => {
    const result = ik.solvePosition({ x: 5, y: 5, z: 5 }, HOME_POSE_DEG);

    expect(result.success).toBe(false);
    expect(result.error).toMatch(/not reachable/i);
    expect(result.residualError).toBeGreaterThan(0);
    expect(isWithinLimitsDeg(result.jointAngles, 1e-6)).toBe(true);
  });

  it('rejects a non-finite target', () => {
    expect(ik.solvePosition({ x: NaN, y: 0, z: 0 }, HOME_POSE_DEG).success).toBe(false);
  });

  it('works without a seed', () => {
    const target = ForwardKinematics.position([-10, 20, 40, 100, 200, 0]);
    expect(ik.solvePosition(target).success).toBe(true);
  });

  it('recovers from a seed sitting outside the joint limits', () => {
    // The old solver seeded from the reported pose and clamped mid-iteration,
    // which teleported J5 by 77 degrees on the first step.
    const target = ForwardKinematics.position(HOME_POSE_DEG);
    const badSeed = [999, -999, 999, -999, 999, -999];

    const result = ik.solvePosition(target, badSeed);
    expect(result.success).toBe(true);
    expect(isWithinLimitsDeg(result.jointAngles, 1e-6)).toBe(true);
  });
});

describe('inverse kinematics - full pose', () => {
  const ik = new InverseKinematics();

  it('solves position and orientation together', () => {
    const rng = makeRng(80808);
    const total = 200;
    let solved = 0;

    for (let trial = 0; trial < total; trial++) {
      const truth = randomPoseDeg(rng);
      const fk = ForwardKinematics.solve(truth);
      const result = ik.solvePose(fk.endEffectorPose, HOME_POSE_DEG);

      if (!result.success) continue;
      solved++;

      const achieved = ForwardKinematics.solve(result.jointAngles);
      expect(dist(achieved.endEffectorPose.position, fk.endEffectorPose.position)).toBeLessThan(
        ik.getConfig().positionTolerance
      );

      const Rerr = multiply3(
        rpyToMatrix(fk.endEffectorPose.rotation),
        transpose3(ForwardKinematics.solveRad(degToRad(result.jointAngles)).rotation)
      );
      const w = rotationLog(Rerr);
      expect(Math.hypot(w.x, w.y, w.z)).toBeLessThan(ik.getConfig().orientationTolerance);
    }

    // Six joints for six constraints, with limits as tight as J2 in [0, 60]
    // and J3 in [0, 70], so not every orientation is attainable at every point.
    expect(solved / total).toBeGreaterThan(0.9);
  });

  it('does not break down at gimbal lock', () => {
    // A target pitch of +/-90 deg makes the rpy representation singular. The
    // solver must not care, because it never differences Euler angles.
    const rng = makeRng(246);
    let solved = 0;
    let attempts = 0;

    for (let trial = 0; trial < 60; trial++) {
      const truth = randomPoseDeg(rng);
      const fk = ForwardKinematics.solve(truth);
      if (Math.abs(Math.abs(fk.endEffectorPose.rotation.pitch) - Math.PI / 2) > 0.05) continue;

      attempts++;
      if (ik.solvePose(fk.endEffectorPose, truth).success) solved++;
    }

    if (attempts > 0) {
      expect(solved).toBe(attempts);
    }
  });
});

// ---------------------------------------------------------------------------
// Workspace
// ---------------------------------------------------------------------------

describe('workspace bounds', () => {
  it('produces a finite box that contains the home pose', () => {
    const bounds = computeWorkspaceBounds(7);
    const home = ForwardKinematics.position(HOME_POSE_DEG);

    (['x', 'y', 'z'] as const).forEach(axis => {
      expect(Number.isFinite(bounds.min[axis])).toBe(true);
      expect(Number.isFinite(bounds.max[axis])).toBe(true);
      expect(bounds.max[axis]).toBeGreaterThan(bounds.min[axis]);
      expect(home[axis]).toBeGreaterThanOrEqual(bounds.min[axis] - 1e-9);
      expect(home[axis]).toBeLessThanOrEqual(bounds.max[axis] + 1e-9);
    });
  });
});

describe('holding the tool orientation', () => {
  // Solving for position alone leaves three of six degrees of freedom
  // unconstrained, so the wrist tips as the arm reaches. That is fine for
  // getting somewhere and useless for carrying a pen or a gripper.

  const ik = new InverseKinematics();

  function tumbleDeg(from: number[], to: number[]): number {
    const Ra = ForwardKinematics.solveRad(degToRad(from)).rotation;
    const Rb = ForwardKinematics.solveRad(degToRad(to)).rotation;
    const w = rotationLog(multiply3(Rb, transpose3(Ra)));
    return (Math.hypot(w.x, w.y, w.z) * 180) / Math.PI;
  }

  it('lets the tool tip when only the position is constrained', () => {
    const home = ForwardKinematics.solve(HOME_POSE_DEG);
    const p = home.endEffectorPose.position;

    const r = ik.solvePosition({ ...p, z: p.z + 0.02 }, [...HOME_POSE_DEG]);
    expect(r.success).toBe(true);

    // Not an accident to be fixed by tightening a tolerance: the constraint is
    // simply absent, and the tool ends up several degrees off.
    expect(tumbleDeg(HOME_POSE_DEG, r.jointAngles)).toBeGreaterThan(3);
    expect(r.orientationError).toBeUndefined();
  });

  it('holds the tool when the orientation is constrained too', () => {
    const home = ForwardKinematics.solve(HOME_POSE_DEG);
    const p = home.endEffectorPose.position;
    const rot = home.endEffectorPose.rotation;

    const r = ik.solvePose(
      { position: { ...p, z: p.z + 0.02 }, rotation: rot },
      [...HOME_POSE_DEG]
    );

    expect(r.success).toBe(true);
    expect(r.residualError!).toBeLessThan(0.0005);
    expect(tumbleDeg(HOME_POSE_DEG, r.jointAngles)).toBeLessThan(0.5);
    expect(r.orientationError).toBeDefined();
    expect(r.orientationError!).toBeLessThan(0.0087);
  });

  it('holds it across a run of moves without ratcheting away', () => {
    // The reason the lock captures one orientation instead of re-reading the
    // current one each move: re-reading lets each solve's residual become the
    // next one's reference, and the tool walks away over a sequence.
    const home = ForwardKinematics.solve(HOME_POSE_DEG);
    const rot = home.endEffectorPose.rotation;
    let q = [...HOME_POSE_DEG];

    for (let step = 1; step <= 6; step++) {
      const p = ForwardKinematics.position(HOME_POSE_DEG);
      const target = { x: p.x, y: p.y + step * 0.01, z: p.z };
      const r = ik.solvePose({ position: target, rotation: rot }, q);
      expect(r.success).toBe(true);
      q = r.jointAngles;
    }

    expect(tumbleDeg(HOME_POSE_DEG, q)).toBeLessThan(0.5);
  });

  it('reports which constraint it could not meet', () => {
    // Locked, reach shrinks - measured from the parked pose, +X gives about
    // 40 mm against 85 mm free. A target beyond that has to fail loudly, with
    // both residuals, rather than quietly returning a tipped-over pose.
    const home = ForwardKinematics.solve(HOME_POSE_DEG);
    const p = home.endEffectorPose.position;
    const rot = home.endEffectorPose.rotation;

    const r = ik.solvePose(
      { position: { ...p, x: p.x + 0.25 }, rotation: rot },
      [...HOME_POSE_DEG]
    );

    expect(r.success).toBe(false);
    expect(r.orientationError).toBeDefined();
    expect(r.error).toMatch(/orientation/);
  });
});
