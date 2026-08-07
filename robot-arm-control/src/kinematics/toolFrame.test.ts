// The tool frame: where the tool is relative to the flange, and which way it
// points.
//
// Both halves are tested against what they are for. The offset decides where
// "the position" is measured to, and it is the reason J6 can move the TCP at
// all. The rotation decides what an orientation *means*, which is what makes
// "point the tool along the world axes" a statement about the tool rather than
// about the flange it happens to be bolted to.

import { ForwardKinematics } from './ForwardKinematics';
import { InverseKinematics, workspaceBounds } from './InverseKinematics';
import {
  DEFAULT_TOOL_FRAME,
  HOME_POSE_DEG,
  degToRad,
  getToolFrame,
  resetToolFrame,
  setToolFrame,
  toolFrameIsIdentity
} from './robotModel';
import { getAxis, multiply3, rotationLog, rpyToMatrix, transpose3 } from './linalg';

const RAD = Math.PI / 180;

/** Angle between two rotation matrices, in degrees. */
function angleBetween(a: number[][], b: number[][]): number {
  const w = rotationLog(multiply3(a, transpose3(b)));
  return (Math.hypot(w.x, w.y, w.z) * 180) / Math.PI;
}

const fk = (q: number[]) => ForwardKinematics.solveRad(degToRad(q));

// Module state: a tool left fitted by one test changes the answer every later
// test gets, and the failure reads as a bug in the code under test.
afterEach(() => resetToolFrame());

describe('the tool frame itself', () => {
  it('starts as a bare flange', () => {
    expect(getToolFrame()).toEqual(DEFAULT_TOOL_FRAME);
    expect(toolFrameIsIdentity()).toBe(true);
  });

  it('can be set one half at a time', () => {
    // The offset is usually measured before the rotation is, so a partial
    // update must not silently zero the half that was not given.
    setToolFrame({ xyz: { x: 0, y: 0, z: 0.1 } });
    setToolFrame({ rpy: { roll: 0.5, pitch: 0, yaw: 0 } });

    const frame = getToolFrame();
    expect(frame.xyz.z).toBeCloseTo(0.1, 9);
    expect(frame.rpy.roll).toBeCloseTo(0.5, 9);
  });

  it('hands out copies, so a caller cannot mutate the model', () => {
    setToolFrame({ xyz: { x: 0, y: 0, z: 0.1 } });
    const frame = getToolFrame();
    frame.xyz.z = 999;
    expect(getToolFrame().xyz.z).toBeCloseTo(0.1, 9);
  });
});

describe('the offset: where the tip is', () => {
  it('moves the reported position along the flange axis, by exactly the offset', () => {
    const q = [...HOME_POSE_DEG];
    const flange = fk(q);

    setToolFrame({ xyz: { x: 0, y: 0, z: 0.1 } });
    const tip = fk(q);

    // T_tcp = T_6 * Translate(xyz), so the tip is the flange origin plus the
    // offset expressed in base coordinates - here 100 mm along frame 6's Z.
    const z6 = getAxis(flange.frames[5], 2);
    expect(tip.position.x).toBeCloseTo(flange.position.x + 0.1 * z6.x, 9);
    expect(tip.position.y).toBeCloseTo(flange.position.y + 0.1 * z6.y, 9);
    expect(tip.position.z).toBeCloseTo(flange.position.z + 0.1 * z6.z, 9);
  });

  it('leaves the orientation alone', () => {
    const q = [...HOME_POSE_DEG];
    const before = fk(q).rotation;
    setToolFrame({ xyz: { x: 0.05, y: -0.02, z: 0.1 } });
    expect(angleBetween(before, fk(q).rotation)).toBeCloseTo(0, 9);
  });

  it('makes J6 move the TCP, which on a bare flange it cannot', () => {
    const a = [...HOME_POSE_DEG];
    const b = [...HOME_POSE_DEG];
    b[5] = 90;

    // On a bare flange the TCP sits on J6's own axis, so spinning it moves
    // nothing. This is why the workspace sampler used to take a single J6
    // sample, and why it must not once a tool hangs off that axis.
    const bare = Math.hypot(
      fk(a).position.x - fk(b).position.x,
      fk(a).position.y - fk(b).position.y,
      fk(a).position.z - fk(b).position.z
    );
    expect(bare * 1000).toBeLessThan(1e-6);

    // 40 mm off the axis, a quarter turn sweeps the tip across the diameter of
    // the circle it traces: 40 * sqrt(2) for 90 degrees.
    setToolFrame({ xyz: { x: 0.04, y: 0, z: 0.02 } });
    const fitted = Math.hypot(
      fk(a).position.x - fk(b).position.x,
      fk(a).position.y - fk(b).position.y,
      fk(a).position.z - fk(b).position.z
    );
    expect(fitted * 1000).toBeCloseTo(40 * Math.SQRT2, 3);
  });

  it('moves the reachable workspace with it, without serving a stale cache', () => {
    // The bounds cost thousands of FK solves and are cached. They were cached on
    // the reasoning that the joint limits are compile-time constants, which
    // stopped being the whole story once the tool became settable: a stale box
    // would reject targets the arm can now reach.
    const bare = workspaceBounds(5);
    setToolFrame({ xyz: { x: 0, y: 0, z: 0.15 } });
    const fitted = workspaceBounds(5);

    const spanBare = bare.max.z - bare.min.z;
    const spanFitted = fitted.max.z - fitted.min.z;
    expect(spanFitted).toBeGreaterThan(spanBare + 0.05);

    // And back, from the cache this time.
    resetToolFrame();
    expect(workspaceBounds(5).max.z).toBeCloseTo(bare.max.z, 9);
  });
});

describe('the rotation: which way the tool points', () => {
  it('turns the reported orientation by exactly the rotation given', () => {
    const q = [...HOME_POSE_DEG];
    const flange = fk(q).rotation;

    const rpy = { roll: 30 * RAD, pitch: 0, yaw: 0 };
    setToolFrame({ rpy });

    // R_tcp = R_6 * R(rpy): the tool's own rotation, applied in the flange frame.
    const expected = multiply3(flange, rpyToMatrix(rpy));
    expect(angleBetween(fk(q).rotation, expected)).toBeCloseTo(0, 9);
  });

  it('leaves the tip where it is', () => {
    // Translate then rotate, so the rotation cannot move the point it rotates
    // about. Worth pinning: reversing the two would make every calibration of
    // the rotation quietly shift the tip as well.
    const q = [...HOME_POSE_DEG];
    setToolFrame({ xyz: { x: 0, y: 0, z: 0.1 } });
    const before = fk(q).position;
    setToolFrame({ rpy: { roll: 30 * RAD, pitch: 20 * RAD, yaw: -10 * RAD } });
    const after = fk(q).position;

    expect(after.x).toBeCloseTo(before.x, 12);
    expect(after.y).toBeCloseTo(before.y, 12);
    expect(after.z).toBeCloseTo(before.z, 12);
  });

  it('is what makes an orientation target mean the tool and not the flange', () => {
    // The point of the rotation half. Ask for an attitude, and it is the tool
    // that must end up in it - the flange goes wherever it has to so that the
    // tool does. Without the rotation modelled there is nowhere to put the
    // difference, and "hold the tool level" silently means "hold the flange
    // level", which is only the same if the tool was bolted on square.
    const ik = new InverseKinematics();
    const seed = [...HOME_POSE_DEG];

    // Somewhere off the wrist singularity, so the solve is well conditioned.
    seed[4] = 151;
    const target = ForwardKinematics.position(seed);
    const wanted = fk(seed).rotation;

    const bolted = { roll: 25 * RAD, pitch: -15 * RAD, yaw: 40 * RAD };
    setToolFrame({ rpy: bolted });

    const solved = ik.solvePose({ position: target, rotation: matrixRpy(wanted) }, seed);
    expect(solved.success).toBe(true);

    const reached = fk(solved.jointAngles);
    // The tool is where it was asked to be...
    expect(angleBetween(reached.rotation, wanted)).toBeLessThan(0.5);
    // ...and the flange is not, by exactly the angle the tool is bolted at.
    const flange = multiply3(reached.rotation, transpose3(rpyToMatrix(bolted)));
    expect(angleBetween(flange, wanted)).toBeGreaterThan(1);
  });
});

describe('IK against a fitted tool', () => {
  it('round-trips: a solved pose puts the tool tip on the target', () => {
    setToolFrame({
      xyz: { x: 0.01, y: 0, z: 0.08 },
      rpy: { roll: 10 * RAD, pitch: 0, yaw: 0 }
    });

    const ik = new InverseKinematics();
    const seed = [...HOME_POSE_DEG];
    seed[4] = 151;

    // Generated by FK from a real pose, so it is reachable by construction and
    // reachable *with this tool on* - which is the thing being checked.
    const pose = [...seed];
    pose[1] = 25;
    pose[2] = 55;
    const target = ForwardKinematics.position(pose);
    const wanted = fk(pose).rotation;

    const res = ik.solvePose({ position: target, rotation: matrixRpy(wanted) }, seed);
    expect(res.success).toBe(true);

    const reached = fk(res.jointAngles);
    const mm = Math.hypot(
      reached.position.x - target.x,
      reached.position.y - target.y,
      reached.position.z - target.z
    ) * 1000;
    expect(mm).toBeLessThan(0.5);
    expect(angleBetween(reached.rotation, wanted)).toBeLessThan(0.5);
  });
});

/** Rotation matrix to the fixed-axis rpy solvePose takes. */
function matrixRpy(R: number[][]) {
  return {
    roll: Math.atan2(R[2][1], R[2][2]),
    pitch: Math.atan2(-R[2][0], Math.hypot(R[0][0], R[1][0])),
    yaw: Math.atan2(R[1][0], R[0][0])
  };
}
