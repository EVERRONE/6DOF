// Finding the tool tip by touching one point from several directions.
//
// Built as a simulation of the real procedure rather than as algebra: take a
// tool offset, generate the flange poses that would put its tip on one fixed
// point, and check the solver gets the offset back. If that works the operator's
// version works, because it is the same problem with worse numbers.

import { solveToolOffset, ToolTouch } from './toolCalibration';
import { ForwardKinematics } from './ForwardKinematics';
import { HOME_POSE_DEG, degToRad, resetToolFrame } from './robotModel';
import { getRotation, getTranslation, rpyToMatrix } from './linalg';
import { Vector3 } from './types';

const DEG = Math.PI / 180;

afterEach(() => resetToolFrame());

/**
 * Flange poses whose tip lands on one point, generated from real arm poses.
 *
 * The rotations are the arm's own, not made up, so the conditioning of the
 * system is the conditioning the operator will actually get.
 */
function touchesFor(offset: Vector3, poses: number[][]): { touches: ToolTouch[]; point: Vector3 } {
  // Put the tip of the first pose somewhere, then place every other flange so
  // its tip lands on the same place.
  const first = ForwardKinematics.solveRad(degToRad(poses[0]));
  const R0 = getRotation(first.frames[5]);
  const p0 = getTranslation(first.frames[5]);
  const point = {
    x: p0.x + R0[0][0] * offset.x + R0[0][1] * offset.y + R0[0][2] * offset.z,
    y: p0.y + R0[1][0] * offset.x + R0[1][1] * offset.y + R0[1][2] * offset.z,
    z: p0.z + R0[2][0] * offset.x + R0[2][1] * offset.y + R0[2][2] * offset.z
  };

  const touches = poses.map(q => {
    const fk = ForwardKinematics.solveRad(degToRad(q));
    const R = getRotation(fk.frames[5]);
    // The flange that puts this orientation's tip on `point`.
    return {
      rotation: R,
      position: {
        x: point.x - (R[0][0] * offset.x + R[0][1] * offset.y + R[0][2] * offset.z),
        y: point.y - (R[1][0] * offset.x + R[1][1] * offset.y + R[1][2] * offset.z),
        z: point.z - (R[2][0] * offset.x + R[2][1] * offset.y + R[2][2] * offset.z)
      }
    };
  });

  return { touches, point };
}

/** Four arm poses that turn the wrist well apart from each other. */
function spreadPoses(): number[][] {
  const base = [...HOME_POSE_DEG];
  return [
    [...base],
    [base[0], base[1], base[2], base[3] - 40, base[4] + 30, base[5]],
    [base[0], base[1], base[2], base[3] + 35, base[4] - 25, base[5] + 40],
    [base[0] + 20, base[1], base[2], base[3], base[4] + 45, base[5] - 30]
  ];
}

describe('solving for the tool tip', () => {
  it('recovers an offset it was never told', () => {
    const truth = { x: 0.012, y: -0.008, z: 0.095 };
    const { touches, point } = touchesFor(truth, spreadPoses());

    const result = solveToolOffset(touches);
    expect(result.ok).toBe(true);
    if (!result.ok) return;

    expect(result.offset.x * 1000).toBeCloseTo(truth.x * 1000, 6);
    expect(result.offset.y * 1000).toBeCloseTo(truth.y * 1000, 6);
    expect(result.offset.z * 1000).toBeCloseTo(truth.z * 1000, 6);

    // And it says where the touched point was, which is worth having: it is the
    // one place in the workspace whose position is now known independently.
    expect(result.point.x * 1000).toBeCloseTo(point.x * 1000, 6);
    expect(result.residualMm).toBeLessThan(0.001);
  });

  it('reports how far the touches disagree, and it is the operator aim', () => {
    const truth = { x: 0, y: 0, z: 0.08 };
    const { touches } = touchesFor(truth, spreadPoses());

    // Half a millimetre of shaky aim on one touch.
    touches[2].position = {
      x: touches[2].position.x + 0.0005,
      y: touches[2].position.y,
      z: touches[2].position.z
    };

    const result = solveToolOffset(touches);
    expect(result.ok).toBe(true);
    if (!result.ok) return;

    // The residual is the same order as the error put in - it does not swallow
    // it, and it does not amplify it into something alarming.
    expect(result.residualMm).toBeGreaterThan(0.1);
    expect(result.residualMm).toBeLessThan(1);
    // The offset is still close, because three good touches outvote one bad one.
    expect(result.offset.z * 1000).toBeCloseTo(80, 0);
  });

  it('refuses touches that are all from the same direction', () => {
    const truth = { x: 0, y: 0, z: 0.08 };
    const base = [...HOME_POSE_DEG];

    // Four poses, wrist barely turned between them. The solve would still
    // return a number, and the number would be noise.
    const { touches } = touchesFor(truth, [
      [...base],
      [base[0], base[1] + 1, base[2], base[3], base[4], base[5]],
      [base[0], base[1], base[2] + 1, base[3], base[4], base[5]],
      [base[0], base[1] + 2, base[2], base[3], base[4], base[5]]
    ]);

    const result = solveToolOffset(touches);
    expect(result.ok).toBe(false);
    if (result.ok) return;
    expect(result.reason).toMatch(/apart in orientation/i);
  });

  it('needs four touches, not three', () => {
    const truth = { x: 0, y: 0, z: 0.08 };
    const { touches } = touchesFor(truth, spreadPoses());

    const three = solveToolOffset(touches.slice(0, 3));
    expect(three.ok).toBe(false);
    if (!three.ok) expect(three.reason).toMatch(/four touches/i);
  });

  it('refuses an answer that is not a tool', () => {
    // Touches of four genuinely different points rather than one - the mistake
    // an operator makes by nudging the workpiece between touches.
    const poses = spreadPoses();
    const touches: ToolTouch[] = poses.map((q, i) => {
      const fk = ForwardKinematics.solveRad(degToRad(q));
      return {
        rotation: getRotation(fk.frames[5]),
        position: getTranslation(fk.frames[5])
      };
    });
    // Each flange sits where the arm puts it, so the "same point" claim is false
    // and the offset that reconciles them is enormous or the residual is.
    const result = solveToolOffset(touches);
    if (result.ok) {
      expect(result.residualMm).toBeGreaterThan(1);
    } else {
      expect(result.reason).toMatch(/500 mm|too alike/i);
    }
  });

  it('is unaffected by a tool frame already being set', () => {
    // The touches are flange poses, frame 6, taken before any tool is applied -
    // so re-calibrating cannot compound onto the previous answer. Worth pinning:
    // reading the TCP instead would make every calibration relative to the last.
    const truth = { x: 0.01, y: 0, z: 0.09 };

    const before = solveToolOffset(touchesFor(truth, spreadPoses()).touches);
    // FK is what generates the touches, and it now includes a tool.
    const { setToolFrame } = require('./robotModel');
    setToolFrame({ xyz: { x: 0.05, y: 0.05, z: 0.05 } });
    const after = solveToolOffset(touchesFor(truth, spreadPoses()).touches);

    expect(before.ok && after.ok).toBe(true);
    if (!before.ok || !after.ok) return;
    expect(after.offset.z).toBeCloseTo(before.offset.z, 9);
  });
});

describe('judging whether the answer means anything', () => {
  it('calls a real tool measured', () => {
    const { touches } = touchesFor({ x: 0.012, y: -0.008, z: 0.095 }, spreadPoses());
    const result = solveToolOffset(touches);
    expect(result.ok).toBe(true);
    if (!result.ok) return;
    expect(result.verdict.kind).toBe('good');
  });

  it('calls an offset smaller than its own scatter noise', () => {
    // What a real session produced: a 5.2 mm offset with 3 mm of disagreement.
    // Least squares always returns something; that does not make it a tool.
    const truth = { x: -0.0025, y: 0.00255, z: -0.00383 };
    const { touches } = touchesFor(truth, spreadPoses());

    // Scatter the touches by a couple of millimetres, as an operator aiming at a
    // nail with a printed arm does.
    const jitter = [
      { x: 0.002, y: -0.001, z: 0.0015 },
      { x: -0.0018, y: 0.002, z: -0.001 },
      { x: 0.001, y: 0.0015, z: 0.002 },
      { x: -0.001, y: -0.002, z: -0.0015 }
    ];
    touches.forEach((t, i) => {
      t.position = {
        x: t.position.x + jitter[i].x,
        y: t.position.y + jitter[i].y,
        z: t.position.z + jitter[i].z
      };
    });

    const result = solveToolOffset(touches);
    expect(result.ok).toBe(true);
    if (!result.ok) return;

    expect(result.residualMm).toBeGreaterThan(1);
    expect(result.verdict.kind).toBe('in-the-noise');
    expect(result.verdict.note).toMatch(/bare flange/i);
  });

  it('calls a large offset from close-together touches soft rather than good', () => {
    const base = [...HOME_POSE_DEG];
    // Above the 20 degree refusal, below the 60 that makes it solid.
    const { touches } = touchesFor({ x: 0, y: 0, z: 0.09 }, [
      [...base],
      [base[0], base[1], base[2], base[3] - 12, base[4] + 8, base[5]],
      [base[0], base[1], base[2], base[3] + 10, base[4] - 9, base[5]],
      [base[0], base[1], base[2], base[3] - 5, base[4] + 14, base[5]]
    ]);

    const result = solveToolOffset(touches);
    expect(result.ok).toBe(true);
    if (!result.ok) return;
    expect(result.spreadDeg).toBeLessThan(60);
    expect(result.verdict.kind).toBe('poorly-conditioned');
  });
});
