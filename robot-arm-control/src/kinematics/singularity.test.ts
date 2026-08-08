// Measuring how close the arm is to a pose it cannot move out of freely.
//
// The parked pose is a known exact singularity - J4 and J6 turn the tool about
// the same axis there, determinant 2.5e-17 - so the arm supplies its own worked
// example, and the tests are written against it rather than against invented
// matrices. If the measure cannot find the one singularity this arm is known to
// have, it is not measuring anything.

import {
  NEAR_SINGULAR,
  manipulability,
  planEscape,
  singularityWarning
} from './singularity';
import { InverseKinematics } from './InverseKinematics';
import { ForwardKinematics } from './ForwardKinematics';
import { HOME_POSE_DEG, JOINT_LIMITS_DEG, NUM_JOINTS, degToRad } from './robotModel';
import { multiply3, rotationLog, symmetricEigen, transpose3 } from './linalg';

/** The parked pose, which is the singularity. */
const PARKED = [...HOME_POSE_DEG];

/** The same pose with J5 moved well clear of it. */
const CLEAR = (() => {
  const q = [...HOME_POSE_DEG];
  q[4] = 161;
  return q;
})();

describe('the symmetric eigensolver underneath it', () => {
  it('recovers eigenvalues that are known by construction', () => {
    // A diagonal matrix conjugated by a rotation: the eigenvalues are the
    // diagonal, whatever the rotation did to them.
    const c = Math.cos(0.7), s = Math.sin(0.7);
    const Q = [[c, -s, 0], [s, c, 0], [0, 0, 1]];
    const D = [[5, 0, 0], [0, 2, 0], [0, 0, 0.25]];
    const A = multiply3(multiply3(Q, D), transpose3(Q));

    const { values } = symmetricEigen(A);
    expect(values[0]).toBeCloseTo(0.25, 10);
    expect(values[1]).toBeCloseTo(2, 10);
    expect(values[2]).toBeCloseTo(5, 10);
  });

  it('returns vectors that really are eigenvectors', () => {
    const A = [
      [4, 1, 0.5],
      [1, 3, -0.2],
      [0.5, -0.2, 2]
    ];
    const { values, vectors } = symmetricEigen(A);

    for (let k = 0; k < 3; k++) {
      const v = vectors[k];
      expect(Math.hypot(...v)).toBeCloseTo(1, 10);
      for (let i = 0; i < 3; i++) {
        const Av = A[i][0] * v[0] + A[i][1] * v[1] + A[i][2] * v[2];
        expect(Av).toBeCloseTo(values[k] * v[i], 9);
      }
    }
  });

  it('handles a singular matrix, which is the case it exists for', () => {
    // Cholesky, next door, simply fails here - and this is exactly the input
    // that arrives when the arm is at a singularity.
    const A = [
      [1, 1, 0],
      [1, 1, 0],
      [0, 0, 2]
    ];
    const { values } = symmetricEigen(A);
    expect(values[0]).toBeCloseTo(0, 12);
    expect(values[2]).toBeCloseTo(2, 12);
  });
});

describe('measuring a pose', () => {
  it('reads zero at the singularity the arm is known to have', () => {
    const m = manipulability(PARKED);
    expect(m.worst).toBeLessThan(1e-6);
    expect(m.nearSingular).toBe(true);
  });

  it('names the two joints that have lined up', () => {
    // The physical content: at the parked pose J4 and J6 turn the tool about
    // one axis between them, so turning one and counter-turning the other moves
    // nothing at all. That pair is what the measure has to find.
    const m = manipulability(PARKED);
    expect(m.joints).toBe('J4 and J6');

    // And they are genuinely opposed in the lost direction, not merely both
    // present in it.
    const [, , , j4, , j6] = m.lostDirection;
    expect(Math.sign(j4)).toBe(-Math.sign(j6));
    expect(Math.abs(j4)).toBeCloseTo(Math.abs(j6), 3);
  });

  it('climbs steadily as the wrist moves away from it', () => {
    // Not just "bigger somewhere else": monotone in the distance from the
    // singularity, which is what makes it usable as a live readout rather than
    // as a flag.
    const at = (j5: number) => {
      const q = [...HOME_POSE_DEG];
      q[4] = j5;
      return manipulability(q).worst;
    };

    const walk = [131, 133, 136, 141, 151, 161].map(at);
    for (let i = 1; i < walk.length; i++) {
      expect(walk[i]).toBeGreaterThan(walk[i - 1]);
    }
    expect(walk[0]).toBeLessThan(1e-6);
    expect(walk[walk.length - 1]).toBeGreaterThan(NEAR_SINGULAR);
  });

  it('is symmetric about the singularity, because the geometry is', () => {
    const at = (j5: number) => {
      const q = [...HOME_POSE_DEG];
      q[4] = j5;
      return manipulability(q).worst;
    };
    expect(at(131 + 20)).toBeCloseTo(at(131 - 20), 3);
  });

  it('agrees with what a Cartesian step actually costs', () => {
    // The claim the number makes: joint motion is roughly tool motion over the
    // measure. If that does not hold, the readout is decoration.
    const ik = new InverseKinematics();

    for (const j5 of [141, 151, 161]) {
      const q = [...HOME_POSE_DEG];
      q[4] = j5;
      const m = manipulability(q);

      const fk = ForwardKinematics.solveRad(degToRad(q));
      // 2 mm along the direction the arm is worst at moving, which is what the
      // measure is about - any other direction is easier by definition.
      const step = 0.002;
      const target = { x: fk.position.x, y: fk.position.y, z: fk.position.z + step };
      const solved = ik.solvePoseMatrix(target, fk.rotation, q);
      expect(solved.success).toBe(true);

      const swing = Math.max(...solved.jointAngles.map((v, i) => Math.abs(v - q[i])));
      // The bound, not the value: the worst direction costs step/worst, and any
      // particular direction costs no more than that.
      const bound = ((step / m.worst) * 180) / Math.PI;
      expect(swing).toBeLessThanOrEqual(bound * 1.05);
    }
  });

  it('says nothing about a pose that is fine', () => {
    expect(singularityWarning(CLEAR)).toBeNull();
  });

  it('says what to do about one that is not', () => {
    const warning = singularityWarning(PARKED);
    expect(warning).not.toBeNull();
    expect(warning).toMatch(/J4 and J6/);
    // Names the joint to move, which is the only actionable part.
    expect(warning).toMatch(/J5/);
  });

  it('survives every pose inside the joint limits', () => {
    // An eigensolver that returned NaN on some pose would poison a live readout
    // and every guard built on it.
    let seed = 12345;
    const rand = () => {
      seed = (seed * 1103515245 + 12345) % 2147483648;
      return seed / 2147483648;
    };
    for (let n = 0; n < 800; n++) {
      const q: number[] = [];
      for (let j = 0; j < NUM_JOINTS; j++) {
        q.push(JOINT_LIMITS_DEG.min[j] + rand() * (JOINT_LIMITS_DEG.max[j] - JOINT_LIMITS_DEG.min[j]));
      }
      const m = manipulability(q);
      expect(Number.isFinite(m.worst)).toBe(true);
      expect(m.worst).toBeGreaterThanOrEqual(0);
      expect(m.lostDirection.every(Number.isFinite)).toBe(true);
    }
  });
});

describe('getting out of one', () => {
  const ik = new InverseKinematics();

  it('offers nothing when there is nothing to escape', () => {
    expect(planEscape(CLEAR, ik)).toBeNull();
  });

  it('finds a way out of the parked pose', () => {
    const escape = planEscape(PARKED, ik);
    expect(escape).not.toBeNull();
    if (!escape) return;

    expect(escape.worst).toBeGreaterThan(NEAR_SINGULAR);
    expect(manipulability(escape.jointAngles).nearSingular).toBe(false);
  });

  it('finds J5 by searching, rather than being told', () => {
    // Every joint is tried and the cheapest wins. On this arm that is reliably
    // J5 - the joint the refusal messages have been telling the operator to move
    // by hand - and searching means the answer stays right if the geometry
    // changes.
    const escape = planEscape(PARKED, ik);
    expect(escape?.joint).toBe('J5');
  });

  it('keeps the tip where it was, and pays in attitude', () => {
    // The honest trade. There is no free escape: every configuration reaching
    // this exact pose has J4 and J6 lined up, so the tool has to move. What can
    // be kept is the tip.
    const escape = planEscape(PARKED, ik);
    expect(escape).not.toBeNull();
    if (!escape) return;

    expect(escape.driftMm).toBeLessThan(0.5);
    // And the attitude cost is real, reported, and modest.
    expect(escape.tiltDeg).toBeGreaterThan(1);
    expect(escape.tiltDeg).toBeLessThan(20);
  });

  it('reports a cost that matches what the move actually does', () => {
    // The numbers shown before the arm moves have to be the numbers it delivers,
    // or the offer is a guess.
    const escape = planEscape(PARKED, ik);
    expect(escape).not.toBeNull();
    if (!escape) return;

    const before = ForwardKinematics.solveRad(degToRad(PARKED));
    const after = ForwardKinematics.solveRad(degToRad(escape.jointAngles));

    const w = rotationLog(multiply3(after.rotation, transpose3(before.rotation)));
    expect((Math.hypot(w.x, w.y, w.z) * 180) / Math.PI).toBeCloseTo(escape.tiltDeg, 6);

    const drift = Math.hypot(
      after.position.x - before.position.x,
      after.position.y - before.position.y,
      after.position.z - before.position.z
    ) * 1000;
    expect(drift).toBeCloseTo(escape.driftMm, 6);
  });

  it('stays inside the joint limits', () => {
    const escape = planEscape(PARKED, ik);
    expect(escape).not.toBeNull();
    if (!escape) return;

    escape.jointAngles.forEach((v, i) => {
      expect(v).toBeGreaterThanOrEqual(JOINT_LIMITS_DEG.min[i] - 1e-6);
      expect(v).toBeLessThanOrEqual(JOINT_LIMITS_DEG.max[i] + 1e-6);
    });
  });
});
