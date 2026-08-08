// How close the arm is to a pose it cannot move out of freely.
//
// A singularity is not a place the arm cannot go. It is a place it can go and
// then cannot leave in some direction without enormous joint motion: two joint
// axes have lined up, so between them they turn the tool about one axis instead
// of two, and the direction they used to cover is gone. The arm sits there
// perfectly happily. It is the next move that fails.
//
// This arm's is the wrist. At the parked pose J5 sits at its URDF zero and J4
// and J6 turn the tool about the same axis - their columns in the orientation
// Jacobian are identical and the determinant is 2.5e-17. The symptom, which the
// operator has already met, is a refusal: "J4 jumps 13 degrees between two
// samples 2 mm apart".
//
// Nothing measured it. The refusals name the joint that jumped and the pose it
// happened at, which is the consequence; this is the cause, as one number that
// exists before anything is refused.

import { ForwardKinematics } from './ForwardKinematics';
import { JOINT_LIMITS_DEG, NUM_JOINTS, degToRad } from './robotModel';
import { multiply3, normalMatrix, rotationLog, symmetricEigen, transpose3 } from './linalg';
import { IKResult, Vector3 } from './types';

/**
 * Length used to put radians and metres on one footing, in metres.
 *
 * A Jacobian's top three rows are metres per radian and its bottom three are
 * dimensionless, so its singular values have no common meaning until a length
 * is chosen. This one is the arm's own reach - so a singular value reads as
 * "tool motion at the far end of the arm, per radian of joint motion", and the
 * wrist and the shoulder are measured on the same scale.
 *
 * Any positive length gives a valid comparison between poses. This one makes
 * the number mean something on its own.
 */
export const TASK_SCALE_M = 0.37;

/**
 * Below this the arm is near enough a singularity that Cartesian moves start
 * costing large joint motion.
 *
 * Measured on this arm rather than picked. At the parked pose - the singularity
 * the path planner already refuses lines through - it reads exactly 0, and it
 * climbs as J5 moves away:
 *
 *   J5 = 131 (parked)  0.0000     J5 = 151   0.0375
 *   J5 = 133           0.0038     J5 = 161   0.0554
 *   J5 = 136           0.0095     J5 =  91   0.0722
 *   J5 = 141           0.0190     best anywhere  0.097
 *
 * A useful way to read it: joint motion needed is roughly tool motion divided
 * by this, so at 0.02 a 2 mm Cartesian step costs about 6 degrees of wrist, and
 * at 0.005 it costs 23. The threshold sits where that starts being felt, which
 * is earlier than the path planner refuses anything - deliberately, since the
 * point is to say so before something is refused.
 *
 * 11.8% of poses drawn uniformly from the joint limits fall below it.
 */
export const NEAR_SINGULAR = 0.02;

export interface Manipulability {
  /**
   * Smallest singular value of the scaled Jacobian: how much tool motion the
   * arm can produce in its worst direction, per radian of joint motion.
   *
   * Zero at an exact singularity. Bigger is better, and there is no upper
   * bound worth quoting - what matters is how close to zero it is.
   */
  worst: number;
  /**
   * Ratio of the best direction to the worst.
   *
   * The unitless companion to `worst`, and the one that says how *lopsided* the
   * arm is here rather than how strong. A large ratio means a Cartesian move
   * that is easy one way is expensive another, which is what makes a path
   * through this region unpredictable.
   */
  ratio: number;
  /** True when the arm is close enough to a singularity to be worth saying so. */
  nearSingular: boolean;
  /**
   * The joint motion that produces (almost) no tool motion, normalised.
   *
   * The physical content of the singularity: at the wrist singularity this
   * comes out as J4 and J6 opposed, because turning one and counter-turning the
   * other moves nothing at all.
   */
  lostDirection: number[];
  /**
   * The one or two joints that dominate `lostDirection`, ready to show - "J4
   * and J6". What the operator has to move to get out.
   */
  joints: string;
}

const LABELS = ['J1', 'J2', 'J3', 'J4', 'J5', 'J6'];

/**
 * Jacobian with the rotation rows scaled into metres, so its singular values
 * are comparable.
 */
function scaledJacobian(qDeg: number[]): number[][] {
  const J = ForwardKinematics.jacobian(qDeg);
  return J.map((row, r) => (r < 3 ? [...row] : row.map(v => v * TASK_SCALE_M)));
}

/**
 * How freely the arm can move the tool from this pose, and which way it cannot.
 *
 * Singular values rather than the determinant. The determinant is the product
 * of all six, so it is small both when one direction is lost and when the arm
 * is merely slow everywhere, and it cannot say which. The smallest singular
 * value answers the question actually being asked - is there a direction the
 * tool cannot be moved in - and its vector says which joints are responsible.
 *
 * Taken through J^T J rather than an SVD of J: it is symmetric, 6x6, and the
 * eigenvectors of J^T J are exactly the joint-space directions wanted here.
 * Squaring does cost half the working precision in the singular value, which is
 * acceptable when the threshold is 0.02 and the value at the singularity is
 * 0.002.
 */
export function manipulability(jointAnglesDeg: number[]): Manipulability {
  const { values, vectors } = symmetricEigen(normalMatrix(scaledJacobian(jointAnglesDeg)));

  // Eigenvalues of J^T J are the squared singular values of J. Clamped because
  // an exactly singular matrix can come back a hair negative.
  const sigma = values.map(v => Math.sqrt(Math.max(0, v)));
  const worst = sigma[0];
  const best = sigma[sigma.length - 1];

  const lost = vectors[0];
  const magnitudes = lost.map(Math.abs);
  const ranked = magnitudes
    .map((m, i) => ({ m, i }))
    .sort((a, b) => b.m - a.m);

  // Second joint named only when it genuinely shares the direction. A
  // singularity is normally two joints fighting each other, but reporting a
  // second one that contributes 5% would point the operator at the wrong axis.
  const named =
    ranked[1].m > ranked[0].m * 0.5
      ? [ranked[0].i, ranked[1].i].sort((a, b) => a - b)
      : [ranked[0].i];

  return {
    worst,
    ratio: worst > 0 ? best / worst : Infinity,
    nearSingular: worst < NEAR_SINGULAR,
    lostDirection: lost,
    joints: named.map(i => LABELS[i]).join(' and ')
  };
}

/**
 * Ready-to-show account of a near-singular pose, or null when it is fine.
 *
 * Written to say what to do about it. "Singular configuration" tells an
 * operator nothing; "J4 and J6 are lined up, move J5" tells them where to put
 * their hand.
 */
export function singularityWarning(jointAnglesDeg: number[]): string | null {
  const m = manipulability(jointAnglesDeg);
  if (!m.nearSingular) return null;

  return (
    `${m.joints} are lined up here (${m.worst.toFixed(4)} of free movement, ` +
    `${NEAR_SINGULAR} is the mark). Cartesian moves from this pose cost large ` +
    'wrist motion for very little tool motion. Move J5 away from where it is.'
  );
}

/** Manipulability at a pose given in radians, for callers already in radians. */
export function manipulabilityRad(qRad: number[]): Manipulability {
  return manipulability(qRad.map(v => (v * 180) / Math.PI).slice(0, NUM_JOINTS));
}

export interface Escape {
  /** Where to move to. */
  jointAngles: number[];
  /** Manipulability there - above NEAR_SINGULAR, or this would not be an escape. */
  worst: number;
  /** Which joint had to move, and by how much. */
  joint: string;
  movedDeg: number;
  /** What it costs: the tool tips this far, degrees. */
  tiltDeg: number;
  /** And its tip drifts this far, millimetres. */
  driftMm: number;
}

/** Solver interface the escape needs, kept narrow so this file owes nothing to the class. */
interface PositionSolver {
  solvePosition(target: Vector3, seed?: number[]): IKResult;
}

/** How far a joint may be pushed looking for a way out, degrees. */
const ESCAPE_LIMIT_DEG = 40;
const ESCAPE_STEP_DEG = 2;

/**
 * The cheapest way out of a near-singular pose, holding the tool tip where it is.
 *
 * There is no free escape and it is worth being plain about why. A singularity
 * is a property of the pose the arm is in, not of a choice it made getting
 * there: at the wrist singularity every configuration that reaches this exact
 * pose has J4 and J6 lined up, so no re-solve of the same pose helps. The tool
 * has to move. What can be kept is the tip - the arm turns about it - so the
 * cost is paid entirely in attitude, and this reports how much before anything
 * moves.
 *
 * Every joint is tried rather than assuming the wrist, and the one costing the
 * least tilt wins. On this arm that is reliably J5, which is the joint the
 * refusal messages have been telling the operator to move by hand; searching
 * for it means the answer stays right if the geometry ever changes.
 *
 * Returns null when the pose is already clear, or when nothing within
 * ESCAPE_LIMIT_DEG of any joint gets clear of it.
 */
export function planEscape(fromDeg: number[], solver: PositionSolver): Escape | null {
  if (!manipulability(fromDeg).nearSingular) return null;

  const startFk = ForwardKinematics.solveRad(degToRad(fromDeg));
  const tip = startFk.position;
  const R0 = startFk.rotation;

  let best: Escape | null = null;

  for (let joint = 0; joint < NUM_JOINTS; joint++) {
    for (let step = ESCAPE_STEP_DEG; step <= ESCAPE_LIMIT_DEG; step += ESCAPE_STEP_DEG) {
      let cleared = false;

      for (const sign of [1, -1]) {
        const seed = [...fromDeg];
        seed[joint] = fromDeg[joint] + sign * step;
        if (
          seed[joint] < JOINT_LIMITS_DEG.min[joint] ||
          seed[joint] > JOINT_LIMITS_DEG.max[joint]
        ) {
          continue;
        }

        // Position only: the tip is what is being kept, and demanding the
        // attitude too would just re-find the singular pose.
        const solved = solver.solvePosition(tip, seed);
        if (!solved.success) continue;

        const q = solved.jointAngles;
        const m = manipulability(q);
        if (!m.nearSingular) {
          cleared = true;

          const fk = ForwardKinematics.solveRad(degToRad(q));
          const w = rotationLog(multiply3(fk.rotation, transpose3(R0)));
          const candidate: Escape = {
            jointAngles: q,
            worst: m.worst,
            joint: LABELS[joint],
            movedDeg: Math.abs(q[joint] - fromDeg[joint]),
            tiltDeg: (Math.hypot(w.x, w.y, w.z) * 180) / Math.PI,
            driftMm:
              Math.hypot(fk.position.x - tip.x, fk.position.y - tip.y, fk.position.z - tip.z) * 1000
          };

          if (best === null || candidate.tiltDeg < best.tiltDeg) best = candidate;
        }
      }

      // Stepping further out only costs more tilt, so this joint is done as
      // soon as either direction works.
      if (cleared) break;
    }
  }

  return best;
}
