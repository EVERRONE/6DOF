// Putting the tool exactly square with a frame's axes.
//
// "Hold it straight along the axes" is not something jogging can do. A turn jog
// moves by a fixed step, so from 1.3 degrees off you can reach 0.3 or -3.7 but
// never zero, and the arm has no reason to prefer square - every attitude is as
// valid as any other to the solver. The operator ends up chasing a number.
//
// Square is not a preference, though. It is a small, finite set: the 24
// attitudes in which every tool axis lies along a frame axis. Snapping to the
// nearest one is exact by construction, needs no iteration, and the distance to
// it is the answer to "how far off am I", which is worth showing on its own.

/** Rotation matrix determinant. Only ever called on candidates being screened. */
function det3(R: number[][]): number {
  return (
    R[0][0] * (R[1][1] * R[2][2] - R[1][2] * R[2][1]) -
    R[0][1] * (R[1][0] * R[2][2] - R[1][2] * R[2][0]) +
    R[0][2] * (R[1][0] * R[2][1] - R[1][1] * R[2][0])
  );
}

/**
 * The 24 attitudes in which each axis of one frame lies along an axis of the
 * other.
 *
 * Built rather than typed out. There are 48 ways to send three axes to three
 * signed axes - 6 orderings times 8 sign choices - and half of them are
 * reflections, which no rigid body can adopt. Screening on the determinant
 * leaves the 24 that are rotations, and it leaves them without a table anybody
 * has to check by eye.
 */
function buildAxisAlignedRotations(): number[][][] {
  const orderings = [
    [0, 1, 2], [0, 2, 1], [1, 0, 2],
    [1, 2, 0], [2, 0, 1], [2, 1, 0]
  ];

  const out: number[][][] = [];
  for (const order of orderings) {
    for (let bits = 0; bits < 8; bits++) {
      const R = [
        [0, 0, 0],
        [0, 0, 0],
        [0, 0, 0]
      ];
      // Column k is the reference axis that the body's k-th axis lands on,
      // which is what makes the columns readable as "tool X goes to +Y".
      for (let k = 0; k < 3; k++) {
        R[order[k]][k] = bits & (1 << k) ? -1 : 1;
      }
      if (det3(R) > 0) out.push(R);
    }
  }
  return out;
}

export const AXIS_ALIGNED_ROTATIONS: number[][][] = buildAxisAlignedRotations();

/**
 * A ceiling on how far any attitude can be from the nearest axis-aligned one.
 *
 * The 24 attitudes cover the whole of SO(3) to within 62.7992 degrees - the
 * covering radius of this set, found by hill-climbing on the distance function
 * rather than quoted. Rounded up here, so an error above it means the input was
 * not a rotation, not that the arm found an unusually crooked pose.
 *
 * Worth knowing because it bounds the surprise: pressing "straighten" can cost
 * at most this much wrist travel, whatever attitude the arm is in.
 */
export const MAX_ALIGNMENT_ERROR_DEG = 63;

/**
 * Below this, the arm is as square as its steppers can make it.
 *
 * Orientation is carried by the wrist, and J6 is much the coarsest joint on this
 * arm - 8.889 microsteps per degree in `firmware/config.h`, so one step is
 * 0.1125 degrees and rounding to the nearest leaves up to half of that. J4 and
 * J5 are three times finer and contribute little beside it.
 *
 * Reported rather than hidden. Without it the panel shows 0.04 degrees after a
 * straighten and reads as a failure, when it is the machine's resolution and no
 * further command can improve it. The number to reduce it is `USTEPS_PER_DEG`,
 * which means a different pulley on J6 - not anything this code can do.
 */
export const SQUARE_ENOUGH_DEG = 0.06;

export interface AxisAlignment {
  /** The nearest axis-aligned attitude, in the frame `R` was expressed in. */
  rotation: number[][];
  /** How far `R` is from it, degrees. Zero when `R` is already square. */
  errorDeg: number;
  /**
   * Where each of the body's axes ends up, as reference-frame axis labels:
   * `['+X', '-Z', '+Y']` reads "body X along reference +X, body Y along
   * reference -Z, body Z along reference +Y".
   *
   * The point of showing it is that "square" does not mean "upright". There are
   * 24 square attitudes and the nearest one may not be the one the operator has
   * in mind, which is a surprise worth having before the arm moves rather than
   * after.
   */
  axes: [string, string, string];
}

/** Angle between two rotations, radians, from the trace identity. */
function angleBetween(A: number[][], B: number[][]): number {
  // trace(A^T B), without forming the product.
  let trace = 0;
  for (let i = 0; i < 3; i++) {
    for (let k = 0; k < 3; k++) trace += A[k][i] * B[k][i];
  }
  // Clamped because a matrix that is orthonormal to a few ulps can put this a
  // hair outside the domain, and acos returns NaN rather than zero for it.
  return Math.acos(Math.max(-1, Math.min(1, (trace - 1) / 2)));
}

/** Label the reference axis a column points along, e.g. `-Z`. */
function axisLabel(column: number[]): string {
  let best = 0;
  for (let r = 1; r < 3; r++) {
    if (Math.abs(column[r]) > Math.abs(column[best])) best = r;
  }
  return `${column[best] < 0 ? '-' : '+'}${'XYZ'[best]}`;
}

/**
 * The nearest attitude in which every axis of `R` lies along a reference axis.
 *
 * `R` is the body's orientation expressed in whichever frame it should be square
 * with - pass it relative to the work object to square it with the fixture, or
 * as it stands to square it with the world. The result is in the same frame.
 *
 * Exhaustive over 24 candidates rather than clever. The set is small, the
 * comparison is a trace, and the alternatives - rounding Euler angles, or
 * rounding each column to the nearest axis - are both wrong: Euler angles round
 * inconsistently near gimbal lock, and independently rounded columns need not be
 * orthogonal, so the "attitude" they describe is not one.
 */
export function nearestAxisAligned(R: number[][]): AxisAlignment {
  let best = AXIS_ALIGNED_ROTATIONS[0];
  let bestAngle = Infinity;

  for (const candidate of AXIS_ALIGNED_ROTATIONS) {
    const angle = angleBetween(candidate, R);
    if (angle < bestAngle) {
      bestAngle = angle;
      best = candidate;
    }
  }

  return {
    rotation: best.map(row => [...row]),
    errorDeg: (bestAngle * 180) / Math.PI,
    axes: [
      axisLabel([best[0][0], best[1][0], best[2][0]]),
      axisLabel([best[0][1], best[1][1], best[2][1]]),
      axisLabel([best[0][2], best[1][2], best[2][2]])
    ]
  };
}

/** How far an attitude is from square, degrees. */
export function alignmentErrorDeg(R: number[][]): number {
  return nearestAxisAligned(R).errorDeg;
}
