// Finding where the tool tip is, by touching one point from several directions.
//
// The software's idea of "the tool" is the origin of frame 6 - the flange face -
// until somebody tells it otherwise. Everything measured with an untold tool is
// then measured to the wrong point: a work object taught by touching three
// corners comes out both shifted and skewed, because the offset from flange to
// tip points a different way at every touch.
//
// The offset could be typed in, but only by somebody who knows which way frame
// 6's X, Y and Z point - and that is not visible on the machine, it falls out of
// a chain of URDF rotations. Touching a fixed point from four directions asks
// nothing of the operator but the touching.
//
// The method every industrial controller has, under one name or another.

import { Vector3 } from './types';
import { multiply3, normalMatrix, normalVector, rotationLog, solveSPD, transpose3 } from './linalg';

/** One touch: where the flange was, and how it was turned. */
export interface ToolTouch {
  /** Flange origin in base coordinates, metres. Frame 6, not the TCP. */
  position: Vector3;
  /** Flange orientation in base coordinates. */
  rotation: number[][];
}

export type ToolCalibration =
  | {
      ok: true;
      /** The tool tip in frame 6 coordinates, metres. */
      offset: Vector3;
      /** Where the touched point turned out to be, in base coordinates. */
      point: Vector3;
      /**
       * How far the touches disagree, in millimetres - the largest distance
       * between where a touch says the tip was and where the solution puts it.
       *
       * This is the number worth reading. It is the operator's aim, the arm's
       * repeatability and the model's fidelity added together, and nothing else
       * measured so far reports any of them.
       */
      residualMm: number;
      /** Largest angle between any two of the touch orientations, degrees. */
      spreadDeg: number;
    }
  | { ok: false; reason: string };

/** Angle between two orientations, in degrees. */
function angleBetween(a: number[][], b: number[][]): number {
  const w = rotationLog(multiply3(a, transpose3(b)));
  return (Math.hypot(w.x, w.y, w.z) * 180) / Math.PI;
}

/**
 * How different the touch orientations have to be before the answer means
 * anything.
 *
 * The whole method rests on the flange being turned differently at each touch:
 * that is what makes the offset the only unknown consistent with all of them.
 * Touch the same point four times from nearly the same angle and every offset
 * along one direction fits equally well - the solve still returns a number, and
 * the number is noise.
 *
 * 20 degrees is enough for the system to be decently conditioned without asking
 * for poses the arm may not reach.
 */
const MIN_SPREAD_DEG = 20;

/**
 * Solve for the tool tip from touches of one fixed point.
 *
 * Each touch says the tip was at the same unknown world point P:
 *
 *   P = p_i + R_i · t
 *
 * with `t` the unknown offset in frame 6. Rearranged, `R_i · t − P = −p_i`,
 * which is linear in the six unknowns `[t, P]` - three for the tool, three for
 * where the point turned out to be. Four touches give twelve equations for six
 * unknowns, and least squares does the rest.
 *
 * Three touches would be the minimum. Four is asked for because the fourth is
 * what makes the residual mean something: with the minimum, the solve fits
 * exactly and reports zero error whether or not the touches were any good.
 */
export function solveToolOffset(touches: ToolTouch[]): ToolCalibration {
  if (touches.length < 4) {
    return { ok: false, reason: `Need four touches, have ${touches.length}` };
  }

  let spread = 0;
  for (let i = 0; i < touches.length; i++) {
    for (let j = i + 1; j < touches.length; j++) {
      spread = Math.max(spread, angleBetween(touches[i].rotation, touches[j].rotation));
    }
  }

  if (spread < MIN_SPREAD_DEG) {
    return {
      ok: false,
      reason:
        `The touches were only ${spread.toFixed(0)}° apart in orientation. Turn the ` +
        'tool more between them — approach the point from genuinely different ' +
        'directions, or the answer is noise however small the error looks.'
    };
  }

  // [ R_i | -I ] [t; P] = -p_i, stacked.
  const A: number[][] = [];
  const b: number[] = [];

  for (const touch of touches) {
    for (let r = 0; r < 3; r++) {
      A.push([
        touch.rotation[r][0], touch.rotation[r][1], touch.rotation[r][2],
        r === 0 ? -1 : 0, r === 1 ? -1 : 0, r === 2 ? -1 : 0
      ]);
    }
    b.push(-touch.position.x, -touch.position.y, -touch.position.z);
  }

  const solved = solveSPD(normalMatrix(A), normalVector(A, b));
  if (!solved) {
    return {
      ok: false,
      reason: 'The touches do not determine a tool offset — they are too alike.'
    };
  }

  const offset: Vector3 = { x: solved[0], y: solved[1], z: solved[2] };
  const point: Vector3 = { x: solved[3], y: solved[4], z: solved[5] };

  // Where each touch says the tip was, against where the solution puts it.
  let residual = 0;
  for (const touch of touches) {
    const tip = {
      x: touch.position.x + touch.rotation[0][0] * offset.x + touch.rotation[0][1] * offset.y + touch.rotation[0][2] * offset.z,
      y: touch.position.y + touch.rotation[1][0] * offset.x + touch.rotation[1][1] * offset.y + touch.rotation[1][2] * offset.z,
      z: touch.position.z + touch.rotation[2][0] * offset.x + touch.rotation[2][1] * offset.y + touch.rotation[2][2] * offset.z
    };
    residual = Math.max(
      residual,
      Math.hypot(tip.x - point.x, tip.y - point.y, tip.z - point.z) * 1000
    );
  }

  // A tool the length of the arm is a mis-touch, not a tool, and accepting it
  // would move the whole reachable workspace with it.
  if (Math.hypot(offset.x, offset.y, offset.z) > 0.5) {
    return {
      ok: false,
      reason:
        'That works out to a tool more than 500 mm from the flange. Check that ' +
        'every touch was on the same physical point.'
    };
  }

  return { ok: true, offset, point, residualMm: residual, spreadDeg: spread };
}
