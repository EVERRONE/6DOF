// Inverse kinematics: damped least squares with Levenberg-Marquardt damping,
// joint-limit clamping and seed restarts.
//
// The four things that make this converge where the previous version did not:
//
//  1. It solves against the URDF chain (see ForwardKinematics), so the model
//     matches the actual robot instead of a mis-derived DH table.
//  2. Everything is in radians and metres, and the damping is scaled relative
//     to the Jacobian's own magnitude, so the damping term can never swamp the
//     signal the way a fixed lambda^2 did.
//  3. The normal equations are solved by Cholesky factorisation rather than a
//     handful of Gauss-Seidel sweeps that were not guaranteed to converge.
//  4. Joints that run into a limit are locked out of the step and the remaining
//     joints re-solve to compensate, instead of the step being silently
//     truncated by a clamp afterwards.

import { IKResult, Pose, Vector3 } from './types';
import { ForwardKinematics, FKFrames } from './ForwardKinematics';
import {
  distance,
  multiply3,
  normalMatrix,
  normalVector,
  norm,
  rotationLog,
  rpyToMatrix,
  solveSPD,
  sub,
  transpose3
} from './linalg';
import {
  HOME_POSE_DEG,
  JOINT_LIMITS_RAD,
  MID_POSE_DEG,
  NUM_JOINTS,
  clampToLimitsRad,
  degToRad,
  radToDeg
} from './robotModel';

export interface IKOptions {
  /** Iteration budget per seed attempt. */
  maxIterations: number;
  /** Position accuracy required to call it solved, in metres. */
  positionTolerance: number;
  /** Orientation accuracy required, in radians. Only used for pose solves. */
  orientationTolerance: number;
  /**
   * How much a radian of orientation error is worth in metres when both are
   * minimised together. 0.05 means 1 rad counts like 50 mm.
   */
  orientationScale: number;
  /** Largest joint change allowed in one iteration, in radians. */
  maxStepRad: number;
  /** How many seeds to try before giving up. */
  maxSeeds: number;
}

export const DEFAULT_IK_OPTIONS: IKOptions = {
  maxIterations: 150,
  positionTolerance: 0.0005, // 0.5 mm
  orientationTolerance: 0.0087, // 0.5 deg
  orientationScale: 0.05,
  maxStepRad: 0.35, // ~20 deg
  maxSeeds: 6
};

/** Relative damping bounds for the Levenberg-Marquardt loop. */
const DAMPING_INITIAL = 1e-3;
const DAMPING_MIN = 1e-10;
const DAMPING_MAX = 1e4;
/** Absolute floor so the normal matrix stays invertible at a singularity. */
const DAMPING_FLOOR = 1e-14;
/** Damping increases per iteration before giving that iteration up. */
const MAX_DAMPING_TRIALS = 12;

interface Target {
  position: Vector3;
  /** Desired orientation as a rotation matrix, or null for position-only. */
  rotation: number[][] | null;
}

interface Attempt {
  q: number[];
  solved: boolean;
  iterations: number;
  positionError: number;
  orientationError: number;
}

export class InverseKinematics {
  private options: IKOptions;

  constructor(options?: Partial<IKOptions>) {
    this.options = { ...DEFAULT_IK_OPTIONS, ...options };
  }

  updateConfig(options: Partial<IKOptions>): void {
    this.options = { ...this.options, ...options };
  }

  getConfig(): IKOptions {
    return { ...this.options };
  }

  /**
   * Solve for a TCP position, leaving orientation free.
   *
   * @param targetPosition - target in metres, base frame
   * @param initialGuess - seed joint angles in DEGREES, usually the current pose
   * @returns joint angles in DEGREES
   */
  solvePosition(targetPosition: Vector3, initialGuess?: number[]): IKResult {
    return this.run({ position: targetPosition, rotation: null }, initialGuess);
  }

  /**
   * Solve for a full TCP pose, position and orientation.
   *
   * @param targetPose - position in metres, rotation as fixed-axis rpy in radians
   * @param initialGuess - seed joint angles in DEGREES
   * @returns joint angles in DEGREES
   */
  solvePose(targetPose: Pose, initialGuess?: number[]): IKResult {
    return this.run(
      { position: targetPose.position, rotation: rpyToMatrix(targetPose.rotation) },
      initialGuess
    );
  }

  /** True when a position can be reached within tolerance from some seed. */
  isReachable(position: Vector3, initialGuess?: number[]): boolean {
    return this.solvePosition(position, initialGuess).success;
  }

  // -------------------------------------------------------------------------
  // Solver
  // -------------------------------------------------------------------------

  private run(target: Target, initialGuessDeg?: number[]): IKResult {
    if (!Number.isFinite(target.position.x + target.position.y + target.position.z)) {
      return {
        jointAngles: initialGuessDeg ? [...initialGuessDeg] : [...HOME_POSE_DEG],
        success: false,
        error: 'Target position contains non-finite values',
        iterations: 0
      };
    }

    const seeds = this.buildSeeds(initialGuessDeg);
    let totalIterations = 0;
    let best: Attempt | null = null;

    for (const seed of seeds) {
      const attempt = this.solveFromSeed(target, seed);
      totalIterations += attempt.iterations;

      if (attempt.solved) {
        return {
          jointAngles: radToDeg(attempt.q),
          success: true,
          iterations: totalIterations,
          residualError: attempt.positionError
        };
      }

      if (best === null || this.isBetter(attempt, best, target)) {
        best = attempt;
      }
    }

    const fallback = best as Attempt;
    const mm = (fallback.positionError * 1000).toFixed(2);
    const detail =
      target.rotation !== null
        ? `${mm} mm position, ${((fallback.orientationError * 180) / Math.PI).toFixed(2)} deg orientation`
        : `${mm} mm`;

    return {
      jointAngles: radToDeg(fallback.q),
      success: false,
      error: `Target not reachable within tolerance (best residual: ${detail})`,
      iterations: totalIterations,
      residualError: fallback.positionError
    };
  }

  private isBetter(a: Attempt, b: Attempt, target: Target): boolean {
    if (target.rotation === null) {
      return a.positionError < b.positionError;
    }
    const scale = this.options.orientationScale;
    return (
      a.positionError + scale * a.orientationError <
      b.positionError + scale * b.orientationError
    );
  }

  /**
   * Seed list, best guess first. The caller's current pose is tried first
   * because continuity matters for a real arm - a nearby solution avoids a
   * large unnecessary reconfiguration - and the deterministic samples only
   * come into play when the near solution does not exist.
   */
  private buildSeeds(initialGuessDeg?: number[]): number[][] {
    const seeds: number[][] = [];

    if (initialGuessDeg && initialGuessDeg.length === NUM_JOINTS && initialGuessDeg.every(Number.isFinite)) {
      seeds.push(clampToLimitsRad(degToRad(initialGuessDeg)));
    }

    seeds.push(clampToLimitsRad(degToRad(HOME_POSE_DEG)));
    seeds.push(clampToLimitsRad(degToRad(MID_POSE_DEG)));

    // Deterministic low-discrepancy samples inside the limits. Deterministic so
    // that a given target always produces the same solution and tests are
    // reproducible.
    const golden = 0.6180339887498949;
    let acc = golden;
    while (seeds.length < this.options.maxSeeds) {
      const q: number[] = [];
      for (let i = 0; i < NUM_JOINTS; i++) {
        acc = (acc + golden) % 1;
        const lo = JOINT_LIMITS_RAD.min[i];
        const hi = JOINT_LIMITS_RAD.max[i];
        q.push(lo + acc * (hi - lo));
      }
      seeds.push(q);
    }

    return seeds.slice(0, Math.max(1, this.options.maxSeeds));
  }

  private solveFromSeed(target: Target, seedRad: number[]): Attempt {
    const { maxIterations } = this.options;

    let q = clampToLimitsRad(seedRad);
    let fk = ForwardKinematics.solveRad(q);
    let error = this.errorVector(target, fk);
    let cost = this.cost(error);
    let damping = DAMPING_INITIAL;
    let iterations = 0;

    while (iterations < maxIterations) {
      if (this.satisfied(error, target)) break;

      const J = this.weightedJacobian(q, fk, target);
      const e = this.weightedError(error, target);

      let accepted = false;

      for (let trial = 0; trial < MAX_DAMPING_TRIALS; trial++) {
        const dq = this.dampedStep(J, e, damping, q);

        if (dq === null) {
          damping = Math.min(DAMPING_MAX, damping * 10);
          if (damping >= DAMPING_MAX) break;
          continue;
        }

        const qNext = clampToLimitsRad(q.map((v, i) => v + dq[i]));
        const fkNext = ForwardKinematics.solveRad(qNext);
        const errorNext = this.errorVector(target, fkNext);
        const costNext = this.cost(errorNext);
        iterations++;

        if (costNext < cost) {
          q = qNext;
          fk = fkNext;
          error = errorNext;
          cost = costNext;
          // Step worked: trust the local model a bit more next time.
          damping = Math.max(DAMPING_MIN, damping * 0.4);
          accepted = true;
          break;
        }

        // Step overshot: shorten it by damping harder and try again.
        damping = Math.min(DAMPING_MAX, damping * 5);
        if (iterations >= maxIterations) break;
      }

      // No damping value produced an improvement - this seed is at a local
      // minimum or pinned against its limits. Restarting from another seed is
      // more productive than grinding here.
      if (!accepted) break;
    }

    return {
      q,
      solved: this.satisfied(error, target),
      iterations,
      positionError: Math.sqrt(error[0] ** 2 + error[1] ** 2 + error[2] ** 2),
      orientationError:
        error.length > 3 ? Math.sqrt(error[3] ** 2 + error[4] ** 2 + error[5] ** 2) : 0
    };
  }

  /**
   * One damped least squares step, with joint limits handled inside the solve.
   *
   * Solves (J^T J + lambda^2 I) dq = J^T e, where lambda^2 is scaled by the
   * largest diagonal of J^T J so the damping stays meaningful whatever the
   * units and configuration. Any joint that the step would push further past a
   * limit is locked out and the system is re-solved so the other joints take
   * over. Returns null when the factorisation fails, which tells the caller to
   * raise the damping.
   */
  private dampedStep(
    J: number[][],
    e: number[],
    dampingRel: number,
    q: number[]
  ): number[] | null {
    const JtJ = normalMatrix(J);
    const Jte = normalVector(J, e);

    let maxDiag = 0;
    for (let i = 0; i < NUM_JOINTS; i++) {
      maxDiag = Math.max(maxDiag, JtJ[i][i]);
    }
    const lambda2 = dampingRel * maxDiag + DAMPING_FLOOR;

    const locked = Array(NUM_JOINTS).fill(false);
    let dq: number[] | null = null;

    // At most one extra pass per joint: each pass locks at least one more.
    for (let pass = 0; pass <= NUM_JOINTS; pass++) {
      const A: number[][] = JtJ.map(row => [...row]);
      const b = [...Jte];

      for (let i = 0; i < NUM_JOINTS; i++) {
        A[i][i] += lambda2;
      }

      // Dropping column j of J is the same as zeroing row j and column j of
      // J^T J and element j of J^T e. A unit diagonal keeps the matrix
      // positive definite and forces dq[j] to exactly zero.
      for (let j = 0; j < NUM_JOINTS; j++) {
        if (!locked[j]) continue;
        for (let k = 0; k < NUM_JOINTS; k++) {
          A[j][k] = 0;
          A[k][j] = 0;
        }
        A[j][j] = 1;
        b[j] = 0;
      }

      dq = solveSPD(A, b);
      if (dq === null) return null;

      for (let j = 0; j < NUM_JOINTS; j++) {
        if (locked[j]) dq[j] = 0;
      }

      let newlyLocked = false;
      for (let j = 0; j < NUM_JOINTS; j++) {
        if (locked[j]) continue;
        const target = q[j] + dq[j];
        const belowMin = target < JOINT_LIMITS_RAD.min[j] - 1e-12 && dq[j] < 0;
        const aboveMax = target > JOINT_LIMITS_RAD.max[j] + 1e-12 && dq[j] > 0;
        if (belowMin || aboveMax) {
          locked[j] = true;
          newlyLocked = true;
        }
      }

      if (!newlyLocked) break;
    }

    if (dq === null) return null;

    // Trust region: a Jacobian is only a local linearisation, so cap how far
    // one step may travel.
    let maxAbs = 0;
    for (let i = 0; i < NUM_JOINTS; i++) {
      maxAbs = Math.max(maxAbs, Math.abs(dq[i]));
    }
    if (maxAbs > this.options.maxStepRad) {
      const scale = this.options.maxStepRad / maxAbs;
      dq = dq.map(v => v * scale);
    }

    return dq;
  }

  /**
   * Task-space error: [dx, dy, dz] for a position target, or
   * [dx, dy, dz, wx, wy, wz] for a pose target, all unweighted and physical
   * (metres and radians).
   *
   * The orientation part is the matrix logarithm of R_target * R_current^T,
   * i.e. the rotation that still has to happen, expressed as an axis-angle
   * vector in base coordinates. That is a genuine rotation error: subtracting
   * Euler triples, as the old solver did, wraps at +/-pi and blows up at gimbal
   * lock.
   */
  private errorVector(target: Target, fk: FKFrames): number[] {
    const dp = sub(target.position, fk.position);

    if (target.rotation === null) {
      return [dp.x, dp.y, dp.z];
    }

    const Rerr = multiply3(target.rotation, transpose3(fk.rotation));
    const w = rotationLog(Rerr);

    return [dp.x, dp.y, dp.z, w.x, w.y, w.z];
  }

  /** Error vector with the orientation rows scaled into length units. */
  private weightedError(error: number[], target: Target): number[] {
    if (target.rotation === null) return error;
    const s = this.options.orientationScale;
    return [error[0], error[1], error[2], s * error[3], s * error[4], s * error[5]];
  }

  /** Jacobian rows matching the error vector, with the same weighting. */
  private weightedJacobian(q: number[], fk: FKFrames, target: Target): number[][] {
    const J = ForwardKinematics.jacobianRad(q, fk);

    if (target.rotation === null) {
      return J.slice(0, 3);
    }

    const s = this.options.orientationScale;
    return [J[0], J[1], J[2], J[3].map(v => s * v), J[4].map(v => s * v), J[5].map(v => s * v)];
  }

  /** Squared norm of the weighted error - the quantity being minimised. */
  private cost(error: number[]): number {
    const weighted = error.length > 3;
    const s = this.options.orientationScale;
    let sum = error[0] ** 2 + error[1] ** 2 + error[2] ** 2;
    if (weighted) {
      sum += (s * error[3]) ** 2 + (s * error[4]) ** 2 + (s * error[5]) ** 2;
    }
    return sum;
  }

  /** Convergence test on the physical errors, not the weighted ones. */
  private satisfied(error: number[], target: Target): boolean {
    const posError = Math.sqrt(error[0] ** 2 + error[1] ** 2 + error[2] ** 2);
    if (posError > this.options.positionTolerance) return false;

    if (target.rotation === null) return true;

    const oriError = Math.sqrt(error[3] ** 2 + error[4] ** 2 + error[5] ** 2);
    return oriError <= this.options.orientationTolerance;
  }
}

/**
 * Axis-aligned bounding box of the reachable TCP positions, found by sampling
 * the joint space. Used by the UI instead of hardcoded workspace numbers.
 *
 * `samplesPerJoint` grid points per joint on the three joints that dominate
 * position (J1..J3); the wrist joints are sampled more coarsely because they
 * move the TCP much less.
 */
export function computeWorkspaceBounds(samplesPerJoint = 9): {
  min: Vector3;
  max: Vector3;
} {
  const min: Vector3 = { x: Infinity, y: Infinity, z: Infinity };
  const max: Vector3 = { x: -Infinity, y: -Infinity, z: -Infinity };

  const grid = (index: number, steps: number): number[] => {
    const lo = JOINT_LIMITS_RAD.min[index];
    const hi = JOINT_LIMITS_RAD.max[index];
    if (steps <= 1) return [(lo + hi) / 2];
    return Array.from({ length: steps }, (_, k) => lo + ((hi - lo) * k) / (steps - 1));
  };

  const wristSteps = Math.max(2, Math.ceil(samplesPerJoint / 3));
  const axes = [
    grid(0, samplesPerJoint),
    grid(1, samplesPerJoint),
    grid(2, samplesPerJoint),
    grid(3, wristSteps),
    grid(4, wristSteps),
    grid(5, 1) // J6 spins the tool about its own axis; with a zero tool
               // offset it cannot move the TCP at all.
  ];

  for (const q1 of axes[0]) {
    for (const q2 of axes[1]) {
      for (const q3 of axes[2]) {
        for (const q4 of axes[3]) {
          for (const q5 of axes[4]) {
            for (const q6 of axes[5]) {
              const p = ForwardKinematics.solveRad([q1, q2, q3, q4, q5, q6]).position;
              if (p.x < min.x) min.x = p.x;
              if (p.y < min.y) min.y = p.y;
              if (p.z < min.z) min.z = p.z;
              if (p.x > max.x) max.x = p.x;
              if (p.y > max.y) max.y = p.y;
              if (p.z > max.z) max.z = p.z;
            }
          }
        }
      }
    }
  }

  return { min, max };
}

/** Straight-line distance the TCP travels between two joint poses (degrees). */
export function cartesianDistanceBetween(aDeg: number[], bDeg: number[]): number {
  return distance(ForwardKinematics.position(aDeg), ForwardKinematics.position(bDeg));
}

/** Magnitude of a task-space vector, exported for callers that need it. */
export { norm as vectorNorm };
