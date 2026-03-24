import { getJointLimits, radiansToDegrees } from './DHParameters';
import { ForwardKinematics } from './ForwardKinematics';
import { QuaternionMath } from './QuaternionMath';
import {
  IKBranchId,
  IKSolveOptions,
  PoseQuat
} from './types';

export interface AnalyticalIKCandidate {
  jointAngles: number[];
  branchId: IKBranchId;
  positionResidualM: number;
  orientationResidualRad: number;
  singularityFlags: string[];
  valid: boolean;
  notes?: string[];
}

const DEG_TO_RAD = Math.PI / 180;
const RAD_TO_DEG = 180 / Math.PI;

const clamp = (value: number, min: number, max: number): number => (
  Math.min(max, Math.max(min, value))
);

const wrapDeg = (value: number): number => {
  if (!Number.isFinite(value)) return 0;
  return ((value % 360) + 540) % 360 - 180;
};

const norm3 = (x: number, y: number, z: number): number => Math.hypot(x, y, z);

type BranchSpec = {
  shoulder: 'L' | 'R';
  elbow: 'U' | 'D';
};

const BRANCH_SPECS: BranchSpec[] = [
  { shoulder: 'L', elbow: 'U' },
  { shoulder: 'L', elbow: 'D' },
  { shoulder: 'R', elbow: 'U' },
  { shoulder: 'R', elbow: 'D' }
];

const makeBranchId = (shoulder: 'L' | 'R', elbow: 'U' | 'D', wristFlip: 'F' | 'N'): IKBranchId => (
  `S${shoulder}_E${elbow}_W${wristFlip}` as IKBranchId
);

// Geometric constants derived from URDF joint origins:
//   BASE_HEIGHT = J1.z + J2.z = 0.08 + 0.05595 = 0.13595
//   RADIAL_OFFSET = hypot(J2.x, J2.y) = hypot(0.0375, 0.02) ≈ 0.0425
//   L1 = |J3.y in J2 frame| = 0.16 (upper arm)
//   L2 ≈ 0.1434 (forearm: J3→wrist center distance in planar projection)
const BASE_HEIGHT = 0.13595;
const RADIAL_OFFSET = 0.0425;
const L1 = 0.16;
const L2 = 0.1434;

/**
 * Analytical-first branch generator:
 * - closed-form decoupled estimate for J1-J3
 * - deterministic wrist branch expansion (normal/flip)
 * - optional local wrist refinement on J4-J6
 */
export class AnalyticalPieperIK {
  private minDeg: number[];
  private maxDeg: number[];

  constructor() {
    const limits = getJointLimits();
    this.minDeg = radiansToDegrees(limits.min);
    this.maxDeg = radiansToDegrees(limits.max);
  }

  setJointLimitsDeg(minDeg: number[], maxDeg: number[]): void {
    if (!Array.isArray(minDeg) || !Array.isArray(maxDeg) || minDeg.length !== 6 || maxDeg.length !== 6) {
      return;
    }
    this.minDeg = [...minDeg];
    this.maxDeg = [...maxDeg];
  }

  solveCandidates(
    targetPose: PoseQuat,
    options?: Partial<IKSolveOptions>
  ): AnalyticalIKCandidate[] {
    const targetPos = targetPose.position;
    const targetOrientation = targetPose.orientation;
    const baseYaw = Math.atan2(targetPos.y, targetPos.x) * RAD_TO_DEG;

    const candidates: AnalyticalIKCandidate[] = [];
    for (const branch of BRANCH_SPECS) {
      const q1Deg = wrapDeg(baseYaw + (branch.shoulder === 'R' ? 180 : 0));
      const q1Rad = q1Deg * DEG_TO_RAD;
      // Project target into J1-rotated frame for correct shoulder-right geometry.
      const radialInFrame = Math.cos(q1Rad) * targetPos.x + Math.sin(q1Rad) * targetPos.y;
      const planar = this.solvePlanarShoulderElbow(radialInFrame, targetPos.z, branch.elbow);
      if (!planar) continue;

      const q2Deg = planar.q2Deg;
      const q3Deg = planar.q3Deg;

      const wristSeeds: Array<{ q: [number, number, number]; wristFlip: 'N' | 'F' }> = [
        { q: [0, 0, 0], wristFlip: 'N' },
        { q: [180, 0, 180], wristFlip: 'F' }
      ];

      // Warm-start: if a previous solution is available, insert its J4–J6 as a
      // third seed. This is almost always the best seed during trajectory tracking
      // and costs nothing extra — it replaces a bad cold-start with a nearby guess.
      const prevSol = options?.previousSolutionDeg;
      if (Array.isArray(prevSol) && prevSol.length === 6) {
        const prevJ5 = prevSol[4] ?? 0;
        // Always label the warm-start seed as 'N'. The refine loop corrects
        // wrist flip naturally; a wrong label only costs one extra refine iter.
        wristSeeds.unshift({ q: [prevSol[3] ?? 0, prevJ5, prevSol[5] ?? 0], wristFlip: 'N' });
      }

      for (const wristSeed of wristSeeds) {
        const qBase = [q1Deg, q2Deg, q3Deg, wristSeed.q[0], wristSeed.q[1], wristSeed.q[2]];
        const wrist = this.refineWristOrientation(targetOrientation, qBase, options, targetPos);
        const jointAngles = this.applyLimitEnvelope(wrist.angles);
        const fk = ForwardKinematics.solve(jointAngles);
        if (!fk.success) continue;

        const posErr = norm3(
          targetPos.x - fk.endEffectorPose.position.x,
          targetPos.y - fk.endEffectorPose.position.y,
          targetPos.z - fk.endEffectorPose.position.z
        );
        const oriErrVec = QuaternionMath.logMapError(
          targetOrientation,
          QuaternionMath.fromEuler(fk.endEffectorPose.rotation)
        );
        const orientationResidual = norm3(oriErrVec.x, oriErrVec.y, oriErrVec.z);
        const singularityFlags: string[] = [];
        if (Math.abs(Math.sin((jointAngles[4] || 0) * DEG_TO_RAD)) < 0.02) {
          singularityFlags.push('wrist_pitch_near_singularity');
        }

        const branchId = makeBranchId(branch.shoulder, branch.elbow, wristSeed.wristFlip);
        candidates.push({
          jointAngles,
          branchId,
          positionResidualM: posErr,
          orientationResidualRad: orientationResidual,
          singularityFlags,
          valid: posErr <= 0.05,
          notes: wrist.notes
        });
      }
    }

    // Sorting delegated to ContinuityPolicy.rankCandidates in HybridIKSolver.
    return candidates;
  }

  private solvePlanarShoulderElbow(
    radialInFrame: number,
    z: number,
    elbow: 'U' | 'D'
  ): { q2Deg: number; q3Deg: number } | null {
    // Planar 2R solve in the J1-rotated shoulder plane.
    const radial = Math.max(0.02, radialInFrame - RADIAL_OFFSET);
    const vertical = z - BASE_HEIGHT;
    const cosElbow = clamp(
      (radial * radial + vertical * vertical - L1 * L1 - L2 * L2) / (2 * L1 * L2),
      -1,
      1
    );
    if (!Number.isFinite(cosElbow)) return null;

    const elbowMag = Math.acos(cosElbow);
    const q3 = elbow === 'D' ? elbowMag : -elbowMag;
    const q2 = Math.atan2(vertical, radial) - Math.atan2(L2 * Math.sin(q3), L1 + L2 * Math.cos(q3));
    return {
      q2Deg: q2 * RAD_TO_DEG,
      q3Deg: q3 * RAD_TO_DEG
    };
  }

  private refineWristOrientation(
    targetOrientation: { w: number; x: number; y: number; z: number },
    seedAnglesDeg: number[],
    options?: Partial<IKSolveOptions>,
    targetPosition?: { x: number; y: number; z: number }
  ): { angles: number[]; notes: string[] } {
    const angles = [...seedAnglesDeg];
    const maxIter = 8;
    const gain = 0.9;
    const maxStep = options?.maxStepDeg ?? 6;
    const notes: string[] = [];

    // Measure initial position error for drift guard (EDGE-07).
    const fkInit = ForwardKinematics.solve(angles);
    const initialPosErr = (fkInit.success && targetPosition)
      ? norm3(
          targetPosition.x - fkInit.endEffectorPose.position.x,
          targetPosition.y - fkInit.endEffectorPose.position.y,
          targetPosition.z - fkInit.endEffectorPose.position.z
        )
      : Number.POSITIVE_INFINITY;
    const posErrCeiling = initialPosErr * 1.5 + 0.005;

    for (let iter = 0; iter < maxIter; iter++) {
      // Single-pass: obtain FK + analytic Jacobian with one chain traversal.
      const { fk, jacobian: fullJ } = ForwardKinematics.solveWithJacobian(angles);
      if (!fk.success) break;

      const currentQ = QuaternionMath.fromEuler(fk.endEffectorPose.rotation);
      const err = QuaternionMath.logMapError(targetOrientation, currentQ);
      const errNorm = norm3(err.x, err.y, err.z);
      if (errNorm < 0.01) break;

      // Analytic orientation Jacobian: extract the angular sub-block (rows 3-5, cols 3-5).
      // J[row+3][col+3] = axisWorld[col] * DEG_TO_RAD for the angular component, which
      // equals the central-difference approximation to first order.
      const j = [
        [fullJ[3][3], fullJ[3][4], fullJ[3][5]],
        [fullJ[4][3], fullJ[4][4], fullJ[4][5]],
        [fullJ[5][3], fullJ[5][4], fullJ[5][5]]
      ];

      const jt = this.transpose3(j);
      const a = this.mul3(jt, j);
      const lambda = 1e-3 + (errNorm > 0.1 ? 5e-3 : 0);
      for (let i = 0; i < 3; i++) {
        a[i][i] += lambda;
      }
      const b = this.mulVec3(jt, [err.x, err.y, err.z]);
      const dq = this.solveLinear3(a, b);
      if (!dq) break;

      const prevAngles = [...angles];
      for (let i = 0; i < 3; i++) {
        const step = clamp(dq[i] * gain, -maxStep, maxStep);
        angles[i + 3] = wrapDeg(angles[i + 3] + step);
      }

      // Position drift guard: if wrist step degrades position, halve the step.
      if (targetPosition) {
        const fkAfter = ForwardKinematics.solve(angles);
        if (fkAfter.success) {
          const posErr = norm3(
            targetPosition.x - fkAfter.endEffectorPose.position.x,
            targetPosition.y - fkAfter.endEffectorPose.position.y,
            targetPosition.z - fkAfter.endEffectorPose.position.z
          );
          if (posErr > posErrCeiling) {
            // Halve step: average with previous
            for (let i = 3; i < 6; i++) {
              angles[i] = wrapDeg((prevAngles[i] + angles[i]) * 0.5);
            }
            notes.push('wrist_position_drift_damped');
          }
        }
      }
    }

    const j5Sin = Math.abs(Math.sin((angles[4] || 0) * DEG_TO_RAD));
    if (j5Sin < 0.015) {
      // Deterministic wrist bypass: preserve combined roll around singular coupling.
      const combined = wrapDeg((angles[3] || 0) + (angles[5] || 0));
      angles[3] = combined;
      angles[5] = 0;
      notes.push('wrist_singularity_bypass_applied');
    }

    return { angles, notes };
  }

  private applyLimitEnvelope(angles: number[]): number[] {
    const out = [...angles];
    for (let i = 0; i < 6; i++) {
      out[i] = clamp(wrapDeg(out[i] || 0), this.minDeg[i], this.maxDeg[i]);
    }
    return out;
  }

  private transpose3(m: number[][]): number[][] {
    return [
      [m[0][0], m[1][0], m[2][0]],
      [m[0][1], m[1][1], m[2][1]],
      [m[0][2], m[1][2], m[2][2]]
    ];
  }

  private mul3(a: number[][], b: number[][]): number[][] {
    const out = Array.from({ length: 3 }, () => Array(3).fill(0));
    for (let i = 0; i < 3; i++) {
      for (let j = 0; j < 3; j++) {
        out[i][j] = a[i][0] * b[0][j] + a[i][1] * b[1][j] + a[i][2] * b[2][j];
      }
    }
    return out;
  }

  private mulVec3(a: number[][], x: number[]): number[] {
    return [
      a[0][0] * x[0] + a[0][1] * x[1] + a[0][2] * x[2],
      a[1][0] * x[0] + a[1][1] * x[1] + a[1][2] * x[2],
      a[2][0] * x[0] + a[2][1] * x[1] + a[2][2] * x[2]
    ];
  }

  private solveLinear3(a: number[][], b: number[]): number[] | null {
    // Gaussian elimination with partial pivoting (small 3x3 fallback).
    const m = a.map((row) => [...row]);
    const v = [...b];
    for (let col = 0; col < 3; col++) {
      let pivot = col;
      let maxAbs = Math.abs(m[col][col]);
      for (let row = col + 1; row < 3; row++) {
        const abs = Math.abs(m[row][col]);
        if (abs > maxAbs) {
          maxAbs = abs;
          pivot = row;
        }
      }
      if (maxAbs < 1e-10) return null;
      if (pivot !== col) {
        [m[col], m[pivot]] = [m[pivot], m[col]];
        [v[col], v[pivot]] = [v[pivot], v[col]];
      }

      const diag = m[col][col];
      for (let j = col; j < 3; j++) m[col][j] /= diag;
      v[col] /= diag;
      for (let row = 0; row < 3; row++) {
        if (row === col) continue;
        const factor = m[row][col];
        for (let j = col; j < 3; j++) m[row][j] -= factor * m[col][j];
        v[row] -= factor * v[col];
      }
    }
    return v;
  }
}
