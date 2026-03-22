import { getJointLimits, radiansToDegrees } from './DHParameters';
import { ForwardKinematics } from './ForwardKinematics';
import { InverseKinematics } from './InverseKinematics';
import {
  CartesianMode,
  IKBranchId,
  IKEngineMode,
  IKResult,
  IKSolveIntent,
  IKSolveOptions,
  IKSolveProfile,
  Pose,
  Vector3
} from './types';
import { AnalyticalPieperIK } from './AnalyticalPieperIK';
import { ContinuityPolicy } from './ContinuityPolicy';
import { QuaternionMath } from './QuaternionMath';
import { CollisionModel } from './CollisionModel';

interface HybridSolveParams {
  mode?: CartesianMode;
  intent?: IKSolveIntent;
  profile?: IKSolveProfile;
  maxSeeds?: number;
  maxStages?: number;
  extraSeeds?: number[][];
  overrideOptions?: Partial<IKSolveOptions>;
}

const clamp = (value: number, min: number, max: number): number => Math.max(min, Math.min(max, value));

const norm2 = (values: number[]): number => Math.sqrt(values.reduce((acc, value) => acc + value * value, 0));

const PROFILE_DEFAULTS: Record<IKSolveProfile, Partial<IKSolveOptions>> = {
  smooth: {
    positionWeight: 1.0,
    orientationWeight: 0.32,
    maxStepDeg: 4.5,
    minDamping: 0.0005,
    maxDamping: 0.3,
    dampingGrowth: 2.0,
    dampingShrink: 0.72,
    postureWeight: 0.0000004,
    tolerancePositionM: 0.0017,
    toleranceOrientationRad: 0.03,
    toleranceWeighted: 0.011,
    singularityThreshold: 0.0000005
  },
  balanced: {
    positionWeight: 1.0,
    orientationWeight: 0.35,
    maxStepDeg: 4.0,
    minDamping: 0.0004,
    maxDamping: 0.25,
    dampingGrowth: 2.0,
    dampingShrink: 0.7,
    postureWeight: 0.0000002,
    tolerancePositionM: 0.0015,
    toleranceOrientationRad: 0.026,
    toleranceWeighted: 0.01,
    singularityThreshold: 0.0000005
  },
  precision: {
    positionWeight: 1.0,
    orientationWeight: 0.4,
    maxStepDeg: 3.5,
    minDamping: 0.0003,
    maxDamping: 0.22,
    dampingGrowth: 2.0,
    dampingShrink: 0.68,
    postureWeight: 0.0000001,
    tolerancePositionM: 0.001,
    toleranceOrientationRad: 0.02,
    toleranceWeighted: 0.008,
    singularityThreshold: 0.0000004
  }
};

/**
 * Hybrid IK V3 orchestrator:
 * - Analytical branch enumeration first
 * - Continuity/branch locking policy
 * - Numeric refinement only when needed
 * - Numeric fallback for degenerate cases
 */
export class HybridIKSolver {
  private readonly solver: InverseKinematics;
  private readonly analyticalSolver: AnalyticalPieperIK;
  private minDeg: number[];
  private maxDeg: number[];
  readonly engineMode: IKEngineMode = 'hybrid_constrained_v2';
  private lastBranchId: IKBranchId | undefined;

  constructor() {
    this.solver = new InverseKinematics({
      maxIterations: 80,
      tolerance: 0.001,
      dampingFactor: 0.01
    });
    this.analyticalSolver = new AnalyticalPieperIK();
    const limits = getJointLimits();
    this.minDeg = radiansToDegrees(limits.min);
    this.maxDeg = radiansToDegrees(limits.max);
  }

  /**
   * Clear persisted branch state before a new endpoint_global move starts.
   * Call this from robotStore before each moveToPosition() to prevent branch
   * contamination from the previous motion's tracking_local interpolation.
   */
  resetBranchState(): void {
    this.lastBranchId = undefined;
  }

  setJointLimitsDeg(minDeg: number[], maxDeg: number[]): void {
    if (!Array.isArray(minDeg) || !Array.isArray(maxDeg) || minDeg.length !== 6 || maxDeg.length !== 6) {
      return;
    }
    this.minDeg = [...minDeg];
    this.maxDeg = [...maxDeg];
    this.solver.setJointLimitsDeg(minDeg, maxDeg);
    this.analyticalSolver.setJointLimitsDeg(minDeg, maxDeg);
  }

  solvePosition(targetPosition: Vector3, initialGuess?: number[]): IKResult {
    const seed = initialGuess || [0, 0, 0, 0, 0, 0];
    const fkSeed = ForwardKinematics.solve(seed);
    if (!fkSeed.success) {
      return {
        jointAngles: [...seed],
        success: false,
        error: 'Forward kinematics failed',
        errorCode: 'FK_FAILED_SOLVEPOSITION',
        stage: 'endpoint',
        limitsSource: 'firmware',
        angleFrame: 'urdf',
        failureCategory: 'fk_failure',
        solverPath: 'numeric_fallback'
      };
    }

    return this.solvePose(
      { position: targetPosition, rotation: fkSeed.endEffectorPose.rotation },
      seed,
      {
        mode: 'position_only',
        profile: 'balanced'
      }
    );
  }

  solvePoseWeighted(targetPose: Pose, initialGuess?: number[], options?: Partial<IKSolveOptions>): IKResult {
    const seed = initialGuess || [0, 0, 0, 0, 0, 0];
    const mode: CartesianMode = options?.mode || ((options?.orientationWeight ?? 0) > 0 ? 'pose_lock' : 'position_only');
    const intent = options?.intent ?? 'endpoint_global';
    return this.solvePose(targetPose, seed, {
      mode,
      intent,
      profile: 'balanced',
      maxSeeds: options?.maxSeeds,
      maxStages: options?.maxStages,
      extraSeeds: [],
      overrideOptions: options
    });
  }

  solvePose(targetPose: Pose, initialGuess: number[], params: HybridSolveParams): IKResult {
    const mode = params.mode || 'pose_lock';
    const intent = params.intent || 'endpoint_global';
    const profile = params.profile || 'balanced';
    const maxStages = intent === 'tracking_local' || intent === 'resolved_rate'
      ? 1
      : (params.maxStages || params.overrideOptions?.maxStages || 3);

    const stageOptions = this.buildStages(mode, profile, params.overrideOptions).slice(0, maxStages);
    const rawPrevious = params.overrideOptions?.previousSolutionDeg;
    const continuityPrevious = (Array.isArray(rawPrevious) && rawPrevious.length === 6)
      ? rawPrevious
      : initialGuess;
    const useInstanceBranch = intent === 'tracking_local' || intent === 'resolved_rate';
    const preferredBranch = params.overrideOptions?.preferredBranch || (useInstanceBranch ? this.lastBranchId : undefined);
    const branchLockEnabled = params.overrideOptions?.branchLockEnabled ?? true;

    const analyticCandidatesRaw = this.analyticalSolver.solveCandidates(
      QuaternionMath.toPoseQuat(targetPose),
      {
        ...params.overrideOptions,
        mode,
        intent,
        preferredBranch,
        previousSolutionDeg: continuityPrevious,
        branchLockEnabled
      }
    );
    const analyticCandidates = ContinuityPolicy.rankCandidates(analyticCandidatesRaw, {
      preferredBranch,
      previousSolutionDeg: continuityPrevious,
      branchLockEnabled
    });

    let bestSuccess: { result: IKResult; source: string; cost: number; branchId?: IKBranchId } | null = null;
    let bestFailure: { result: IKResult; source: string; cost: number; branchId?: IKBranchId } | null = null;

    const maxAnalytic = intent === 'tracking_local' || intent === 'resolved_rate'
      ? 1
      : (params.maxSeeds || params.overrideOptions?.maxSeeds || 6);
    const candidatePool: Array<{ seed: number[]; branchId?: IKBranchId; source: string }> = [];

    for (const candidate of analyticCandidates.slice(0, maxAnalytic)) {
      candidatePool.push({
        seed: this.normalizeToLimits(candidate.jointAngles),
        branchId: candidate.branchId,
        source: 'analytic'
      });
    }

    if (Array.isArray(params.extraSeeds)) {
      for (const extra of params.extraSeeds) {
        candidatePool.push({
          seed: this.normalizeToLimits(extra),
          source: 'extra_seed'
        });
      }
    }

    // Always keep numeric fallback seed path available.
    candidatePool.push({
      seed: this.normalizeToLimits(initialGuess),
      source: 'numeric_fallback_seed'
    });

    for (const stage of stageOptions) {
      for (const candidate of candidatePool) {
        const stageOpts: Partial<IKSolveOptions> = {
          ...stage.options,
          intent,
          mode,
          preferredBranch: candidate.branchId || preferredBranch,
          previousSolutionDeg: continuityPrevious,
          branchLockEnabled
        };
        const result = this.solver.solvePoseWeighted(targetPose, candidate.seed, stageOpts);
        this.decorateQuality(result, candidate.seed, stage.id, candidate.branchId);
        const cost = this.computeCost(result, candidate.seed);

        if (result.success) {
          const branchDiscontinuity = ContinuityPolicy.detectBranchDiscontinuity(
            preferredBranch,
            candidate.branchId,
            branchLockEnabled
          );
          if (branchDiscontinuity && branchLockEnabled) {
            const rejected: IKResult = {
              ...result,
              success: false,
              error: 'Branch switch rejected by continuity lock',
              errorCode: 'BRANCH_LOCK_REJECTED',
              stage: 'endpoint',
              limitsSource: 'firmware',
              angleFrame: 'urdf',
              failureCategory: 'branch_discontinuity',
              solverPath: 'analytic_refined',
              branchId: candidate.branchId
            };
            if (!bestFailure || cost < bestFailure.cost) {
              bestFailure = { result: rejected, source: candidate.source, cost, branchId: candidate.branchId };
            }
            continue;
          }

          const collision = stageOpts.collisionCheckEnabled
            ? CollisionModel.checkSelfCollision(result.jointAngles)
            : {
                collisionFree: true,
                collisionPairs: [],
                minimumClearanceM: Number.POSITIVE_INFINITY
              };
          if (!collision.collisionFree) {
            const rejected: IKResult = {
              ...result,
              success: false,
              error: `Collision detected (${collision.collisionPairs.join(', ')})`,
              errorCode: 'COLLISION_DETECTED',
              stage: 'endpoint',
              limitsSource: 'firmware',
              angleFrame: 'urdf',
              failureCategory: 'collision',
              solverPath: 'analytic_refined',
              collisionFree: false,
              branchId: candidate.branchId
            };
            if (!bestFailure || cost < bestFailure.cost) {
              bestFailure = { result: rejected, source: candidate.source, cost, branchId: candidate.branchId };
            }
            continue;
          }

          const solverPath = candidate.source === 'analytic' ? 'analytic_refined' : 'numeric_fallback';
          const accepted: IKResult = {
            ...result,
            errorCode: result.errorCode || 'IK_CONVERGED',
            stage: 'endpoint',
            limitsSource: 'firmware',
            angleFrame: 'urdf',
            solverPath,
            branchId: candidate.branchId || result.branchId,
            collisionFree: collision.collisionFree
          };
          if (!bestSuccess || cost < bestSuccess.cost) {
            bestSuccess = { result: accepted, source: candidate.source, cost, branchId: candidate.branchId };
          }

          if (stage.id === 'A/base' && candidate.source === 'analytic' && this.isWithinThresholds(accepted, stage.options)) {
            if (useInstanceBranch) this.lastBranchId = accepted.branchId as IKBranchId | undefined;
            return accepted;
          }

          if (intent === 'tracking_local' || intent === 'resolved_rate') {
            if (useInstanceBranch) this.lastBranchId = accepted.branchId as IKBranchId | undefined;
            return accepted;
          }
        } else if (!bestFailure || cost < bestFailure.cost) {
          bestFailure = { result, source: candidate.source, cost, branchId: candidate.branchId };
        }
      }

      if (bestSuccess) break;
    }

    if (bestSuccess) {
      if (useInstanceBranch) this.lastBranchId = bestSuccess.branchId;
      return bestSuccess.result;
    }

    // EDGE-12: Auto-release branch lock when preferred branch becomes unreachable.
    const releasedPos = bestFailure?.result.quality?.positionResidualM ?? Number.POSITIVE_INFINITY;
    if (!bestSuccess && bestFailure?.result.failureCategory === 'branch_discontinuity' && releasedPos <= 0.015) {
      const released: IKResult = {
        ...bestFailure.result,
        success: true,
        error: undefined,
        errorCode: 'BRANCH_LOCK_AUTO_RELEASED',
        stage: 'endpoint',
        limitsSource: 'firmware',
        angleFrame: 'urdf',
        failureCategory: undefined,
        solverPath: bestFailure.result.solverPath,
        notes: [...(bestFailure.result.notes || []), 'branch_lock_auto_released']
      };
      if (useInstanceBranch) this.lastBranchId = bestFailure.branchId;
      return released;
    }

    if (bestFailure) {
      const out: IKResult = { ...bestFailure.result };
      if (out.failureCategory === 'max_iterations') {
        const residual = out.quality?.positionResidualM ?? Number.POSITIVE_INFINITY;
        if (Number.isFinite(residual) && residual > 0.0035) {
          out.failureCategory = 'invalid_target';
        }
      }
      return out;
    }

    return {
      jointAngles: [...initialGuess],
      success: false,
      error: 'Hybrid IK failed to evaluate candidates',
      errorCode: 'HYBRID_NO_CANDIDATE',
      stage: 'endpoint',
      limitsSource: 'firmware',
      angleFrame: 'urdf',
      failureCategory: 'max_iterations',
      solverPath: 'numeric_fallback'
    };
  }

  private buildStages(
    mode: CartesianMode,
    profile: IKSolveProfile,
    overrideOptions?: Partial<IKSolveOptions>
  ): Array<{ id: string; options: Partial<IKSolveOptions> }> {
    const base = {
      ...PROFILE_DEFAULTS[profile],
      ...overrideOptions,
      mode,
      orientationWeight: mode === 'pose_lock'
        ? (overrideOptions?.orientationWeight ?? PROFILE_DEFAULTS[profile].orientationWeight ?? 0.35)
        : 0.0,
      toleranceOrientationRad: mode === 'pose_lock'
        ? (overrideOptions?.toleranceOrientationRad ?? PROFILE_DEFAULTS[profile].toleranceOrientationRad ?? 0.026)
        : Number.POSITIVE_INFINITY,
      activeConstraints: mode === 'pose_lock' ? ['position', 'orientation_hold'] : ['position']
    };

    return [
      { id: 'A/base', options: { ...base } },
      {
        id: 'B/relaxed',
        options: {
          ...base,
          maxStepDeg: Math.min(6.0, (base.maxStepDeg ?? 4.0) + 1.0),
          minDamping: Math.max(0.0001, (base.minDamping ?? 0.0005) * 0.7),
          postureWeight: Math.max(0, (base.postureWeight ?? 0) * 0.2)
        }
      },
      {
        id: 'C/robust',
        options: {
          ...base,
          maxStepDeg: Math.min(7.0, (base.maxStepDeg ?? 4.0) + 1.8),
          minDamping: Math.max(0.00005, (base.minDamping ?? 0.0005) * 0.45),
          postureWeight: 0
        }
      }
    ];
  }

  private computeCost(result: IKResult, seedAngles: number[]): number {
    const pos = result.quality?.positionResidualM ?? 1;
    const ori = result.quality?.orientationResidualRad ?? 1;
    const dqNorm = norm2(result.jointAngles.map((value, index) => value - (seedAngles[index] ?? value)));
    const limitMargin = this.computeJointLimitMarginDeg(result.jointAngles);
    const limitPenalty = 1 / (Math.max(limitMargin, 0.01));
    const manip = result.quality?.minSingularValue ?? 0;
    const continuityPenalty = result.failureCategory === 'branch_discontinuity' ? 5 : 0;
    return (
      (pos * 1200.0) +
      (ori * 90.0) +
      (dqNorm * 0.18) +
      (limitPenalty * 5.5) +
      continuityPenalty -
      (Math.log10(Math.max(manip, 1e-10)) * 0.15)
    );
  }

  private decorateQuality(result: IKResult, seedAngles: number[], stageId: string, branchId?: IKBranchId): void {
    if (!result.quality) return;
    result.quality.retryStage = stageId;
    result.quality.branchId = branchId || result.quality.branchId;
    result.quality.jointLimitMarginDeg = this.computeJointLimitMarginDeg(result.jointAngles);
    result.quality.linearityErrorMm = result.quality.positionResidualM * 1000;
    if (branchId) {
      result.branchId = branchId;
    }
  }

  private computeJointLimitMarginDeg(angles: number[]): number {
    let minMargin = Number.POSITIVE_INFINITY;
    for (let i = 0; i < 6; i++) {
      const marginMin = Math.abs((angles[i] ?? 0) - this.minDeg[i]);
      const marginMax = Math.abs(this.maxDeg[i] - (angles[i] ?? 0));
      minMargin = Math.min(minMargin, marginMin, marginMax);
    }
    return Number.isFinite(minMargin) ? minMargin : 0;
  }

  private normalizeToLimits(angles: number[]): number[] {
    const normalized = [...angles];
    for (let i = 0; i < 6; i++) {
      normalized[i] = clamp(normalized[i] ?? 0, this.minDeg[i], this.maxDeg[i]);
    }
    return normalized;
  }

  private isWithinThresholds(result: IKResult, options: Partial<IKSolveOptions>): boolean {
    if (!result.success || !result.quality) return false;
    const posTol = options.tolerancePositionM ?? 0.0015;
    const oriTol = options.toleranceOrientationRad ?? 0.026;
    const weightedTol = options.toleranceWeighted ?? 0.01;
    return (
      result.quality.positionResidualM <= posTol &&
      result.quality.orientationResidualRad <= oriTol &&
      result.quality.weightedResidual <= weightedTol
    );
  }
}
