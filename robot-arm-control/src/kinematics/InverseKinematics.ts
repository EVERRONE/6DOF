// Inverse Kinematics Implementation using weighted Damped Least Squares
// mathjs SVD removed: svd() does not exist in mathjs v15.
// Singular spectrum computed via Jacobi eigenvalue iteration on J^T*J.
import {
  IKConfig,
  IKQualityMetrics,
  IKResult,
  IKSolveOptions,
  Pose,
  Vector3
} from './types';
import { ForwardKinematics } from './ForwardKinematics';
import { degreesToRadians, getJointLimits, radiansToDegrees } from './DHParameters';
import { QuaternionMath } from './QuaternionMath';
import { SingularityHandler } from './SingularityHandler';
import { CollisionModel } from './CollisionModel';

const DEFAULT_POSE_OPTIONS: Required<IKSolveOptions> = {
  positionWeight: 1.0,
  orientationWeight: 0.25,
  intent: 'endpoint_global',
  mode: 'pose_lock',
  maxStepDeg: 4.0,
  maxJointVelocityDegS: 120.0,
  maxJointAccelerationDegS2: 360.0,
  trackingMaxIterations: 30,
  maxSeeds: 6,
  maxStages: 3,
  minDamping: 0.0005,
  maxDamping: 0.25,
  dampingGrowth: 2.0,
  dampingShrink: 0.75,
  singularityThreshold: 0.0000005,
  postureWeight: 0.0000001,
  tolerancePositionM: 0.0015,
  toleranceOrientationRad: 0.035,
  toleranceWeighted: 0.01,
  computeDiagnostics: true,
  diagnosticStride: 4,
  logDiagnostics: false,
  activeConstraints: [],
  preferredBranch: null,
  previousSolutionDeg: null,
  branchLockEnabled: true,
  boundaryDistanceM: null,
  collisionCheckEnabled: false,
  trackingMode: 'iterative_pose'
};

type SolveState = {
  q: number[];
  metrics: IKQualityMetrics;
  posError: number[];
  orientationError: number[];
  jacobian: number[][];
  collisionFree: boolean;
};

const MIN_REGULARIZATION_SCALE = 1e-12;

/**
 * Inverse Kinematics Solver
 *
 * Uses weighted Jacobian-based Damped Least Squares (Levenberg-Marquardt)
 * with adaptive damping, joint limit barriers, and singularity-aware scaling.
 */
export class InverseKinematics {
  private config: IKConfig;
  private minDeg: number[];
  private maxDeg: number[];

  constructor(config?: Partial<IKConfig>) {
    const limits = getJointLimits();
    this.config = {
      maxIterations: config?.maxIterations || 100,
      tolerance: config?.tolerance || 0.001,
      dampingFactor: config?.dampingFactor || 0.01,
      jointLimits: config?.jointLimits || limits
    };
    this.minDeg = radiansToDegrees(this.config.jointLimits.min);
    this.maxDeg = radiansToDegrees(this.config.jointLimits.max);
  }

  /**
   * Position-only IK, retained for compatibility.
   */
  solvePosition(targetPosition: Vector3, initialGuess?: number[]): IKResult {
    const seed = initialGuess || [0, 0, 0, 0, 0, 0];
    const fkSeed = ForwardKinematics.solve(seed);
    if (!fkSeed.success) {
      return {
        jointAngles: [...seed],
        success: false,
        error: 'Forward kinematics failed',
        failureCategory: 'fk_failure',
        solverPath: 'numeric_fallback'
      };
    }

    const targetPose: Pose = {
      position: targetPosition,
      rotation: fkSeed.endEffectorPose.rotation
    };

    return this.solvePoseWeighted(targetPose, seed, {
      mode: 'position_only',
      orientationWeight: 0,
      postureWeight: 0,
      tolerancePositionM: this.config.tolerance,
      toleranceOrientationRad: Number.POSITIVE_INFINITY,
      toleranceWeighted: this.config.tolerance,
      maxStepDeg: 4.0,
      activeConstraints: ['position']
    });
  }

  /**
   * Full-pose IK with default weighted options.
   */
  solvePose(targetPose: Pose, initialGuess?: number[]): IKResult {
    return this.solvePoseWeighted(targetPose, initialGuess);
  }

  /**
   * Resolved-rate single step for high-rate Cartesian tracking.
   */
  stepResolvedRate(
    targetPose: Pose,
    currentAnglesDeg: number[],
    dtSeconds: number,
    options?: Partial<IKSolveOptions>
  ): IKResult {
    const opts = this.getSolveOptions({
      ...options,
      intent: 'resolved_rate',
      trackingMode: 'resolved_rate'
    });
    const state = this.evaluateState(targetPose, currentAnglesDeg, 0, this.config.dampingFactor, opts, false);
    if (!state) {
      return {
        jointAngles: [...currentAnglesDeg],
        success: false,
        error: 'Forward kinematics failed',
        errorCode: 'FK_FAILED_RESOLVED_RATE',
        stage: 'interpolation',
        failureCategory: 'fk_failure',
        limitsSource: 'firmware',
        angleFrame: 'urdf',
        solverPath: 'numeric_fallback'
      };
    }

    const weightedJacobian = this.applyRowWeights(state.jacobian, opts.positionWeight, opts.orientationWeight);
    const invDt = 1 / Math.max(1e-3, dtSeconds);
    const weightedError = this.scaleErrorVector(state, opts.positionWeight, opts.orientationWeight)
      .map((component) => component * invDt);
    const damping = SingularityHandler.directionalDamping(
      this.config.dampingFactor,
      state.metrics.minSingularValue,
      opts.boundaryDistanceM
    );
    const dq = this.solveDampedLeastSquares(
      weightedJacobian,
      weightedError,
      damping,
      opts.postureWeight,
      currentAnglesDeg,
      this.getJointCentersDeg(),
      0.25
    );
    const stepBound = Math.max(0.2, opts.maxJointVelocityDegS * dtSeconds);
    const next = currentAnglesDeg.map((angle, i) => angle + this.clamp(dq[i], -stepBound, stepBound));
    const wristStabilized = SingularityHandler.stabilizeWristStep(currentAnglesDeg, next);
    const shoulderStabilized = SingularityHandler.stabilizeShoulderStep(
      currentAnglesDeg,
      wristStabilized,
      this.computeJ1PositionSensitivity(state.jacobian)
    );
    const clamped = this.clampJointAngles(shoulderStabilized);
    const nextState = this.evaluateState(targetPose, clamped, 1, damping, opts, false);

    const residualM = nextState?.metrics.positionResidualM ?? Number.POSITIVE_INFINITY;
    const orientationResidualRad = nextState?.metrics.orientationResidualRad ?? Number.POSITIVE_INFINITY;
    const weightedResidual = nextState?.metrics.weightedResidual ?? Number.POSITIVE_INFINITY;
    const success = residualM <= opts.tolerancePositionM;
    const failureCategory: IKResult['failureCategory'] | undefined = success
      ? undefined
      : (residualM > 0.0035 ? 'invalid_target' : 'max_iterations');
    return {
      jointAngles: nextState?.q || clamped,
      success,
      error: success
        ? undefined
        : `Resolved-rate residual too high (pos ${residualM.toFixed(4)}m, ori ${orientationResidualRad.toFixed(4)}rad, weighted ${weightedResidual.toFixed(4)})`,
      errorCode: success ? undefined : 'RESOLVED_RATE_RESIDUAL_HIGH',
      stage: 'interpolation',
      limitsSource: 'firmware',
      angleFrame: 'urdf',
      residualError: residualM,
      quality: nextState?.metrics,
      solverPath: 'numeric_fallback',
      singularityFlags: nextState
        ? SingularityHandler.detectFlags(nextState.q, nextState.metrics.minSingularValue, nextState.metrics.conditionNumber, nextState.jacobian)
        : [],
      limitMarginDeg: this.computeLimitMarginDeg(clamped),
      collisionFree: nextState?.collisionFree,
      failureCategory
    };
  }

  /**
   * Weighted full-pose IK solve with adaptive damping and posture regularization.
   */
  solvePoseWeighted(
    targetPose: Pose,
    initialGuess?: number[],
    options?: Partial<IKSolveOptions>
  ): IKResult {
    if (!this.isFinitePose(targetPose)) {
      return {
        jointAngles: initialGuess ? [...initialGuess] : [0, 0, 0, 0, 0, 0],
        success: false,
        error: 'Invalid target pose',
        errorCode: 'INVALID_TARGET_POSE',
        stage: 'unknown',
        limitsSource: 'firmware',
        angleFrame: 'urdf',
        failureCategory: 'invalid_target',
        solverPath: 'numeric_fallback'
      };
    }

    const opts = this.getSolveOptions(options);
    const trackingLocal = opts.intent === 'tracking_local' || opts.intent === 'resolved_rate';
    const iterationBudget = trackingLocal
      ? Math.min(this.config.maxIterations, Math.max(8, opts.trackingMaxIterations))
      : this.config.maxIterations;
    const diagnosticStride = Math.max(1, opts.diagnosticStride);
    const computeDiagnosticsDefault = opts.computeDiagnostics || !trackingLocal;
    const qCenter = this.getJointCentersDeg();
    let damping = this.clamp(this.config.dampingFactor, opts.minDamping, opts.maxDamping);
    let q = this.clampJointAngles(initialGuess || [0, 0, 0, 0, 0, 0]);
    let wasLimitClamped = false;
    let collisionRejected = false;

    let bestState: SolveState | null = null;
    let failureCategory: IKResult['failureCategory'] = 'max_iterations';
    let cachedMinSingularValue = Number.POSITIVE_INFINITY;

    for (let iteration = 0; iteration < iterationBudget; iteration++) {
      const computeDiagnosticsThisIteration = computeDiagnosticsDefault &&
        (iteration === 0 || iteration % diagnosticStride === 0);

      const current = this.evaluateState(
        targetPose,
        q,
        iteration,
        damping,
        opts,
        computeDiagnosticsThisIteration
      );
      if (!current) {
        return {
          jointAngles: [...q],
          success: false,
          error: 'Forward kinematics failed',
          errorCode: 'FK_FAILED_ITERATIVE',
          stage: 'unknown',
          limitsSource: 'firmware',
          angleFrame: 'urdf',
          failureCategory: 'fk_failure',
          solverPath: 'numeric_fallback'
        };
      }

      // Cache minSingularValue so damping stays coherent when Jacobi is skipped.
      if (computeDiagnosticsThisIteration) {
        cachedMinSingularValue = current.metrics.minSingularValue;
      } else {
        current.metrics.minSingularValue = cachedMinSingularValue;
      }

      if (!bestState || current.metrics.weightedResidual < bestState.metrics.weightedResidual) {
        bestState = current;
      }

      if (!current.collisionFree && opts.collisionCheckEnabled) {
        failureCategory = 'collision';
      }

      if (
        current.metrics.positionResidualM <= opts.tolerancePositionM &&
        current.metrics.orientationResidualRad <= opts.toleranceOrientationRad &&
        current.metrics.weightedResidual <= opts.toleranceWeighted &&
        (!opts.collisionCheckEnabled || current.collisionFree)
      ) {
        const singularityFlags = SingularityHandler.detectFlags(
          current.q,
          current.metrics.minSingularValue,
          current.metrics.conditionNumber,
          current.jacobian
        );
        return {
          jointAngles: SingularityHandler.applyWristBypass(current.q),
          success: true,
          errorCode: 'IK_CONVERGED',
          stage: 'unknown',
          limitsSource: 'firmware',
          angleFrame: 'urdf',
          iterations: iteration,
          residualError: current.metrics.positionResidualM,
          quality: current.metrics,
          solverPath: 'numeric_fallback',
          singularityFlags,
          limitMarginDeg: this.computeLimitMarginDeg(current.q),
          boundaryDistanceM: opts.boundaryDistanceM,
          collisionFree: current.collisionFree
        };
      }

      if (
        current.metrics.minSingularValue <= opts.singularityThreshold ||
        current.metrics.conditionNumber > 1e8
      ) {
        failureCategory = 'singularity';
      }

      const adaptiveDamping = this.clamp(
        SingularityHandler.directionalDamping(damping, current.metrics.minSingularValue, opts.boundaryDistanceM),
        opts.minDamping,
        opts.maxDamping
      );

      const weightedJacobian = this.applyRowWeights(
        current.jacobian,
        opts.positionWeight,
        opts.orientationWeight
      );
      const weightedError = this.scaleErrorVector(current, opts.positionWeight, opts.orientationWeight);
      const limitWeight = current.metrics.positionResidualM > 0.01 ? 0.45 : 0.2;

      const dq = this.solveDampedLeastSquares(
        weightedJacobian,
        weightedError,
        adaptiveDamping,
        opts.postureWeight,
        q,
        qCenter,
        limitWeight
      );

      const velocityBoundStep = Math.max(0.5, opts.maxJointVelocityDegS * 0.05);
      const stepBound = Math.min(opts.maxStepDeg, velocityBoundStep);
      const dqClamped = dq.map((step) => this.clamp(step, -stepBound, stepBound));

      let acceptedCandidate: SolveState | null = null;
      let alpha = 1.0;

      for (let lineStep = 0; lineStep < 4; lineStep++) {
        const trialAngles = Array(6);
        for (let i = 0; i < 6; i++) {
          trialAngles[i] = q[i] + dqClamped[i] * alpha;
        }
        const wristStabilizedTrial = SingularityHandler.stabilizeWristStep(q, trialAngles);
        const stabilizedTrial = SingularityHandler.stabilizeShoulderStep(
          q,
          wristStabilizedTrial,
          this.computeJ1PositionSensitivity(current.jacobian)
        );
        const clamped = this.clampJointAnglesWithFlag(
          stabilizedTrial
        );
        if (clamped.clipped) {
          wasLimitClamped = true;
          if (failureCategory !== 'singularity') {
            failureCategory = 'joint_limit';
          }
        }

        const candidate = this.evaluateState(
          targetPose,
          clamped.angles,
          iteration + 1,
          adaptiveDamping,
          opts,
          computeDiagnosticsThisIteration
        );
        if (!candidate) {
          alpha *= 0.5;
          continue;
        }

        if (opts.collisionCheckEnabled && !candidate.collisionFree) {
          collisionRejected = true;
          alpha *= 0.5;
          continue;
        }

        if (candidate.metrics.weightedResidual < current.metrics.weightedResidual) {
          acceptedCandidate = candidate;
          break;
        }
        alpha *= 0.5;
      }

      if (acceptedCandidate) {
        q = acceptedCandidate.q;
        damping = this.clamp(adaptiveDamping * opts.dampingShrink, opts.minDamping, opts.maxDamping);
      } else {
        damping = this.clamp(adaptiveDamping * opts.dampingGrowth, opts.minDamping, opts.maxDamping);
      }
    }

    const fallback = bestState || this.evaluateState(
      targetPose,
      q,
      iterationBudget,
      damping,
      opts,
      true
    );
    const quality = fallback?.metrics;
    const resultQ = fallback?.q || q;
    const singularityFlags = quality
      ? SingularityHandler.detectFlags(resultQ, quality.minSingularValue, quality.conditionNumber, fallback?.jacobian)
      : [];

    const notes: string[] = [];
    if (opts.logDiagnostics && quality) {
      notes.push(
        `pos=${(quality.positionResidualM * 1000).toFixed(2)}mm, ` +
        `ori=${(quality.orientationResidualRad * 180 / Math.PI).toFixed(2)}deg, ` +
        `cond~${quality.conditionNumber.toFixed(1)} ` +
        `sigmaMin=${quality.minSingularValue.toExponential(2)}`
      );
    }

    if (
      failureCategory === 'max_iterations' &&
      opts.orientationWeight > 0 &&
      (quality?.positionResidualM ?? Infinity) <= opts.tolerancePositionM * 2 &&
      (quality?.orientationResidualRad ?? Infinity) > opts.toleranceOrientationRad
    ) {
      failureCategory = 'orientation_infeasible';
    }
    if (failureCategory === 'max_iterations' && wasLimitClamped) {
      failureCategory = 'joint_limit';
    }
    if (failureCategory === 'max_iterations' && collisionRejected) {
      failureCategory = 'collision';
    }
    if (
      failureCategory === 'max_iterations' &&
      opts.boundaryDistanceM !== undefined &&
      opts.boundaryDistanceM !== null &&
      opts.boundaryDistanceM < 0
    ) {
      failureCategory = 'boundary_clamped';
    }

    return {
      jointAngles: SingularityHandler.applyWristBypass(resultQ),
      success: false,
      error: `Failed to converge (pos ${(quality?.positionResidualM ?? Infinity).toFixed(4)}m, ` +
        `ori ${(quality?.orientationResidualRad ?? Infinity).toFixed(4)}rad)`,
      errorCode: 'IK_MAX_ITERATIONS',
      stage: 'unknown',
      limitsSource: 'firmware',
      angleFrame: 'urdf',
      iterations: quality?.iterations ?? iterationBudget,
      residualError: quality?.positionResidualM,
      quality,
      failureCategory,
      solverPath: 'numeric_fallback',
      singularityFlags,
      limitMarginDeg: this.computeLimitMarginDeg(resultQ),
      boundaryDistanceM: opts.boundaryDistanceM,
      collisionFree: fallback?.collisionFree,
      notes: notes.length > 0 ? notes : undefined
    };
  }

  private evaluateState(
    targetPose: Pose,
    q: number[],
    iteration: number,
    damping: number,
    opts: Required<IKSolveOptions>,
    computeDiagnostics: boolean
  ): SolveState | null {
    const { fk, jacobian } = ForwardKinematics.solveWithJacobian(q);
    if (!fk.success) return null;

    const posError = this.vectorSubtract(targetPose.position, fk.endEffectorPose.position);
    const orientationError = this.computeOrientationErrorVector(
      targetPose.rotation,
      fk.endEffectorPose.rotation
    );

    const positionResidualM = this.vectorNorm(posError);
    const orientationResidualRad = this.vectorNorm(orientationError);
    const weightedResidual = Math.sqrt(
      Math.pow(opts.positionWeight * positionResidualM, 2) +
      Math.pow(opts.orientationWeight * orientationResidualRad, 2)
    );
    const spectrum = computeDiagnostics
      ? this.computeSingularSpectrum(jacobian)
      : { minSingularValue: Number.POSITIVE_INFINITY, conditionNumber: 1.0 };
    const jointParticipation = computeDiagnostics
      ? this.computeJointParticipation(jacobian, opts.positionWeight, opts.orientationWeight)
      : [0, 0, 0, 0, 0, 0];
    const collision = opts.collisionCheckEnabled
      ? CollisionModel.checkSelfCollision(q)
      : { collisionFree: true };

    return {
      q: [...q],
      metrics: {
        positionResidualM,
        orientationResidualRad,
        weightedResidual,
        minSingularValue: spectrum.minSingularValue,
        conditionNumber: spectrum.conditionNumber,
        activeConstraints: [...opts.activeConstraints],
        jointParticipation,
        iterations: iteration,
        damping
      },
      posError,
      orientationError,
      jacobian,
      collisionFree: collision.collisionFree
    };
  }

  private solveDampedLeastSquares(
    weightedJacobian: number[][],
    weightedError: number[],
    damping: number,
    postureWeight: number,
    q: number[],
    qCenter: number[],
    limitWeight: number
  ): number[] {
    const jt = this.transpose(weightedJacobian);
    const jtj = this.multiplyMatrices(jt, weightedJacobian);
    const rhs = this.multiplyMatrixVector(jt, weightedError);

    const n = jtj.length;
    const a = jtj.map((row) => [...row]);
    const b = [...rhs];
    const normalization = this.computeRegularizationScale(jtj);
    const dampingTerm = damping * damping * normalization;
    const postureTerm = postureWeight * normalization;
    const limitGradient = this.computeLimitGradient(q);

    for (let i = 0; i < n; i++) {
      a[i][i] += dampingTerm + postureTerm;
      b[i] += postureTerm * (qCenter[i] - q[i]);
      b[i] -= limitWeight * normalization * limitGradient[i];
    }

    const solved = this.solveLinearSystem(a, b);
    return solved || Array(n).fill(0);
  }

  private computeLimitGradient(q: number[]): number[] {
    const gradient = Array(6).fill(0);
    const MAX_GRADIENT = 50;
    for (let i = 0; i < 6; i++) {
      const range = this.maxDeg[i] - this.minDeg[i];
      if (range <= 0) continue;
      const t = (q[i] - this.minDeg[i]) / range;
      const marginMin = Math.max(1e-3, t);
      const marginMax = Math.max(1e-3, 1 - t);
      const raw = (1 / (marginMin * marginMin)) - (1 / (marginMax * marginMax));
      gradient[i] = this.clamp(raw, -MAX_GRADIENT, MAX_GRADIENT) / range;
    }
    return gradient;
  }

  private solveLinearSystem(a: number[][], b: number[]): number[] | null {
    const n = b.length;
    const m = a.map((row) => [...row]);
    const x = [...b];

    for (let col = 0; col < n; col++) {
      let pivot = col;
      let pivotAbs = Math.abs(m[col][col]);

      for (let row = col + 1; row < n; row++) {
        const value = Math.abs(m[row][col]);
        if (value > pivotAbs) {
          pivotAbs = value;
          pivot = row;
        }
      }

      if (pivotAbs < 1e-12) {
        return null;
      }

      if (pivot !== col) {
        [m[col], m[pivot]] = [m[pivot], m[col]];
        [x[col], x[pivot]] = [x[pivot], x[col]];
      }

      const diag = m[col][col];
      for (let j = col; j < n; j++) {
        m[col][j] /= diag;
      }
      x[col] /= diag;

      for (let row = 0; row < n; row++) {
        if (row === col) continue;
        const factor = m[row][col];
        if (Math.abs(factor) < 1e-16) continue;

        for (let j = col; j < n; j++) {
          m[row][j] -= factor * m[col][j];
        }
        x[row] -= factor * x[col];
      }
    }

    return x;
  }

  private applyRowWeights(jacobian: number[][], posWeight: number, oriWeight: number): number[][] {
    const out = jacobian.map((row) => [...row]);
    for (let r = 0; r < out.length; r++) {
      const w = r < 3 ? posWeight : oriWeight;
      for (let c = 0; c < out[r].length; c++) {
        out[r][c] *= w;
      }
    }
    return out;
  }

  private scaleErrorVector(state: SolveState, posWeight: number, oriWeight: number): number[] {
    return [
      state.posError[0] * posWeight,
      state.posError[1] * posWeight,
      state.posError[2] * posWeight,
      state.orientationError[0] * oriWeight,
      state.orientationError[1] * oriWeight,
      state.orientationError[2] * oriWeight
    ];
  }

  private computeOrientationErrorVector(target: { roll: number; pitch: number; yaw: number }, current: { roll: number; pitch: number; yaw: number }): number[] {
    const qTarget = QuaternionMath.fromEuler(target);
    const qCurrent = QuaternionMath.fromEuler(current);
    const error = QuaternionMath.logMapError(qTarget, qCurrent);
    return [error.x, error.y, error.z];
  }

  private computeSingularSpectrum(jacobian: number[][]): {
    minSingularValue: number;
    conditionNumber: number;
  } {
    if (jacobian.length === 0 || jacobian[0].length === 0) {
      return { minSingularValue: 0, conditionNumber: Number.POSITIVE_INFINITY };
    }
    return this.singularSpectrumFromJtJ(jacobian);
  }

  /**
   * Jacobi eigenvalue iteration on J^T*J to compute singular values.
   * Correct fallback when SVD throws (unlike sqrt(diag(J^T*J)) which is wrong).
   */
  private singularSpectrumFromJtJ(jacobian: number[][]): {
    minSingularValue: number;
    conditionNumber: number;
  } {
    const jt = this.transpose(jacobian);
    const a = this.multiplyMatrices(jt, jacobian);
    const n = a.length;
    if (n === 0) return { minSingularValue: 0, conditionNumber: Number.POSITIVE_INFINITY };

    // Jacobi eigenvalue algorithm for symmetric matrix
    const maxIter = 100;
    for (let sweep = 0; sweep < maxIter; sweep++) {
      let offDiagSum = 0;
      for (let i = 0; i < n; i++) {
        for (let j = i + 1; j < n; j++) {
          offDiagSum += a[i][j] * a[i][j];
        }
      }
      if (offDiagSum < 1e-24) break;

      for (let p = 0; p < n; p++) {
        for (let q = p + 1; q < n; q++) {
          if (Math.abs(a[p][q]) < 1e-14) continue;
          const tau = (a[q][q] - a[p][p]) / (2 * a[p][q]);
          const t = Math.sign(tau) / (Math.abs(tau) + Math.sqrt(1 + tau * tau));
          const c = 1 / Math.sqrt(1 + t * t);
          const s = t * c;

          // Apply Givens rotation
          const app = a[p][p] - t * a[p][q];
          const aqq = a[q][q] + t * a[p][q];
          a[p][p] = app;
          a[q][q] = aqq;
          a[p][q] = 0;
          a[q][p] = 0;

          for (let r = 0; r < n; r++) {
            if (r === p || r === q) continue;
            const arp = a[r][p];
            const arq = a[r][q];
            a[r][p] = c * arp - s * arq;
            a[p][r] = a[r][p];
            a[r][q] = s * arp + c * arq;
            a[q][r] = a[r][q];
          }
        }
      }
    }

    // Diagonal now contains eigenvalues of J^T*J; singular values = sqrt(eigenvalues)
    const singularValues = Array.from({ length: n }, (_, i) => Math.sqrt(Math.max(0, a[i][i])))
      .sort((a, b) => a - b);

    const minSV = singularValues[0] ?? 0;
    const maxSV = singularValues[singularValues.length - 1] ?? 0;
    return {
      minSingularValue: minSV,
      conditionNumber: maxSV / Math.max(minSV, 1e-12)
    };
  }

  private computeRegularizationScale(jtj: number[][]): number {
    if (jtj.length === 0) return MIN_REGULARIZATION_SCALE;
    let trace = 0;
    for (let i = 0; i < jtj.length; i++) {
      trace += Math.max(0, jtj[i][i] ?? 0);
    }
    return Math.max(trace / jtj.length, MIN_REGULARIZATION_SCALE);
  }

  private computeJointParticipation(
    jacobian: number[][],
    posWeight: number,
    oriWeight: number
  ): number[] {
    if (jacobian.length !== 6 || jacobian[0]?.length !== 6) {
      return [0, 0, 0, 0, 0, 0];
    }

    const participation: number[] = [];
    for (let col = 0; col < 6; col++) {
      const px = (jacobian[0][col] || 0) * posWeight;
      const py = (jacobian[1][col] || 0) * posWeight;
      const pz = (jacobian[2][col] || 0) * posWeight;
      const ox = (jacobian[3][col] || 0) * oriWeight;
      const oy = (jacobian[4][col] || 0) * oriWeight;
      const oz = (jacobian[5][col] || 0) * oriWeight;
      participation.push(Math.sqrt(px * px + py * py + pz * pz + ox * ox + oy * oy + oz * oz));
    }

    return participation;
  }

  private computeJ1PositionSensitivity(jacobian: number[][]): number {
    if (jacobian.length < 3 || jacobian[0]?.length < 1) {
      return Number.POSITIVE_INFINITY;
    }
    const x = jacobian[0][0] || 0;
    const y = jacobian[1][0] || 0;
    const z = jacobian[2][0] || 0;
    return Math.sqrt((x * x) + (y * y) + (z * z));
  }

  private getSolveOptions(options?: Partial<IKSolveOptions>): Required<IKSolveOptions> {
    return {
      ...DEFAULT_POSE_OPTIONS,
      ...options
    };
  }

  private computeLimitMarginDeg(angles: number[]): number {
    let minMargin = Number.POSITIVE_INFINITY;
    for (let i = 0; i < 6; i++) {
      const marginMin = Math.abs((angles[i] ?? 0) - this.minDeg[i]);
      const marginMax = Math.abs(this.maxDeg[i] - (angles[i] ?? 0));
      minMargin = Math.min(minMargin, marginMin, marginMax);
    }
    return Number.isFinite(minMargin) ? minMargin : 0;
  }

  private getJointCentersDeg(): number[] {
    return this.minDeg.map((min, i) => (min + this.maxDeg[i]) * 0.5);
  }

  private clampJointAngles(q: number[]): number[] {
    return this.clampJointAnglesWithFlag(q).angles;
  }

  private clampJointAnglesWithFlag(q: number[]): { angles: number[]; clipped: boolean } {
    let clipped = false;
    const angles = q.map((value, i) => {
      const clamped = this.clamp(value, this.minDeg[i], this.maxDeg[i]);
      if (Math.abs(clamped - value) > 1e-9) clipped = true;
      return clamped;
    });
    return { angles, clipped };
  }

  private multiplyMatrices(a: number[][], b: number[][]): number[][] {
    const rowsA = a.length;
    const colsA = a[0].length;
    const colsB = b[0].length;

    const c: number[][] = Array.from({ length: rowsA }, () => Array(colsB).fill(0));
    for (let i = 0; i < rowsA; i++) {
      for (let j = 0; j < colsB; j++) {
        for (let k = 0; k < colsA; k++) {
          c[i][j] += a[i][k] * b[k][j];
        }
      }
    }
    return c;
  }

  private multiplyMatrixVector(a: number[][], x: number[]): number[] {
    const out = Array(a.length).fill(0);
    for (let i = 0; i < a.length; i++) {
      for (let j = 0; j < x.length; j++) {
        out[i] += a[i][j] * x[j];
      }
    }
    return out;
  }

  private transpose(m: number[][]): number[][] {
    const rows = m.length;
    const cols = m[0].length;
    const mt: number[][] = Array.from({ length: cols }, () => Array(rows).fill(0));

    for (let i = 0; i < rows; i++) {
      for (let j = 0; j < cols; j++) {
        mt[j][i] = m[i][j];
      }
    }
    return mt;
  }

  private vectorSubtract(a: Vector3, b: Vector3): number[] {
    return [a.x - b.x, a.y - b.y, a.z - b.z];
  }

  private vectorNorm(v: number[]): number {
    return Math.sqrt(v.reduce((sum, value) => sum + value * value, 0));
  }

  private clamp(value: number, min: number, max: number): number {
    return Math.max(min, Math.min(max, value));
  }

  private isFinitePose(pose: Pose): boolean {
    return (
      Number.isFinite(pose.position.x) &&
      Number.isFinite(pose.position.y) &&
      Number.isFinite(pose.position.z) &&
      Number.isFinite(pose.rotation.roll) &&
      Number.isFinite(pose.rotation.pitch) &&
      Number.isFinite(pose.rotation.yaw)
    );
  }

  /**
   * Update solver configuration
   */
  updateConfig(config: Partial<IKConfig>): void {
    this.config = { ...this.config, ...config };
    this.minDeg = radiansToDegrees(this.config.jointLimits.min);
    this.maxDeg = radiansToDegrees(this.config.jointLimits.max);
  }

  setJointLimitsDeg(minDeg: number[], maxDeg: number[]): void {
    if (!Array.isArray(minDeg) || !Array.isArray(maxDeg) || minDeg.length !== 6 || maxDeg.length !== 6) {
      return;
    }
    this.updateConfig({
      jointLimits: {
        min: degreesToRadians(minDeg),
        max: degreesToRadians(maxDeg)
      }
    });
  }
}
