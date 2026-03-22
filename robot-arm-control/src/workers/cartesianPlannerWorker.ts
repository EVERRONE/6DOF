/// <reference lib="webworker" />

import {
  PlannerWorkerRequest,
  PlannerWorkerResponse
} from '../communication/types';
import { CartesianInterpolationResult } from '../motion/types';
import { TrajectoryPlanner } from '../motion/TrajectoryPlanner';
import {
  compareStage2StrictCandidates
} from '../services/cartesian/Stage2RefinementPolicy';
import { Stage2CandidateDiagnostic } from '../services/cartesian/types';

const strictConverged = (
  interpolation: CartesianInterpolationResult,
  tolerancePositionM: number,
  toleranceOrientationRad: number,
  toleranceWeighted: number
): boolean => {
  if (!interpolation.success || !interpolation.lastIK?.quality) {
    return false;
  }
  const q = interpolation.lastIK.quality;
  return (
    q.positionResidualM <= tolerancePositionM &&
    q.orientationResidualRad <= toleranceOrientationRad &&
    q.weightedResidual <= toleranceWeighted
  );
};

const buildPpsCandidates = (
  minPps: number,
  maxPps: number
): number[] => {
  const clampedMin = Math.max(1, Math.floor(minPps));
  const clampedMax = Math.max(clampedMin, Math.floor(maxPps));
  if (clampedMin >= clampedMax) {
    return [clampedMin];
  }

  const candidates = new Set<number>();
  candidates.add(clampedMin);
  candidates.add(clampedMax);

  let probe = clampedMin;
  while (probe < clampedMax) {
    probe = Math.min(clampedMax, Math.round(probe * 1.35));
    candidates.add(probe);
    if (probe === clampedMax) break;
  }

  return Array.from(candidates).sort((a, b) => a - b);
};

const workerScope = globalThis as unknown as DedicatedWorkerGlobalScope;
let cancelledRequestId = -1;

workerScope.onmessage = (event: MessageEvent<PlannerWorkerRequest>) => {
  const message = event.data;
  if (message.type === 'cancel') {
    cancelledRequestId = message.requestId;
    return;
  }

  if (message.type !== 'plan_stage2') {
    return;
  }

  const requestId = message.requestId;
  const payload = message.payload;
  const planner = new TrajectoryPlanner();
  planner.setUrdfOffsets(payload.urdfOffsetsDeg);
  planner.setJointLimitsDeg(payload.jointLimitsUrdfDeg.min, payload.jointLimitsUrdfDeg.max);

  const startedAt = Date.now();
  const responses: string[] = [];
  const candidateDiagnostics: Stage2CandidateDiagnostic[] = [];

  const ppsCandidates = buildPpsCandidates(
    payload.minPointsPerSecond,
    payload.maxPointsPerSecond
  );
  const maxCandidateAttempts = Math.max(1, payload.maxCandidateAttempts ?? 4);
  const maxAdditionalProbesAfterSuccess = Math.max(0, payload.maxAdditionalProbesAfterSuccess ?? 1);
  const deadlineMs = payload.deadlineMs ?? (Date.now() + 6000);
  const selectionPolicy = payload.selectionPolicy ?? 'first_success_plus_one_probe';

  let bestFailure: { error: string; notes: string[]; failureCategory?: string } = {
    error: 'Stage2 refinement failed',
    notes: []
  };
  const strictSuccesses: Array<{
    interpolation: CartesianInterpolationResult;
    pointsPerSecond: number;
    weightedResidual: number;
    candidateIndex: number;
  }> = [];
  let attemptsWithoutStrictSuccess = 0;
  let firstStrictSuccessSeen = false;
  let additionalProbeCount = 0;

  for (let candidateIndex = 0; candidateIndex < ppsCandidates.length; candidateIndex++) {
    const pps = ppsCandidates[candidateIndex];
    if (cancelledRequestId === requestId) {
      return;
    }
    const now = Date.now();
    if (now > deadlineMs) {
      const timeoutMessage = `Stage2 refinement timeout after ${Math.max(0, deadlineMs - startedAt)}ms`;
      responses.push(timeoutMessage);
      bestFailure = {
        error: timeoutMessage,
        notes: [...responses],
        failureCategory: 'stage2_timeout'
      };
      break;
    }

    if (!firstStrictSuccessSeen && attemptsWithoutStrictSuccess >= maxCandidateAttempts) {
      responses.push(`Stopped after ${maxCandidateAttempts} attempts without strict success`);
      break;
    }

    if (firstStrictSuccessSeen && additionalProbeCount >= maxAdditionalProbesAfterSuccess) {
      responses.push('Stopped after one extra probe following first strict success');
      break;
    }

    const progress: PlannerWorkerResponse = {
      type: 'progress',
      requestId,
      elapsedMs: Date.now() - startedAt,
      candidatePps: pps,
      message: `Testing strict refinement at ${pps} Hz`
    };
    workerScope.postMessage(progress);

    const solveStartedAt = Date.now();
    const interpolation = planner.planPoseLockedCartesianMove(
      payload.startAngles,
      payload.targetPosition,
      payload.targetOrientation,
      {
        speedMmS: payload.speedMmS,
        accelerationMmS2: payload.accelerationMmS2,
        pointsPerSecond: pps,
        ikOptions: payload.strictIkOptions,
        ikEngineMode: payload.ikEngineMode,
        deadlineMs
      }
    );
    const solveDurationMs = Date.now() - solveStartedAt;

    const pointCount = interpolation.segment.points.length;
    const remainingBudgetMs = Math.max(0, deadlineMs - Date.now());
    const strictOk = strictConverged(
      interpolation,
      payload.strictIkOptions.tolerancePositionM ?? 0.0015,
      payload.strictIkOptions.toleranceOrientationRad ?? Number.POSITIVE_INFINITY,
      payload.strictIkOptions.toleranceWeighted ?? 0.01
    );
    const interpolationTimedOut = Boolean(
      interpolation.timedOut ||
      interpolation.lastIK?.failureCategory === 'stage2_timeout'
    );

    const candidateDiagnostic: Stage2CandidateDiagnostic = {
      candidateIndex,
      pointsPerSecond: pps,
      sampleCount: pointCount,
      solveDurationMs,
      strictConverged: strictOk && interpolation.success,
      remainingBudgetMs
    };

    if (pointCount > payload.queuePointBudget) {
      const failureReason = `${pps}Hz skipped: ${pointCount} points > budget ${payload.queuePointBudget}`;
      candidateDiagnostic.failureReason = failureReason;
      candidateDiagnostics.push(candidateDiagnostic);
      responses.push(failureReason);
      if (!firstStrictSuccessSeen) {
        bestFailure = {
          error: 'Stage2 refinement exceeded queue point budget',
          notes: [...responses],
          failureCategory: 'queue_upload_failed'
        };
      }
      break;
    }

    if (interpolationTimedOut) {
      const timeoutReason = interpolation.error || `Stage2 refinement timeout at ${pps}Hz`;
      candidateDiagnostic.failureReason = timeoutReason;
      candidateDiagnostics.push(candidateDiagnostic);
      responses.push(`${pps}Hz timeout: ${timeoutReason}`);
      bestFailure = {
        error: timeoutReason,
        notes: [...responses],
        failureCategory: 'stage2_timeout'
      };
      break;
    }

    if (interpolation.success && strictOk) {
      candidateDiagnostics.push(candidateDiagnostic);
      strictSuccesses.push({
        interpolation,
        pointsPerSecond: pps,
        weightedResidual: interpolation.lastIK?.quality?.weightedResidual ?? Number.POSITIVE_INFINITY,
        candidateIndex
      });
      responses.push(`Strict pass at ${pps}Hz`);
      if (!firstStrictSuccessSeen) {
        firstStrictSuccessSeen = true;
        continue;
      }
      additionalProbeCount += 1;
      continue;
    }

    const reason = interpolation.error || interpolation.lastIK?.error || 'No strict solution';
    candidateDiagnostic.failureReason = reason;
    candidateDiagnostics.push(candidateDiagnostic);
    responses.push(`${pps}Hz failed: ${reason}`);
    bestFailure = {
      error: reason,
      notes: [...responses],
      failureCategory: interpolation.lastIK?.failureCategory
    };

    if (!firstStrictSuccessSeen) {
      attemptsWithoutStrictSuccess += 1;
    } else {
      additionalProbeCount += 1;
    }
  }

  if (strictSuccesses.length > 0) {
    let selected = strictSuccesses[0];
    for (let i = 1; i < strictSuccesses.length; i++) {
      if (
        compareStage2StrictCandidates(
          {
            pointsPerSecond: strictSuccesses[i].pointsPerSecond,
            weightedResidual: strictSuccesses[i].weightedResidual
          },
          {
            pointsPerSecond: selected.pointsPerSecond,
            weightedResidual: selected.weightedResidual
          }
        ) < 0
      ) {
        selected = strictSuccesses[i];
      }
    }

    for (let i = 0; i < candidateDiagnostics.length; i++) {
      const diag = candidateDiagnostics[i];
      if (diag.candidateIndex === selected.candidateIndex) {
        diag.selected = true;
        diag.selectionReason = selectionPolicy;
      }
    }

    const result: PlannerWorkerResponse = {
      type: 'result',
      requestId,
      chosenPps: selected.pointsPerSecond,
      interpolation: selected.interpolation,
      notes: [
        ...responses,
        `Stage2 strict refinement complete at ${selected.pointsPerSecond}Hz`,
        `Planning time ${Date.now() - startedAt}ms`
      ],
      diagnostics: candidateDiagnostics
    };
    workerScope.postMessage(result);
    return;
  }

  const fail: PlannerWorkerResponse = {
    type: 'error',
    requestId,
    error: bestFailure.error,
    notes: bestFailure.notes,
    failureCategory: bestFailure.failureCategory,
    diagnostics: candidateDiagnostics
  };
  workerScope.postMessage(fail);
};
