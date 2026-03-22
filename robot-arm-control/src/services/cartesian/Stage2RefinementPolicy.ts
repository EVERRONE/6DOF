export interface AdaptiveStage2TimeoutInput {
  stage1PointCount: number;
  stage1LatencyMs: number;
  stage1DurationSec: number;
  stage2MinPointsPerSecond: number;
  minTimeoutMs?: number;
  maxTimeoutMs?: number;
}

export interface Stage2StrictCandidateScore {
  pointsPerSecond: number;
  weightedResidual: number;
}

const clamp = (value: number, min: number, max: number): number => (
  Math.min(max, Math.max(min, value))
);

export const computeAdaptiveStage2TimeoutMs = (
  input: AdaptiveStage2TimeoutInput
): number => {
  const minTimeoutMs = input.minTimeoutMs ?? 6000;
  const maxTimeoutMs = input.maxTimeoutMs ?? 20000;
  const stage1PointCount = Math.max(1, Math.floor(input.stage1PointCount || 1));
  const stage1LatencyMs = Math.max(1, input.stage1LatencyMs || 1);
  const stage1DurationSec = Math.max(0.001, input.stage1DurationSec || 0.001);
  const stage2MinPps = Math.max(1, input.stage2MinPointsPerSecond || 1);

  const costPerPointMs = clamp(stage1LatencyMs / stage1PointCount, 1, 40);
  const stage2BasePoints = Math.max(2, Math.ceil(stage1DurationSec * stage2MinPps));
  const predictedOneCandidateMs = stage2BasePoints * costPerPointMs * 1.35;
  const predictedTotalMs = (predictedOneCandidateMs * 2) + 500;
  return Math.round(clamp(predictedTotalMs, minTimeoutMs, maxTimeoutMs));
};

export const compareStage2StrictCandidates = (
  a: Stage2StrictCandidateScore,
  b: Stage2StrictCandidateScore
): number => {
  const aResidual = Number.isFinite(a.weightedResidual) ? a.weightedResidual : Number.POSITIVE_INFINITY;
  const bResidual = Number.isFinite(b.weightedResidual) ? b.weightedResidual : Number.POSITIVE_INFINITY;
  if (Math.abs(aResidual - bResidual) > 1e-12) {
    return aResidual < bResidual ? -1 : 1;
  }
  if (a.pointsPerSecond !== b.pointsPerSecond) {
    return a.pointsPerSecond > b.pointsPerSecond ? -1 : 1;
  }
  return 0;
};

export const selectBestStage2StrictCandidateIndex = (
  candidates: Stage2StrictCandidateScore[]
): number => {
  if (!Array.isArray(candidates) || candidates.length === 0) {
    return -1;
  }
  let best = 0;
  for (let i = 1; i < candidates.length; i++) {
    if (compareStage2StrictCandidates(candidates[i], candidates[best]) < 0) {
      best = i;
    }
  }
  return best;
};

export const isStage2TimeoutMessage = (message: string): boolean => (
  /stage2\s+refinement\s+timeout|deadline\s+exceeded|stg2_timeout|stage2_timeout/i.test(message)
);
