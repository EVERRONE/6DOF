import {
  compareStage2StrictCandidates,
  computeAdaptiveStage2TimeoutMs,
  selectBestStage2StrictCandidateIndex
} from '../Stage2RefinementPolicy';

describe('Stage2RefinementPolicy', () => {
  test('computes adaptive timeout clamped between 6000ms and 20000ms', () => {
    const lowComplexity = computeAdaptiveStage2TimeoutMs({
      stage1PointCount: 120,
      stage1LatencyMs: 150,
      stage1DurationSec: 0.8,
      stage2MinPointsPerSecond: 25
    });
    expect(lowComplexity).toBeGreaterThanOrEqual(6000);
    expect(lowComplexity).toBeLessThanOrEqual(20000);

    const highComplexity = computeAdaptiveStage2TimeoutMs({
      stage1PointCount: 20,
      stage1LatencyMs: 4000,
      stage1DurationSec: 4.2,
      stage2MinPointsPerSecond: 25
    });
    expect(highComplexity).toBeGreaterThanOrEqual(lowComplexity);
    expect(highComplexity).toBeLessThanOrEqual(20000);
  });

  test('selects strict candidate by lowest weighted residual then highest pps', () => {
    const candidates = [
      { pointsPerSecond: 25, weightedResidual: 0.0042 },
      { pointsPerSecond: 40, weightedResidual: 0.0042 },
      { pointsPerSecond: 30, weightedResidual: 0.0038 }
    ];
    const selectedIndex = selectBestStage2StrictCandidateIndex(candidates);
    expect(selectedIndex).toBe(2);

    const tieCandidates = [
      { pointsPerSecond: 25, weightedResidual: 0.0042 },
      { pointsPerSecond: 40, weightedResidual: 0.0042 }
    ];
    const tieSelected = selectBestStage2StrictCandidateIndex(tieCandidates);
    expect(tieSelected).toBe(1);
  });

  test('candidate comparator orders by residual then pps', () => {
    const betterResidual = compareStage2StrictCandidates(
      { pointsPerSecond: 25, weightedResidual: 0.002 },
      { pointsPerSecond: 60, weightedResidual: 0.004 }
    );
    expect(betterResidual).toBeLessThan(0);

    const tieResidualHigherPps = compareStage2StrictCandidates(
      { pointsPerSecond: 60, weightedResidual: 0.004 },
      { pointsPerSecond: 25, weightedResidual: 0.004 }
    );
    expect(tieResidualHigherPps).toBeLessThan(0);
  });
});
