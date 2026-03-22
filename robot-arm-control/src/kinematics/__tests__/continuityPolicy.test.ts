import { ContinuityPolicy } from '../ContinuityPolicy';
import { AnalyticalIKCandidate } from '../AnalyticalPieperIK';
import { HybridIKSolver } from '../HybridIKSolver';
import { ForwardKinematics } from '../ForwardKinematics';

const candidate = (
  branchId: AnalyticalIKCandidate['branchId'],
  q: number[],
  pos = 0.001,
  ori = 0.01
): AnalyticalIKCandidate => ({
  jointAngles: q,
  branchId,
  positionResidualM: pos,
  orientationResidualRad: ori,
  singularityFlags: [],
  valid: true
});

test('resetBranchState() clears lastBranchId so next endpoint solve starts fresh', () => {
  const solver = new HybridIKSolver();

  // Simulate a tracking_local solve that sets lastBranchId
  const seedA = [0, -20, 40, 90, -45, 0];
  const fkA = ForwardKinematics.solve(seedA);
  expect(fkA.success).toBe(true);
  if (!fkA.success) return;

  solver.solvePose(fkA.endEffectorPose, seedA, {
    mode: 'pose_lock',
    intent: 'tracking_local'
  });

  // The reset method must exist and be callable
  expect(() => solver.resetBranchState()).not.toThrow();

  // After reset, a solve at a completely different target should succeed
  const seedB = [15, -30, 55, -90, 30, 0];
  const fkB = ForwardKinematics.solve(seedB);
  expect(fkB.success).toBe(true);
  if (!fkB.success) return;

  const resultAfterReset = solver.solvePose(fkB.endEffectorPose, seedB, {
    mode: 'pose_lock',
    intent: 'endpoint_global',
    profile: 'balanced'
  });
  expect(resultAfterReset.success).toBe(true);
});

describe('ContinuityPolicy', () => {
  test('prioritizes preferred branch when branch lock is enabled', () => {
    const ranked = ContinuityPolicy.rankCandidates(
      [
        candidate('SR_ED_WF', [10, 10, 10, 10, 10, 10]),
        candidate('SL_EU_WN', [0, 0, 0, 0, 0, 0]),
        candidate('SR_EU_WN', [2, 2, 2, 2, 2, 2])
      ],
      {
        preferredBranch: 'SR_EU_WN',
        previousSolutionDeg: [0, 0, 0, 0, 0, 0],
        branchLockEnabled: true
      }
    );

    expect(ranked[0].branchId).toBe('SR_EU_WN');
  });

  test('detects branch discontinuity only when lock is active', () => {
    expect(ContinuityPolicy.detectBranchDiscontinuity('SL_EU_WN', 'SR_EU_WN', true)).toBe(true);
    expect(ContinuityPolicy.detectBranchDiscontinuity('SL_EU_WN', 'SR_EU_WN', false)).toBe(false);
    expect(ContinuityPolicy.detectBranchDiscontinuity(undefined, 'SR_EU_WN', true)).toBe(false);
  });
});
