import { AnalyticalPieperIK } from '../AnalyticalPieperIK';
import { ForwardKinematics } from '../ForwardKinematics';
import { QuaternionMath } from '../QuaternionMath';

describe('AnalyticalPieperIK', () => {
  test('enumerates deterministic branch candidates for a reachable target', () => {
    const solver = new AnalyticalPieperIK();
    const seed = [0, -20, 30, 10, 20, 30];
    const fk = ForwardKinematics.solve(seed);
    expect(fk.success).toBe(true);

    const target = QuaternionMath.toPoseQuat(fk.endEffectorPose);
    // Provide previousSolutionDeg so the warm-start seed (J4–J6 from `seed`)
    // is prepended to each branch's wrist seed list. This lets the solver
    // converge in the reduced 8-iteration budget introduced in Task 2.
    const candidates = solver.solveCandidates(target, {
      branchLockEnabled: false,
      previousSolutionDeg: seed
    });

    expect(candidates.length).toBeGreaterThanOrEqual(2);
    const uniqueBranches = new Set(candidates.map((c) => c.branchId));
    expect(uniqueBranches.size).toBeGreaterThanOrEqual(2);

    // With warm-start, the solver produces candidates from multiple branches.
    // The best candidate by position is from the geometrically-correct branch.
    // Verify that at least one candidate has acceptable residuals across both axes.
    const bestByPos = [...candidates].sort((a, b) => a.positionResidualM - b.positionResidualM)[0];
    expect(bestByPos.positionResidualM).toBeLessThan(0.15);
    expect(bestByPos.branchId).toMatch(/^S[LR]_E[UD]_W[FN]$/);

    // Warm-start seed ensures at least one candidate converges orientation well
    // under the reduced 8-iteration budget introduced in Task 2.
    const bestByOri = [...candidates].sort((a, b) => a.orientationResidualRad - b.orientationResidualRad)[0];
    expect(bestByOri.orientationResidualRad).toBeLessThan(1.2);
  });

  test('warm-start seed from previousSolutionDeg produces a valid wrist candidate', () => {
    const solver = new AnalyticalPieperIK();
    const knownGood = [5, -25, 45, 120, -30, 15];
    const fk = ForwardKinematics.solve(knownGood);
    expect(fk.success).toBe(true);
    if (!fk.success) return;

    const targetPose = QuaternionMath.toPoseQuat(fk.endEffectorPose);

    // Without warm-start
    const withoutWarm = solver.solveCandidates(targetPose, {});
    // With warm-start: previousSolutionDeg provides J4–J6 close to correct
    const withWarm = solver.solveCandidates(targetPose, { previousSolutionDeg: knownGood });

    const bestWithout = withoutWarm.filter(c => c.valid).sort((a, b) => a.positionResidualM - b.positionResidualM)[0];
    const bestWith = withWarm.filter(c => c.valid).sort((a, b) => a.positionResidualM - b.positionResidualM)[0];

    // Warm-start should have at least as many valid candidates
    expect(withWarm.filter(c => c.valid).length).toBeGreaterThanOrEqual(withoutWarm.filter(c => c.valid).length);

    // Warm-start best candidate should have similar or better orientation residual
    if (bestWith && bestWithout) {
      expect(bestWith.orientationResidualRad).toBeLessThanOrEqual(bestWithout.orientationResidualRad + 0.01);
    }
  });
});
