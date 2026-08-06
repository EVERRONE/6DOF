import { ForwardKinematics } from '../ForwardKinematics';
import { InverseKinematics } from '../InverseKinematics';

describe('InverseKinematics motion quality behavior', () => {
  const solver = new InverseKinematics({
    maxIterations: 160,
    tolerance: 0.001,
    dampingFactor: 0.01
  });

  test('position-only IK keeps J6 unchanged when already at target', () => {
    const seed = [10, 20, -15, 30, -25, 45];
    const fk = ForwardKinematics.solve(seed);
    expect(fk.success).toBe(true);
    if (!fk.success) return;

    const result = solver.solvePosition(fk.endEffectorPose.position, seed);
    expect(result.success).toBe(true);
    expect(result.jointAngles[5]).toBeCloseTo(seed[5], 3);
  });

  test('pose IK uses wrist DOF for orientation-constrained target', () => {
    const seed = [0, 25, -20, 35, 10, 0];
    const desired = [0, 25, -20, 35, 10, 5];

    const targetFk = ForwardKinematics.solve(desired);
    expect(targetFk.success).toBe(true);
    if (!targetFk.success) return;

    const result = solver.solvePoseWeighted(targetFk.endEffectorPose, seed, {
      orientationWeight: 0.3,
      tolerancePositionM: 0.0015,
      toleranceOrientationRad: 0.03,
      toleranceWeighted: 0.02
    });

    expect(result.success).toBe(true);
    expect(result.quality?.positionResidualM ?? Infinity).toBeLessThanOrEqual(0.002);
    expect(result.quality?.orientationResidualRad ?? Infinity).toBeLessThanOrEqual(0.04);
    expect(Math.abs(result.jointAngles[5] - seed[5])).toBeGreaterThan(1);
  });
});
