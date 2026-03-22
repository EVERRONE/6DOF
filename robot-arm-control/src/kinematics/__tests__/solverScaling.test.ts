import { ForwardKinematics } from '../ForwardKinematics';
import { InverseKinematics } from '../InverseKinematics';
import { HybridIKSolver } from '../HybridIKSolver';

describe('IK normalization scaling', () => {
  test('normalized regularization keeps near-boundary solve stable for practical posture weights', () => {
    const solver = new InverseKinematics({
      maxIterations: 220,
      tolerance: 0.001,
      dampingFactor: 0.01
    });

    const seed = [0, 0, 0, 0, 0, 0];
    const fkSeed = ForwardKinematics.solve(seed);
    expect(fkSeed.success).toBe(true);
    if (!fkSeed.success) return;

    const targetPose = {
      position: { x: -0.22, y: -0.0005, z: 0.311 },
      rotation: fkSeed.endEffectorPose.rotation
    };

    const withPosture = solver.solvePoseWeighted(targetPose, seed, {
      positionWeight: 1.0,
      orientationWeight: 0.35,
      maxStepDeg: 4.0,
      minDamping: 0.0005,
      maxDamping: 0.3,
      dampingGrowth: 2.0,
      dampingShrink: 0.7,
      singularityThreshold: 0.0000005,
      postureWeight: 0.0000002,
      tolerancePositionM: 0.0015,
      toleranceOrientationRad: 0.026,
      toleranceWeighted: 0.01
    });

    const noPosture = solver.solvePoseWeighted(targetPose, seed, {
      positionWeight: 1.0,
      orientationWeight: 0.35,
      maxStepDeg: 4.0,
      minDamping: 0.0005,
      maxDamping: 0.3,
      dampingGrowth: 2.0,
      dampingShrink: 0.7,
      singularityThreshold: 0.0000005,
      postureWeight: 0,
      tolerancePositionM: 0.0015,
      toleranceOrientationRad: 0.026,
      toleranceWeighted: 0.01
    });

    expect(withPosture.success).toBe(true);
    expect(noPosture.success).toBe(true);
    expect((withPosture.quality?.positionResidualM ?? Infinity) * 1000).toBeLessThanOrEqual(1.5);
    expect((noPosture.quality?.positionResidualM ?? Infinity) * 1000).toBeLessThanOrEqual(1.5);
  });

  test('endpoint_global solve completes in under 200ms for a reachable target', () => {
    const solver = new InverseKinematics({ maxIterations: 80, tolerance: 0.001, dampingFactor: 0.01 });
    const seed = [0, 10, 30, 0, 0, 0];
    const fk = ForwardKinematics.solve(seed);
    expect(fk.success).toBe(true);
    if (!fk.success) return;

    const target = fk.endEffectorPose;
    const slightly_off = { ...target, position: { x: target.position.x + 0.02, y: target.position.y, z: target.position.z } };

    const t0 = performance.now();
    const result = solver.solvePoseWeighted(slightly_off, seed, {
      intent: 'endpoint_global',
      diagnosticStride: 4,
      computeDiagnostics: true,
      tolerancePositionM: 0.0015,
      toleranceOrientationRad: 0.035,
      toleranceWeighted: 0.01
    });
    const elapsed = performance.now() - t0;

    expect(result.success).toBe(true);
    expect(elapsed).toBeLessThan(200);
  }, 5000);

  test('HybridIKSolver endpoint_global solve under 300ms for a standard reachable target', () => {
    const solver = new HybridIKSolver();
    const seed = [0, -20, 40, 0, 0, 0];
    const fk = ForwardKinematics.solve(seed);
    expect(fk.success).toBe(true);
    if (!fk.success) return;

    const target = {
      position: { x: fk.endEffectorPose.position.x + 0.03, y: fk.endEffectorPose.position.y, z: fk.endEffectorPose.position.z },
      rotation: fk.endEffectorPose.rotation
    };

    const t0 = performance.now();
    const result = solver.solvePose(target, seed, { mode: 'pose_lock', intent: 'endpoint_global', profile: 'balanced' });
    const elapsed = performance.now() - t0;

    expect(result.success).toBe(true);
    expect((result.quality?.positionResidualM ?? Infinity) * 1000).toBeLessThan(2.0);
    expect(elapsed).toBeLessThan(300);
  }, 5000);
});

