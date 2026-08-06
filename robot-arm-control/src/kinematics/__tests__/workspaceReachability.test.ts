import { ForwardKinematics } from '../ForwardKinematics';
import { PathInterpolator } from '../../motion/PathInterpolator';

describe('Workspace reachability regression', () => {
  test('reported X-axis targets are reachable in both cartesian modes', () => {
    const interpolator = new PathInterpolator();
    const startAngles = [0, 0, 0, 0, 0, 0];
    const fkStart = ForwardKinematics.solve(startAngles);
    expect(fkStart.success).toBe(true);
    if (!fkStart.success) return;

    const targets = [
      { x: -0.22, y: -0.0005, z: 0.311 },
      { x: -0.25, y: -0.0005, z: 0.311 }
    ];

    for (const target of targets) {
      const poseLock = interpolator.interpolateCartesianSpace(
        startAngles,
        target,
        20,
        60,
        100,
        fkStart.endEffectorPose.rotation,
        {
          positionWeight: 1.0,
          orientationWeight: 0.35,
          maxStepDeg: 4.0,
          maxJointVelocityDegS: 110,
          maxJointAccelerationDegS2: 320,
          minDamping: 0.0005,
          maxDamping: 0.3,
          dampingGrowth: 2.0,
          dampingShrink: 0.7,
          singularityThreshold: 0.0000005,
          postureWeight: 0.0000002,
          tolerancePositionM: 0.0015,
          toleranceOrientationRad: 0.026,
          toleranceWeighted: 0.01,
          activeConstraints: ['position', 'orientation_hold']
        }
      );
      expect(poseLock.success).toBe(true);
      expect((poseLock.lastIK?.quality?.positionResidualM ?? Infinity) * 1000).toBeLessThanOrEqual(1.5);

      const positionOnly = interpolator.interpolateCartesianSpace(
        startAngles,
        target,
        20,
        60,
        100,
        undefined,
        {
          positionWeight: 1.0,
          orientationWeight: 0.0,
          maxStepDeg: 4.0,
          maxJointVelocityDegS: 110,
          maxJointAccelerationDegS2: 320,
          minDamping: 0.0005,
          maxDamping: 0.3,
          dampingGrowth: 2.0,
          dampingShrink: 0.7,
          singularityThreshold: 0.0000005,
          postureWeight: 0.0000002,
          tolerancePositionM: 0.0015,
          toleranceOrientationRad: Number.POSITIVE_INFINITY,
          toleranceWeighted: 0.01,
          activeConstraints: ['position']
        }
      );
      expect(positionOnly.success).toBe(true);
      expect((positionOnly.lastIK?.quality?.positionResidualM ?? Infinity) * 1000).toBeLessThanOrEqual(1.5);
    }
  });
});

