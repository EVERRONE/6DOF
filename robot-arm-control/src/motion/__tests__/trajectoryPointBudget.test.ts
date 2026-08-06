import { ForwardKinematics } from '../../kinematics/ForwardKinematics';
import { PathInterpolator } from '../PathInterpolator';

const estimateMinimumJerkDuration = (distanceM: number, speedMmS: number, accelMmS2: number): number => {
  const speedMs = Math.max(0.001, speedMmS / 1000);
  const accelMs = Math.max(0.001, accelMmS2 / 1000);
  const tVel = (1.875 * distanceM) / speedMs;
  const tAcc = Math.sqrt((5.8 * distanceM) / accelMs);
  return Math.max(0.25, tVel, tAcc);
};

describe('Cartesian trajectory point budget', () => {
  test('uses auto-budgeted sampling rate to keep generated points within queue max', () => {
    const interpolator = new PathInterpolator();
    const startAngles = [0, 0, 0, 0, 0, 0];
    const startFk = ForwardKinematics.solve(startAngles);
    expect(startFk.success).toBe(true);
    if (!startFk.success) return;

    const target = { x: -0.26, y: -0.0005, z: 0.311 };
    const speedMmS = 20;
    const accelMmS2 = 60;
    const defaultPps = 100;
    const queueBudget = 120;

    const distance = Math.sqrt(
      Math.pow(target.x - startFk.endEffectorPose.position.x, 2) +
      Math.pow(target.y - startFk.endEffectorPose.position.y, 2) +
      Math.pow(target.z - startFk.endEffectorPose.position.z, 2)
    );
    const duration = estimateMinimumJerkDuration(distance, speedMmS, accelMmS2);
    const maxPpsByQueue = Math.max(1, Math.floor((queueBudget - 1) / duration));
    const pps = Math.min(defaultPps, maxPpsByQueue);

    const result = interpolator.interpolateCartesianSpace(
      startAngles,
      target,
      speedMmS,
      accelMmS2,
      pps,
      startFk.endEffectorPose.rotation,
      {
        positionWeight: 1.0,
        orientationWeight: 0.35,
        maxStepDeg: 4.0,
        minDamping: 0.0005,
        maxDamping: 0.3,
        postureWeight: 0.0000002,
        tolerancePositionM: 0.0015,
        toleranceOrientationRad: 0.026,
        toleranceWeighted: 0.01
      }
    );

    expect(result.success).toBe(true);
    expect(result.segment.points.length).toBeLessThanOrEqual(queueBudget);
  });
});

