import { ForwardKinematics } from '../../kinematics/ForwardKinematics';
import { PathInterpolator } from '../PathInterpolator';

describe('PathInterpolator deadline handling', () => {
  test('returns explicit timeout metadata when deadline is exceeded', () => {
    const interpolator = new PathInterpolator();
    const startLogicalAngles = [0, 0, 0, 0, 0, 0];
    const startFk = ForwardKinematics.solve(startLogicalAngles);
    expect(startFk.success).toBe(true);
    if (!startFk.success) return;

    const result = interpolator.interpolateCartesianSpace(
      startLogicalAngles,
      {
        x: startFk.endEffectorPose.position.x + 0.02,
        y: startFk.endEffectorPose.position.y,
        z: startFk.endEffectorPose.position.z
      },
      20,
      60,
      25,
      startFk.endEffectorPose.rotation,
      undefined,
      'hybrid_constrained_v2',
      true,
      Date.now() - 1
    );

    expect(result.success).toBe(false);
    expect(result.timedOut).toBe(true);
    expect(result.error).toContain('deadline');
    expect(result.lastIK?.failureCategory).toBe('stage2_timeout');
    expect(result.lastIK?.errorCode).toBe('STAGE2_TIMEOUT');
  });
});
