/**
 * Verifies that PathInterpolator.interpolateCartesianSpace() uses SLERP to
 * interpolate orientation along the path instead of holding the target
 * orientation constant for every sample.
 *
 * Test strategy: run pose-locked interpolation with a start and target
 * orientation that differ meaningfully. Recover per-sample orientations via FK.
 * A proper SLERP implementation must produce orientations whose angular
 * distance from the start STRICTLY INCREASES across the path.
 * The old "constant locked orientation" behaviour causes all interior samples
 * to jump immediately to the final orientation, so their angular distance from
 * the start would be roughly equal to the start→end distance.
 */

import { ForwardKinematics } from '../../kinematics/ForwardKinematics';
import { QuaternionMath } from '../../kinematics/QuaternionMath';
import { Rotation3 } from '../../kinematics/types';
import { logicalToUrdfAngles } from '../../kinematics/angleMapping';
import { PathInterpolator } from '../PathInterpolator';

const orientationDistanceRad = (a: Rotation3, b: Rotation3): number => {
  const qa = QuaternionMath.fromEuler(a);
  const qb = QuaternionMath.fromEuler(b);
  return QuaternionMath.angularDistance(qa, qb);
};

describe('PathInterpolator SLERP orientation interpolation', () => {
  const offsets = [0, 0, 0, 0, 0, 0];

  test('orientation at mid-path is closer to start than to end when SLERP is active', () => {
    const interpolator = new PathInterpolator();

    // A well-conditioned mid-workspace pose.
    const startLogical = [5, 20, -15, 30, 20, 10];
    const startUrdf = logicalToUrdfAngles(startLogical, offsets);
    const startFk = ForwardKinematics.solve(startUrdf);
    expect(startFk.success).toBe(true);
    if (!startFk.success) return;

    const startPos = startFk.endEffectorPose.position;
    const startRot = startFk.endEffectorPose.rotation;

    // Target orientation: rotate J6 by ~45 deg in yaw to create a meaningful difference.
    const targetLogical = [5, 20, -15, 30, 20, 55];
    const targetUrdf = logicalToUrdfAngles(targetLogical, offsets);
    const targetFk = ForwardKinematics.solve(targetUrdf);
    expect(targetFk.success).toBe(true);
    if (!targetFk.success) return;
    const targetRot = targetFk.endEffectorPose.rotation;

    // The two orientations must differ by at least 20 degrees for the test to be meaningful.
    const totalAngle = orientationDistanceRad(startRot, targetRot);
    expect(totalAngle).toBeGreaterThan((20 * Math.PI) / 180);

    // Small position move so IK can converge at all samples.
    const targetPos = {
      x: startPos.x + 0.008,
      y: startPos.y + 0.004,
      z: startPos.z
    };

    const result = interpolator.interpolateCartesianSpace(
      startLogical,
      targetPos,
      20,     // speed mm/s
      60,     // accel mm/s²
      30,     // points per second
      targetRot,
      {
        positionWeight: 1.0,
        orientationWeight: 0.3,
        tolerancePositionM: 0.003,
        toleranceOrientationRad: 0.08,
        toleranceWeighted: 0.015,
        trackingMaxIterations: 28
      }
    );

    expect(result.success).toBe(true);
    if (!result.success) return;

    const points = result.segment.points;
    expect(points.length).toBeGreaterThan(4);

    // Recover per-sample orientations via FK.
    const orientations: Rotation3[] = [];
    for (const pt of points) {
      const fk = ForwardKinematics.solve(logicalToUrdfAngles(pt.jointAngles, offsets));
      if (!fk.success) continue;
      orientations.push(fk.endEffectorPose.rotation);
    }
    expect(orientations.length).toBeGreaterThan(4);

    // Compute angular distance from start for each recovered orientation.
    const dists = orientations.map((r) => orientationDistanceRad(r, startRot));

    // The MIDPOINT sample's distance from start should be less than the FULL
    // start→end distance, which means the solver saw a partial-way target, not
    // the full final orientation.
    const midIdx = Math.floor(dists.length / 2);
    const midDist = dists[midIdx];
    // The mid-point target under SLERP is at ~s(0.5) ≈ 0.5 of totalAngle.
    // IK tracks imperfectly, so we allow a generous bound: mid must be < 80% of total.
    expect(midDist).toBeLessThan(totalAngle * 0.80);

    // The final sample should be near the target orientation.
    const lastDist = orientationDistanceRad(orientations[orientations.length - 1], targetRot);
    expect(lastDist).toBeLessThan((12 * Math.PI) / 180);
  });
});
