import { ForwardKinematics } from '../../kinematics/ForwardKinematics';
import { Rotation3, Vector3 } from '../../kinematics/types';
import { PathInterpolator } from '../PathInterpolator';
import { logicalToUrdfAngles } from '../../kinematics/angleMapping';
import { HybridIKSolver } from '../../kinematics/HybridIKSolver';
import { InverseKinematics } from '../../kinematics/InverseKinematics';
import { TrajectoryPoint } from '../types';

const distancePointToLine = (point: Vector3, a: Vector3, b: Vector3): number => {
  const ab = { x: b.x - a.x, y: b.y - a.y, z: b.z - a.z };
  const ap = { x: point.x - a.x, y: point.y - a.y, z: point.z - a.z };
  const abLen2 = ab.x * ab.x + ab.y * ab.y + ab.z * ab.z;
  if (abLen2 < 1e-12) return 0;
  const t = Math.max(0, Math.min(1, (ap.x * ab.x + ap.y * ab.y + ap.z * ab.z) / abLen2));
  const proj = { x: a.x + t * ab.x, y: a.y + t * ab.y, z: a.z + t * ab.z };
  const dx = point.x - proj.x;
  const dy = point.y - proj.y;
  const dz = point.z - proj.z;
  return Math.sqrt(dx * dx + dy * dy + dz * dz);
};

const eulerToQuaternion = (r: Rotation3): { w: number; x: number; y: number; z: number } => {
  const cr = Math.cos(r.roll * 0.5);
  const sr = Math.sin(r.roll * 0.5);
  const cp = Math.cos(r.pitch * 0.5);
  const sp = Math.sin(r.pitch * 0.5);
  const cy = Math.cos(r.yaw * 0.5);
  const sy = Math.sin(r.yaw * 0.5);

  return {
    w: cr * cp * cy + sr * sp * sy,
    x: sr * cp * cy - cr * sp * sy,
    y: cr * sp * cy + sr * cp * sy,
    z: cr * cp * sy - sr * sp * cy
  };
};

const orientationDistance = (a: Rotation3, b: Rotation3): number => {
  const qa = eulerToQuaternion(a);
  const qb = eulerToQuaternion(b);
  const dot = Math.max(-1, Math.min(1, qa.w * qb.w + qa.x * qb.x + qa.y * qb.y + qa.z * qb.z));
  return 2 * Math.acos(Math.abs(dot));
};

describe('populateJointVelocities velocity clamping', () => {
  class TestInterpolator extends PathInterpolator {
    public exposedPopulate(points: TrajectoryPoint[]): void {
      (this as any).populateJointVelocities(points);
    }
  }

  test('populateJointVelocities clamps interior velocities to 120 deg/s', () => {
    const interp = new TestInterpolator();

    // The interior central-difference formula uses dt = points[i+1].time - points[i-1].time.
    // With a 90° jump between t=0 and t=0.5s the raw estimate is 90/0.5 = 180 °/s (> 120).
    // After clamping it must be reduced to ±120.
    const points: TrajectoryPoint[] = [
      { time: 0.0,  jointAngles: [0, 0, 0,   0, 0, 0], velocity: Array(6).fill(0) },
      { time: 0.25, jointAngles: [0, 0, 0,  45, 0, 0], velocity: Array(6).fill(0) }, // midpoint
      { time: 0.5,  jointAngles: [0, 0, 0,  90, 0, 0], velocity: Array(6).fill(0) }  // 90°/0.5s = 180 °/s raw
    ];

    interp.exposedPopulate(points);

    // Interior point (index 1) velocity for J4 should be clamped to ±120
    const interiorVel = points[1].velocity![3];
    expect(Math.abs(interiorVel ?? 0)).toBeLessThanOrEqual(120);
    // Endpoints must remain zero
    expect(points[0].velocity).toEqual(Array(6).fill(0));
    expect(points[2].velocity).toEqual(Array(6).fill(0));
  });
});

describe('pose-locked Cartesian interpolation quality', () => {
  afterEach(() => {
    jest.restoreAllMocks();
  });

  test('keeps path close to straight line and orientation locked', () => {
    const interpolator = new PathInterpolator();
    const startLogicalAngles = [5, 20, -15, 30, 20, 10];
    const offsets = [0, 0, 0, 0, 0, 0];
    const startUrdfAngles = logicalToUrdfAngles(startLogicalAngles, offsets);
    const startFk = ForwardKinematics.solve(startUrdfAngles);
    expect(startFk.success).toBe(true);
    if (!startFk.success) return;

    const target = {
      x: startFk.endEffectorPose.position.x + 0.03,
      y: startFk.endEffectorPose.position.y - 0.015,
      z: startFk.endEffectorPose.position.z + 0.015
    };

    const result = interpolator.interpolateCartesianSpace(
      startLogicalAngles,
      target,
      25,
      80,
      40,
      startFk.endEffectorPose.rotation
    );

    expect(result.success).toBe(true);
    expect(result.segment.points.length).toBeGreaterThan(5);

    const pathPos: Vector3[] = [];
    const pathRot: Rotation3[] = [];
    for (const point of result.segment.points) {
      const fk = ForwardKinematics.solve(logicalToUrdfAngles(point.jointAngles, offsets));
      expect(fk.success).toBe(true);
      if (!fk.success) return;
      pathPos.push(fk.endEffectorPose.position);
      pathRot.push(fk.endEffectorPose.rotation);
    }

    const maxLineDeviation = pathPos.reduce((max, p) => {
      const d = distancePointToLine(p, pathPos[0], target);
      return Math.max(max, d);
    }, 0);
    expect(maxLineDeviation).toBeLessThanOrEqual(0.003);

    const maxOrientationDrift = pathRot.reduce((max, r) => {
      const d = orientationDistance(r, startFk.endEffectorPose.rotation);
      return Math.max(max, d);
    }, 0);
    expect(maxOrientationDrift).toBeLessThanOrEqual((2 * Math.PI) / 180);
  });

  test('solves reported near-boundary target in both pose-lock and position-only', () => {
    const interpolator = new PathInterpolator();
    const startLogicalAngles = [0, 0, 0, 0, 0, 0];
    const startFk = ForwardKinematics.solve(startLogicalAngles);
    expect(startFk.success).toBe(true);
    if (!startFk.success) return;

    const target = {
      x: -0.25,
      y: -0.0005,
      z: 0.311
    };

    const poseLockResult = interpolator.interpolateCartesianSpace(
      startLogicalAngles,
      target,
      20,
      60,
      100,
      startFk.endEffectorPose.rotation,
      {
        positionWeight: 1.0,
        orientationWeight: 0.35,
        maxStepDeg: 4.0,
        maxJointVelocityDegS: 100,
        maxJointAccelerationDegS2: 300,
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

    const positionOnlyResult = interpolator.interpolateCartesianSpace(
      startLogicalAngles,
      target,
      20,
      60,
      100,
      undefined,
      {
        positionWeight: 1.0,
        orientationWeight: 0.0,
        maxStepDeg: 4.0,
        maxJointVelocityDegS: 100,
        maxJointAccelerationDegS2: 300,
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

    expect(poseLockResult.success).toBe(true);
    expect((poseLockResult.lastIK?.quality?.positionResidualM ?? Infinity) * 1000).toBeLessThanOrEqual(1.5);
    expect(positionOnlyResult.success).toBe(true);
    expect((positionOnlyResult.lastIK?.quality?.positionResidualM ?? Infinity) * 1000).toBeLessThanOrEqual(1.5);
  });

  test('can bypass strict final residual gate for stage-1 feasibility planning', () => {
    jest.spyOn(HybridIKSolver.prototype, 'solvePoseWeighted').mockImplementation((_: any, seed: number[]) => ({
      jointAngles: [...seed],
      success: true,
      quality: {
        positionResidualM: 0.006,
        orientationResidualRad: 0.001,
        weightedResidual: 0.006,
        minSingularValue: 1e-4,
        conditionNumber: 1000,
        activeConstraints: ['position'],
        jointParticipation: [1, 1, 1, 1, 1, 1],
        iterations: 8,
        damping: 0.01
      }
    } as any));

    const interpolator = new PathInterpolator();
    const startLogicalAngles = [0, 0, 0, 0, 0, 0];
    const strict = interpolator.interpolateCartesianSpace(
      startLogicalAngles,
      { x: -0.22, y: -0.0005, z: 0.311 },
      20,
      60,
      12
    );
    const feasibility = interpolator.interpolateCartesianSpace(
      startLogicalAngles,
      { x: -0.22, y: -0.0005, z: 0.311 },
      20,
      60,
      12,
      undefined,
      undefined,
      'hybrid_constrained_v2',
      false
    );

    expect(strict.success).toBe(false);
    expect(strict.error).toContain('strict residual gate');
    expect(feasibility.success).toBe(true);
  });

  test('refines terminal sample when strict final gate is narrowly missed', () => {
    jest.spyOn(HybridIKSolver.prototype, 'solvePoseWeighted').mockImplementation((_: any, seed: number[], options?: any) => {
      const refined = (options?.trackingMaxIterations ?? 0) >= 32;
      const residual = refined ? 0.0012 : 0.0017;
      return {
        jointAngles: [...seed],
        success: true,
        quality: {
          positionResidualM: residual,
          orientationResidualRad: 0.001,
          weightedResidual: residual,
          minSingularValue: 1e-4,
          conditionNumber: 1000,
          activeConstraints: ['position', 'orientation_hold'],
          jointParticipation: [1, 1, 1, 1, 1, 1],
          iterations: refined ? 20 : 10,
          damping: 0.01
        }
      } as any;
    });

    const interpolator = new PathInterpolator();
    const startLogicalAngles = [0, 0, 0, 0, 0, 0];
    const startFk = ForwardKinematics.solve(startLogicalAngles);
    expect(startFk.success).toBe(true);
    if (!startFk.success) return;

    const result = interpolator.interpolateCartesianSpace(
      startLogicalAngles,
      {
        x: startFk.endEffectorPose.position.x + 0.01,
        y: startFk.endEffectorPose.position.y,
        z: startFk.endEffectorPose.position.z
      },
      20,
      60,
      12,
      startFk.endEffectorPose.rotation,
      {
        tolerancePositionM: 0.0015,
        toleranceOrientationRad: 0.026,
        toleranceWeighted: 0.01,
        trackingMaxIterations: 20
      }
    );

    expect(result.success).toBe(true);
    expect(result.lastIK?.quality?.positionResidualM ?? Infinity).toBeLessThanOrEqual(0.0015);
    expect(result.lastIK?.notes || []).toContain('terminal_refinement_pass');
  });

  test('recovers resolved-rate sample failures with iterative fallback', () => {
    const resolvedRateSpy = jest.spyOn(InverseKinematics.prototype, 'stepResolvedRate').mockImplementation((_: any, seed: number[]) => ({
      jointAngles: [...seed],
      success: false,
      error: 'Resolved-rate residual too high (pos 0.0120m, ori 0.0010rad, weighted 0.0120)',
      failureCategory: 'max_iterations',
      quality: {
        positionResidualM: 0.012,
        orientationResidualRad: 0.001,
        weightedResidual: 0.012,
        minSingularValue: 1e-4,
        conditionNumber: 800,
        activeConstraints: ['position'],
        jointParticipation: [1, 1, 1, 1, 1, 1],
        iterations: 1,
        damping: 0.01
      }
    } as any));
    const iterativeSpy = jest.spyOn(HybridIKSolver.prototype, 'solvePoseWeighted').mockImplementation((_: any, seed: number[]) => ({
      jointAngles: [...seed],
      success: true,
      quality: {
        positionResidualM: 0.0008,
        orientationResidualRad: 0.001,
        weightedResidual: 0.0008,
        minSingularValue: 1e-4,
        conditionNumber: 800,
        activeConstraints: ['position'],
        jointParticipation: [1, 1, 1, 1, 1, 1],
        iterations: 8,
        damping: 0.01
      }
    } as any));

    const interpolator = new PathInterpolator();
    const startLogicalAngles = [0, 0, 0, 0, 0, 0];
    const startFk = ForwardKinematics.solve(startLogicalAngles);
    expect(startFk.success).toBe(true);
    if (!startFk.success) return;

    const result = interpolator.interpolateCartesianSpace(
      startLogicalAngles,
      {
        x: startFk.endEffectorPose.position.x + 0.01,
        y: startFk.endEffectorPose.position.y,
        z: startFk.endEffectorPose.position.z
      },
      20,
      60,
      12,
      startFk.endEffectorPose.rotation,
      { trackingMode: 'resolved_rate' }
    );

    expect(result.success).toBe(true);
    expect(resolvedRateSpy).toHaveBeenCalled();
    expect(iterativeSpy).toHaveBeenCalled();
  });
});
