import { ForwardKinematics } from '../ForwardKinematics';
import { InverseKinematics } from '../InverseKinematics';

type BenchmarkCase = {
  name: string;
  seed: number[];
  targetJointAngles: number[];
  mode: 'pose_lock' | 'position_only';
};

const CASES: BenchmarkCase[] = [
  {
    name: 'pose-lock nominal',
    seed: [3, 24, 13, 18, -7, 18],
    targetJointAngles: [5, 25, 15, 20, -8, 20],
    mode: 'pose_lock'
  },
  {
    name: 'pose-lock wrist-heavy',
    seed: [-9, 29, -21, 48, -14, 30],
    targetJointAngles: [-8, 28, -22, 55, -10, 35],
    mode: 'pose_lock'
  },
  {
    name: 'position-only shoulder move',
    seed: [11, 20, 24, 7, -13, 30],
    targetJointAngles: [12, 24, 30, 8, -12, 30],
    mode: 'position_only'
  },
  {
    name: 'position-only elbow retract',
    seed: [17, 30, -5, -8, -14, -10],
    targetJointAngles: [18, 28, -10, -5, -15, -10],
    mode: 'position_only'
  }
];

describe('IK quality benchmark', () => {
  const solver = new InverseKinematics({
    maxIterations: 180,
    tolerance: 0.001,
    dampingFactor: 0.01
  });

  test('runs deterministic benchmark cases and logs quality metrics', () => {
    let successCount = 0;
    const rows: Array<Record<string, string | number>> = [];

    for (const testCase of CASES) {
      const fkTarget = ForwardKinematics.solve(testCase.targetJointAngles);
      expect(fkTarget.success).toBe(true);
      if (!fkTarget.success) continue;

      const result = solver.solvePoseWeighted(
        fkTarget.endEffectorPose,
        testCase.seed,
        {
          orientationWeight: testCase.mode === 'pose_lock' ? 0.35 : 0.0,
          postureWeight: 0.0000002,
          maxStepDeg: 4.0,
          maxJointVelocityDegS: 100,
          maxJointAccelerationDegS2: 300,
          minDamping: 0.0005,
          maxDamping: 0.3,
          dampingGrowth: 2.0,
          dampingShrink: 0.7,
          singularityThreshold: 0.0000005,
          tolerancePositionM: 0.0015,
          toleranceOrientationRad: testCase.mode === 'pose_lock' ? 0.026 : Number.POSITIVE_INFINITY,
          toleranceWeighted: 0.01,
          activeConstraints: testCase.mode === 'pose_lock' ? ['position', 'orientation_hold'] : ['position']
        }
      );

      if (result.success) successCount += 1;

      rows.push({
        case: testCase.name,
        success: result.success ? 'yes' : 'no',
        pos_mm: ((result.quality?.positionResidualM ?? Infinity) * 1000).toFixed(3),
        ori_deg: (((result.quality?.orientationResidualRad ?? Infinity) * 180) / Math.PI).toFixed(3),
        cond: (result.quality?.conditionNumber ?? Infinity).toFixed(1),
        sigma_min: (result.quality?.minSingularValue ?? 0).toExponential(2)
      });

      expect(Number.isFinite(result.quality?.positionResidualM ?? NaN)).toBe(true);
      expect(Number.isFinite(result.quality?.orientationResidualRad ?? NaN)).toBe(true);
    }

    // Keep a reproducible baseline visible in test logs.
    // eslint-disable-next-line no-console
    console.table(rows);

    expect(rows).toHaveLength(CASES.length);
    expect(successCount).toBeGreaterThanOrEqual(3);
  });
});
