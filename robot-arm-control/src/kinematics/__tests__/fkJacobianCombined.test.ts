import { ForwardKinematics } from '../ForwardKinematics';

describe('ForwardKinematics.solveWithJacobian', () => {
  const TEST_ANGLES = [5, -20, 35, 120, -30, 15];

  test('returns same pose as solve()', () => {
    const fkOnly = ForwardKinematics.solve(TEST_ANGLES);
    const { fk } = ForwardKinematics.solveWithJacobian(TEST_ANGLES);
    expect(fk.success).toBe(true);
    expect(fkOnly.success).toBe(true);
    expect(fk.endEffectorPose.position.x).toBeCloseTo(fkOnly.endEffectorPose.position.x, 10);
    expect(fk.endEffectorPose.position.y).toBeCloseTo(fkOnly.endEffectorPose.position.y, 10);
    expect(fk.endEffectorPose.position.z).toBeCloseTo(fkOnly.endEffectorPose.position.z, 10);
  });

  test('returns same Jacobian as computeJacobian()', () => {
    const jOnly = ForwardKinematics.computeJacobian(TEST_ANGLES);
    const { jacobian } = ForwardKinematics.solveWithJacobian(TEST_ANGLES);
    for (let row = 0; row < 6; row++) {
      for (let col = 0; col < 6; col++) {
        expect(jacobian[row][col]).toBeCloseTo(jOnly[row][col], 10);
      }
    }
  });

  test('handles all-zero angles', () => {
    const { fk, jacobian } = ForwardKinematics.solveWithJacobian([0, 0, 0, 0, 0, 0]);
    expect(fk.success).toBe(true);
    expect(jacobian.length).toBe(6);
    expect(jacobian[0].length).toBe(6);
  });
});
