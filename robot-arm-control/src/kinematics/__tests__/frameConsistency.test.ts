import { ForwardKinematics } from '../ForwardKinematics';

describe('ForwardKinematics frame consistency', () => {
  test('matches URDF-chain golden pose at zero angles', () => {
    const fk = ForwardKinematics.solve([0, 0, 0, 0, 0, 0]);
    expect(fk.success).toBe(true);

    if (!fk.success) return;

    expect(fk.endEffectorPose.position.x).toBeCloseTo(-0.20223, 3);
    expect(fk.endEffectorPose.position.y).toBeCloseTo(-0.00048, 3);
    expect(fk.endEffectorPose.position.z).toBeCloseTo(0.31105, 3);
  });

  test('J1 rotation preserves radial reach and height', () => {
    const fkA = ForwardKinematics.solve([0, 0, 0, 0, 0, 0]);
    const fkB = ForwardKinematics.solve([90, 0, 0, 0, 0, 0]);

    expect(fkA.success).toBe(true);
    expect(fkB.success).toBe(true);
    if (!fkA.success || !fkB.success) return;

    const radiusA = Math.hypot(fkA.endEffectorPose.position.x, fkA.endEffectorPose.position.y);
    const radiusB = Math.hypot(fkB.endEffectorPose.position.x, fkB.endEffectorPose.position.y);

    expect(radiusA).toBeCloseTo(radiusB, 6);
    expect(fkA.endEffectorPose.position.z).toBeCloseTo(fkB.endEffectorPose.position.z, 6);
  });

  test('J6 rotation does not change end-effector position', () => {
    const start = [10, 20, -30, 40, -50, 0];
    const rotated = [10, 20, -30, 40, -50, 120];

    const fkStart = ForwardKinematics.solve(start);
    const fkRotated = ForwardKinematics.solve(rotated);

    expect(fkStart.success).toBe(true);
    expect(fkRotated.success).toBe(true);
    if (!fkStart.success || !fkRotated.success) return;

    const dx = fkStart.endEffectorPose.position.x - fkRotated.endEffectorPose.position.x;
    const dy = fkStart.endEffectorPose.position.y - fkRotated.endEffectorPose.position.y;
    const dz = fkStart.endEffectorPose.position.z - fkRotated.endEffectorPose.position.z;
    const distance = Math.sqrt(dx * dx + dy * dy + dz * dz);

    expect(distance).toBeLessThan(1e-6);
  });
});
