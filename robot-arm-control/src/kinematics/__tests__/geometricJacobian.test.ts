import { ForwardKinematics } from '../ForwardKinematics';
import { GeometricJacobian } from '../GeometricJacobian';

const EPS_DEG = 0.05;

const eulerToQuaternion = (rotation: { roll: number; pitch: number; yaw: number }) => {
  const cr = Math.cos(rotation.roll * 0.5);
  const sr = Math.sin(rotation.roll * 0.5);
  const cp = Math.cos(rotation.pitch * 0.5);
  const sp = Math.sin(rotation.pitch * 0.5);
  const cy = Math.cos(rotation.yaw * 0.5);
  const sy = Math.sin(rotation.yaw * 0.5);

  return {
    w: cr * cp * cy + sr * sp * sy,
    x: sr * cp * cy - cr * sp * sy,
    y: cr * sp * cy + sr * cp * sy,
    z: cr * cp * sy - sr * sp * cy
  };
};

const quatMul = (
  a: { w: number; x: number; y: number; z: number },
  b: { w: number; x: number; y: number; z: number }
) => ({
  w: a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
  x: a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
  y: a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
  z: a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w
});

const quatConj = (q: { w: number; x: number; y: number; z: number }) => ({
  w: q.w,
  x: -q.x,
  y: -q.y,
  z: -q.z
});

const quatToRotVec = (qInput: { w: number; x: number; y: number; z: number }): [number, number, number] => {
  const q = qInput.w < 0
    ? { w: -qInput.w, x: -qInput.x, y: -qInput.y, z: -qInput.z }
    : qInput;
  const vNorm = Math.sqrt(q.x * q.x + q.y * q.y + q.z * q.z);
  if (vNorm < 1e-12) return [0, 0, 0];
  const angle = 2 * Math.atan2(vNorm, Math.max(-1, Math.min(1, q.w)));
  const scale = angle / vNorm;
  return [q.x * scale, q.y * scale, q.z * scale];
};

const numericJacobian = (jointAnglesDeg: number[]): number[][] => {
  const jacobian = Array.from({ length: 6 }, () => Array(6).fill(0));
  for (let i = 0; i < 6; i++) {
    const plus = [...jointAnglesDeg];
    const minus = [...jointAnglesDeg];
    plus[i] += EPS_DEG;
    minus[i] -= EPS_DEG;
    const fkPlus = ForwardKinematics.solve(plus);
    const fkMinus = ForwardKinematics.solve(minus);
    if (!fkPlus.success || !fkMinus.success) continue;
    const denom = 2 * EPS_DEG;
    jacobian[0][i] = (fkPlus.endEffectorPose.position.x - fkMinus.endEffectorPose.position.x) / denom;
    jacobian[1][i] = (fkPlus.endEffectorPose.position.y - fkMinus.endEffectorPose.position.y) / denom;
    jacobian[2][i] = (fkPlus.endEffectorPose.position.z - fkMinus.endEffectorPose.position.z) / denom;

    const qPlus = eulerToQuaternion(fkPlus.endEffectorPose.rotation);
    const qMinus = eulerToQuaternion(fkMinus.endEffectorPose.rotation);
    const rotVec = quatToRotVec(quatMul(qPlus, quatConj(qMinus)));
    jacobian[3][i] = rotVec[0] / denom;
    jacobian[4][i] = rotVec[1] / denom;
    jacobian[5][i] = rotVec[2] / denom;
  }
  return jacobian;
};

describe('GeometricJacobian', () => {
  test('matches finite-difference Jacobian in URDF frame', () => {
    const q = [12, 25, -18, 33, -20, 40];
    const analytic = GeometricJacobian.compute(q);
    const numeric = numericJacobian(q);

    for (let r = 0; r < 6; r++) {
      for (let c = 0; c < 6; c++) {
        expect(analytic[r][c]).toBeCloseTo(numeric[r][c], 2);
      }
    }
  });
});
