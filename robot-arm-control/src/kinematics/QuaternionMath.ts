import { Pose, PoseQuat, Quaternion, Rotation3 } from './types';

const clamp = (value: number, min: number, max: number): number => (
  Math.min(max, Math.max(min, value))
);

const normalize = (q: Quaternion): Quaternion => {
  const norm = Math.hypot(q.w, q.x, q.y, q.z);
  if (norm < 1e-12) {
    return { w: 1, x: 0, y: 0, z: 0 };
  }
  return {
    w: q.w / norm,
    x: q.x / norm,
    y: q.y / norm,
    z: q.z / norm
  };
};

const withPositiveHemisphere = (q: Quaternion): Quaternion => (
  q.w < 0 ? { w: -q.w, x: -q.x, y: -q.y, z: -q.z } : q
);

export class QuaternionMath {
  static identity(): Quaternion {
    return { w: 1, x: 0, y: 0, z: 0 };
  }

  static conjugate(q: Quaternion): Quaternion {
    return { w: q.w, x: -q.x, y: -q.y, z: -q.z };
  }

  static multiply(a: Quaternion, b: Quaternion): Quaternion {
    return normalize({
      w: a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z,
      x: a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y,
      y: a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
      z: a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w
    });
  }

  static inverse(q: Quaternion): Quaternion {
    return this.conjugate(normalize(q));
  }

  static fromEuler(rotation: Rotation3): Quaternion {
    const cr = Math.cos(rotation.roll * 0.5);
    const sr = Math.sin(rotation.roll * 0.5);
    const cp = Math.cos(rotation.pitch * 0.5);
    const sp = Math.sin(rotation.pitch * 0.5);
    const cy = Math.cos(rotation.yaw * 0.5);
    const sy = Math.sin(rotation.yaw * 0.5);

    return normalize({
      w: cr * cp * cy + sr * sp * sy,
      x: sr * cp * cy - cr * sp * sy,
      y: cr * sp * cy + sr * cp * sy,
      z: cr * cp * sy - sr * sp * cy
    });
  }

  static toEuler(qInput: Quaternion): Rotation3 {
    const q = normalize(qInput);

    const sinrCosp = 2 * (q.w * q.x + q.y * q.z);
    const cosrCosp = 1 - 2 * (q.x * q.x + q.y * q.y);
    const roll = Math.atan2(sinrCosp, cosrCosp);

    const sinp = 2 * (q.w * q.y - q.z * q.x);
    const pitch = Math.abs(sinp) >= 1
      ? Math.sign(sinp) * (Math.PI / 2)
      : Math.asin(sinp);

    const sinyCosp = 2 * (q.w * q.z + q.x * q.y);
    const cosyCosp = 1 - 2 * (q.y * q.y + q.z * q.z);
    const yaw = Math.atan2(sinyCosp, cosyCosp);

    return { roll, pitch, yaw };
  }

  static fromRotationMatrix(rotation: number[][]): Quaternion {
    const trace = rotation[0][0] + rotation[1][1] + rotation[2][2];
    let q: Quaternion;

    if (trace > 0) {
      const s = Math.sqrt(trace + 1.0) * 2;
      q = {
        w: 0.25 * s,
        x: (rotation[2][1] - rotation[1][2]) / s,
        y: (rotation[0][2] - rotation[2][0]) / s,
        z: (rotation[1][0] - rotation[0][1]) / s
      };
    } else if (rotation[0][0] > rotation[1][1] && rotation[0][0] > rotation[2][2]) {
      const s = Math.sqrt(1.0 + rotation[0][0] - rotation[1][1] - rotation[2][2]) * 2;
      q = {
        w: (rotation[2][1] - rotation[1][2]) / s,
        x: 0.25 * s,
        y: (rotation[0][1] + rotation[1][0]) / s,
        z: (rotation[0][2] + rotation[2][0]) / s
      };
    } else if (rotation[1][1] > rotation[2][2]) {
      const s = Math.sqrt(1.0 + rotation[1][1] - rotation[0][0] - rotation[2][2]) * 2;
      q = {
        w: (rotation[0][2] - rotation[2][0]) / s,
        x: (rotation[0][1] + rotation[1][0]) / s,
        y: 0.25 * s,
        z: (rotation[1][2] + rotation[2][1]) / s
      };
    } else {
      const s = Math.sqrt(1.0 + rotation[2][2] - rotation[0][0] - rotation[1][1]) * 2;
      q = {
        w: (rotation[1][0] - rotation[0][1]) / s,
        x: (rotation[0][2] + rotation[2][0]) / s,
        y: (rotation[1][2] + rotation[2][1]) / s,
        z: 0.25 * s
      };
    }

    return withPositiveHemisphere(normalize(q));
  }

  static toRotationMatrix(qInput: Quaternion): number[][] {
    const q = normalize(qInput);
    const xx = q.x * q.x;
    const yy = q.y * q.y;
    const zz = q.z * q.z;
    const xy = q.x * q.y;
    const xz = q.x * q.z;
    const yz = q.y * q.z;
    const wx = q.w * q.x;
    const wy = q.w * q.y;
    const wz = q.w * q.z;

    return [
      [1 - 2 * (yy + zz), 2 * (xy - wz), 2 * (xz + wy)],
      [2 * (xy + wz), 1 - 2 * (xx + zz), 2 * (yz - wx)],
      [2 * (xz - wy), 2 * (yz + wx), 1 - 2 * (xx + yy)]
    ];
  }

  static toPoseQuat(pose: Pose): PoseQuat {
    return {
      position: { ...pose.position },
      orientation: this.fromEuler(pose.rotation)
    };
  }

  static toPoseEuler(pose: PoseQuat): Pose {
    return {
      position: { ...pose.position },
      rotation: this.toEuler(pose.orientation)
    };
  }

  static logMapError(target: Quaternion, current: Quaternion): { x: number; y: number; z: number } {
    let qErr = this.multiply(target, this.inverse(current));
    qErr = withPositiveHemisphere(qErr);
    const vNorm = Math.hypot(qErr.x, qErr.y, qErr.z);
    if (vNorm < 1e-12) {
      return { x: 0, y: 0, z: 0 };
    }

    const angle = 2 * Math.atan2(vNorm, clamp(qErr.w, -1, 1));
    const scale = angle / vNorm;
    return {
      x: qErr.x * scale,
      y: qErr.y * scale,
      z: qErr.z * scale
    };
  }

  static angularDistance(a: Quaternion, b: Quaternion): number {
    const qa = normalize(a);
    const qb = normalize(b);
    const dot = clamp(Math.abs(qa.w * qb.w + qa.x * qb.x + qa.y * qb.y + qa.z * qb.z), 0, 1);
    return 2 * Math.acos(dot);
  }

  static slerp(aInput: Quaternion, bInput: Quaternion, t: number): Quaternion {
    const a = normalize(aInput);
    let b = normalize(bInput);
    let cosTheta = a.w * b.w + a.x * b.x + a.y * b.y + a.z * b.z;

    if (cosTheta < 0) {
      b = { w: -b.w, x: -b.x, y: -b.y, z: -b.z };
      cosTheta = -cosTheta;
    }

    if (cosTheta > 0.9995) {
      return normalize({
        w: a.w + t * (b.w - a.w),
        x: a.x + t * (b.x - a.x),
        y: a.y + t * (b.y - a.y),
        z: a.z + t * (b.z - a.z)
      });
    }

    const theta = Math.acos(clamp(cosTheta, -1, 1));
    const sinTheta = Math.sin(theta);
    const wA = Math.sin((1 - t) * theta) / sinTheta;
    const wB = Math.sin(t * theta) / sinTheta;

    return normalize({
      w: a.w * wA + b.w * wB,
      x: a.x * wA + b.x * wB,
      y: a.y * wA + b.y * wB,
      z: a.z * wA + b.z * wB
    });
  }
}
