// Small, dependency-free linear algebra for the kinematics layer.
//
// Everything here works in radians and metres. Mixing units was the root of
// several bugs in the previous implementation, so the rule is: no degrees
// below the public API boundary.

import { Matrix4x4, Rotation3, Vector3 } from './types';

// ---------------------------------------------------------------------------
// 4x4 homogeneous transforms
// ---------------------------------------------------------------------------

export function identity4(): Matrix4x4 {
  return [
    [1, 0, 0, 0],
    [0, 1, 0, 0],
    [0, 0, 1, 0],
    [0, 0, 0, 1]
  ];
}

export function multiply4(A: Matrix4x4, B: Matrix4x4): Matrix4x4 {
  const C: Matrix4x4 = [
    [0, 0, 0, 0],
    [0, 0, 0, 0],
    [0, 0, 0, 0],
    [0, 0, 0, 0]
  ];

  for (let i = 0; i < 4; i++) {
    const Ai = A[i];
    const Ci = C[i];
    for (let k = 0; k < 4; k++) {
      const a = Ai[k];
      if (a === 0) continue;
      const Bk = B[k];
      for (let j = 0; j < 4; j++) {
        Ci[j] += a * Bk[j];
      }
    }
  }

  return C;
}

/**
 * Fixed joint origin transform: Translate(xyz) * Rz(yaw) * Ry(pitch) * Rx(roll).
 *
 * This is the URDF rpy convention (extrinsic XYZ / fixed-axis roll-pitch-yaw),
 * which is the same thing THREE.Euler expresses with order 'ZYX'. The 3D
 * viewer uses that order, so the two stay in agreement.
 */
export function fromXyzRpy(xyz: Vector3, rpy: Rotation3): Matrix4x4 {
  const cr = Math.cos(rpy.roll);
  const sr = Math.sin(rpy.roll);
  const cp = Math.cos(rpy.pitch);
  const sp = Math.sin(rpy.pitch);
  const cy = Math.cos(rpy.yaw);
  const sy = Math.sin(rpy.yaw);

  return [
    [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr, xyz.x],
    [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr, xyz.y],
    [-sp, cp * sr, cp * cr, xyz.z],
    [0, 0, 0, 1]
  ];
}

/** Rotation about Z by theta radians, as a 4x4 transform. */
export function rotZ4(theta: number): Matrix4x4 {
  const c = Math.cos(theta);
  const s = Math.sin(theta);
  return [
    [c, -s, 0, 0],
    [s, c, 0, 0],
    [0, 0, 1, 0],
    [0, 0, 0, 1]
  ];
}

/** Translation column of a transform. */
export function getTranslation(T: Matrix4x4): Vector3 {
  return { x: T[0][3], y: T[1][3], z: T[2][3] };
}

/** Upper-left 3x3 rotation block of a transform. */
export function getRotation(T: Matrix4x4): number[][] {
  return [
    [T[0][0], T[0][1], T[0][2]],
    [T[1][0], T[1][1], T[1][2]],
    [T[2][0], T[2][1], T[2][2]]
  ];
}

/** Column `k` of the rotation block, i.e. a basis axis in world coordinates. */
export function getAxis(T: Matrix4x4, k: number): Vector3 {
  return { x: T[0][k], y: T[1][k], z: T[2][k] };
}

/** Transform a point by a 4x4 homogeneous transform. */
export function transformPoint(T: Matrix4x4, p: Vector3): Vector3 {
  return {
    x: T[0][0] * p.x + T[0][1] * p.y + T[0][2] * p.z + T[0][3],
    y: T[1][0] * p.x + T[1][1] * p.y + T[1][2] * p.z + T[1][3],
    z: T[2][0] * p.x + T[2][1] * p.y + T[2][2] * p.z + T[2][3]
  };
}

// ---------------------------------------------------------------------------
// 3D vectors
// ---------------------------------------------------------------------------

export function sub(a: Vector3, b: Vector3): Vector3 {
  return { x: a.x - b.x, y: a.y - b.y, z: a.z - b.z };
}

export function cross(a: Vector3, b: Vector3): Vector3 {
  return {
    x: a.y * b.z - a.z * b.y,
    y: a.z * b.x - a.x * b.z,
    z: a.x * b.y - a.y * b.x
  };
}

export function norm(v: Vector3): number {
  return Math.sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
}

export function distance(a: Vector3, b: Vector3): number {
  return norm(sub(a, b));
}

// ---------------------------------------------------------------------------
// Rotations
// ---------------------------------------------------------------------------

/**
 * Rotation matrix -> fixed-axis roll/pitch/yaw (the URDF convention, so
 * R = Rz(yaw) * Ry(pitch) * Rx(roll)). Radians.
 *
 * Used for display and for the pose API. Never used to build an orientation
 * error - see rotationLog for that.
 */
export function matrixToRpy(R: number[][]): Rotation3 {
  const cp = Math.sqrt(R[0][0] * R[0][0] + R[1][0] * R[1][0]);

  // Gimbal lock: pitch is +/-90 deg, roll and yaw are not separable.
  // Pin yaw to zero and fold the whole rotation into roll.
  if (cp < 1e-9) {
    const pitch = R[2][0] <= 0 ? Math.PI / 2 : -Math.PI / 2;
    const roll =
      R[2][0] <= 0
        ? Math.atan2(R[0][1], R[0][2])
        : Math.atan2(-R[0][1], -R[0][2]);
    return { roll, pitch, yaw: 0 };
  }

  return {
    roll: Math.atan2(R[2][1], R[2][2]),
    pitch: Math.atan2(-R[2][0], cp),
    yaw: Math.atan2(R[1][0], R[0][0])
  };
}

/** Fixed-axis roll/pitch/yaw (radians) -> rotation matrix. */
export function rpyToMatrix(rpy: Rotation3): number[][] {
  return getRotation(fromXyzRpy({ x: 0, y: 0, z: 0 }, rpy));
}

export function transpose3(R: number[][]): number[][] {
  return [
    [R[0][0], R[1][0], R[2][0]],
    [R[0][1], R[1][1], R[2][1]],
    [R[0][2], R[1][2], R[2][2]]
  ];
}

export function multiply3(A: number[][], B: number[][]): number[][] {
  const C = [
    [0, 0, 0],
    [0, 0, 0],
    [0, 0, 0]
  ];
  for (let i = 0; i < 3; i++) {
    for (let j = 0; j < 3; j++) {
      C[i][j] = A[i][0] * B[0][j] + A[i][1] * B[1][j] + A[i][2] * B[2][j];
    }
  }
  return C;
}

/**
 * Matrix logarithm of a rotation: the axis-angle vector whose direction is the
 * rotation axis and whose magnitude is the rotation angle in radians.
 *
 * This is the correct orientation error for IK. Subtracting Euler angles - what
 * the old solver did - is wrong: Euler triples wrap, are discontinuous at
 * gimbal lock, and their difference is not a rotation.
 */
export function rotationLog(R: number[][]): Vector3 {
  // Antisymmetric part, equal to 2 * sin(theta) * axis.
  const wx = R[2][1] - R[1][2];
  const wy = R[0][2] - R[2][0];
  const wz = R[1][0] - R[0][1];

  const twoSin = Math.sqrt(wx * wx + wy * wy + wz * wz);
  const cosTheta = (R[0][0] + R[1][1] + R[2][2] - 1) / 2;

  // atan2 rather than acos: acos has an infinite derivative at +/-1, so it
  // loses roughly half the available precision near 0 and near pi. atan2 takes
  // the angle from the antisymmetric part, whose absolute error stays at the
  // rounding floor, and stays accurate over the full range.
  const theta = Math.atan2(twoSin / 2, cosTheta);

  if (twoSin > 1e-7) {
    // Normal case: the axis is the normalised antisymmetric part.
    const k = theta / twoSin;
    return { x: k * wx, y: k * wy, z: k * wz };
  }

  if (cosTheta > 0) {
    // Near identity: log(R) ~= antisymmetric part / 2.
    return { x: wx / 2, y: wy / 2, z: wz / 2 };
  }

  // Within ~1e-7 rad of a 180 degree rotation the antisymmetric part has
  // collapsed into the noise floor, so recover the axis from the symmetric
  // part instead: R = 2 * axis axis^T - I. The axis sign is genuinely
  // unrecoverable this close to pi, and either sign describes the same rotation.
  const xx = (R[0][0] + 1) / 2;
  const yy = (R[1][1] + 1) / 2;
  const zz = (R[2][2] + 1) / 2;

  let ax: number;
  let ay: number;
  let az: number;

  // Pivot on the largest diagonal entry to keep the division well conditioned.
  if (xx >= yy && xx >= zz) {
    ax = Math.sqrt(Math.max(0, xx));
    ay = R[0][1] / (2 * ax);
    az = R[0][2] / (2 * ax);
  } else if (yy >= zz) {
    ay = Math.sqrt(Math.max(0, yy));
    ax = R[0][1] / (2 * ay);
    az = R[1][2] / (2 * ay);
  } else {
    az = Math.sqrt(Math.max(0, zz));
    ax = R[0][2] / (2 * az);
    ay = R[1][2] / (2 * az);
  }

  const len = Math.sqrt(ax * ax + ay * ay + az * az) || 1;
  return { x: (theta * ax) / len, y: (theta * ay) / len, z: (theta * az) / len };
}

// ---------------------------------------------------------------------------
// Dense linear algebra on small matrices
// ---------------------------------------------------------------------------

export function transpose(M: number[][]): number[][] {
  const rows = M.length;
  const cols = M[0].length;
  const T: number[][] = Array.from({ length: cols }, () => Array(rows).fill(0));
  for (let i = 0; i < rows; i++) {
    for (let j = 0; j < cols; j++) {
      T[j][i] = M[i][j];
    }
  }
  return T;
}

/** A^T * A for an m x n matrix, returning n x n. */
export function normalMatrix(A: number[][]): number[][] {
  const m = A.length;
  const n = A[0].length;
  const N: number[][] = Array.from({ length: n }, () => Array(n).fill(0));

  for (let i = 0; i < n; i++) {
    for (let j = i; j < n; j++) {
      let sum = 0;
      for (let k = 0; k < m; k++) {
        sum += A[k][i] * A[k][j];
      }
      N[i][j] = sum;
      N[j][i] = sum;
    }
  }

  return N;
}

/** A^T * b for an m x n matrix and length-m vector, returning length n. */
export function normalVector(A: number[][], b: number[]): number[] {
  const m = A.length;
  const n = A[0].length;
  const out = Array(n).fill(0);

  for (let i = 0; i < n; i++) {
    let sum = 0;
    for (let k = 0; k < m; k++) {
      sum += A[k][i] * b[k];
    }
    out[i] = sum;
  }

  return out;
}

/**
 * Solve A x = b for a symmetric positive definite A, by Cholesky factorisation.
 *
 * Returns null if A turns out not to be positive definite, which lets the
 * caller raise the damping and retry. This replaces the Gauss-Seidel solver
 * the old code used: Gauss-Seidel only converges for diagonally dominant
 * systems, and J^T J + lambda^2 I is not diagonally dominant here (measured:
 * all six rows failed the test), so its 20 fixed sweeps returned whatever they
 * happened to land on.
 */
export function solveSPD(A: number[][], b: number[]): number[] | null {
  const n = b.length;
  const L: number[][] = Array.from({ length: n }, () => Array(n).fill(0));

  for (let i = 0; i < n; i++) {
    for (let j = 0; j <= i; j++) {
      let sum = A[i][j];
      for (let k = 0; k < j; k++) {
        sum -= L[i][k] * L[j][k];
      }

      if (i === j) {
        if (sum <= 0 || !Number.isFinite(sum)) {
          return null;
        }
        L[i][j] = Math.sqrt(sum);
      } else {
        L[i][j] = sum / L[j][j];
      }
    }
  }

  // Forward substitution: L y = b
  const y = Array(n).fill(0);
  for (let i = 0; i < n; i++) {
    let sum = b[i];
    for (let k = 0; k < i; k++) {
      sum -= L[i][k] * y[k];
    }
    y[i] = sum / L[i][i];
  }

  // Back substitution: L^T x = y
  const x = Array(n).fill(0);
  for (let i = n - 1; i >= 0; i--) {
    let sum = y[i];
    for (let k = i + 1; k < n; k++) {
      sum -= L[k][i] * x[k];
    }
    x[i] = sum / L[i][i];
  }

  if (!x.every(Number.isFinite)) {
    return null;
  }

  return x;
}
