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
// Quaternions, for interpolating between orientations
// ---------------------------------------------------------------------------
//
// Interpolating a rotation is the one job Euler angles cannot do. Averaging two
// rpy triples does not give a rotation halfway between them: the triples wrap at
// +/-pi, so a pair either side of the wrap averages to the opposite of what was
// meant, and near gimbal lock two very different triples describe nearly the
// same rotation. A quaternion has neither problem, and slerp traces the shortest
// rotation between two attitudes at a constant angular rate - which is what a
// tool changing attitude along a path should do.

export interface Quaternion {
  w: number;
  x: number;
  y: number;
  z: number;
}

/**
 * Rotation matrix to unit quaternion, by Shepperd's method.
 *
 * The naive formula divides by sqrt(1 + trace), which loses precision as the
 * trace approaches -1 and fails outright at a 180 degree rotation. Pivoting on
 * whichever of the four components is largest keeps every division well
 * conditioned over the whole range.
 */
export function matrixToQuat(R: number[][]): Quaternion {
  const trace = R[0][0] + R[1][1] + R[2][2];

  if (trace > 0) {
    const s = Math.sqrt(trace + 1) * 2;
    return {
      w: s / 4,
      x: (R[2][1] - R[1][2]) / s,
      y: (R[0][2] - R[2][0]) / s,
      z: (R[1][0] - R[0][1]) / s
    };
  }

  if (R[0][0] > R[1][1] && R[0][0] > R[2][2]) {
    const s = Math.sqrt(1 + R[0][0] - R[1][1] - R[2][2]) * 2;
    return {
      w: (R[2][1] - R[1][2]) / s,
      x: s / 4,
      y: (R[0][1] + R[1][0]) / s,
      z: (R[0][2] + R[2][0]) / s
    };
  }

  if (R[1][1] > R[2][2]) {
    const s = Math.sqrt(1 + R[1][1] - R[0][0] - R[2][2]) * 2;
    return {
      w: (R[0][2] - R[2][0]) / s,
      x: (R[0][1] + R[1][0]) / s,
      y: s / 4,
      z: (R[1][2] + R[2][1]) / s
    };
  }

  const s = Math.sqrt(1 + R[2][2] - R[0][0] - R[1][1]) * 2;
  return {
    w: (R[1][0] - R[0][1]) / s,
    x: (R[0][2] + R[2][0]) / s,
    y: (R[1][2] + R[2][1]) / s,
    z: s / 4
  };
}

/** Unit quaternion to rotation matrix. */
export function quatToMatrix(q: Quaternion): number[][] {
  const n = Math.hypot(q.w, q.x, q.y, q.z) || 1;
  const w = q.w / n, x = q.x / n, y = q.y / n, z = q.z / n;

  return [
    [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
    [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
    [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]
  ];
}

/**
 * Spherical linear interpolation, taking the short way round.
 *
 * q and -q are the same rotation, so a pair whose dot product is negative would
 * otherwise be interpolated the long way - up to 360 degrees of travel to reach
 * an attitude a few degrees away. Negating one of them first is what makes this
 * the *shortest* path rather than merely a path.
 *
 * Falls back to normalised linear interpolation when the two are nearly
 * parallel, where sin(theta) approaches zero and the division loses meaning. The
 * two agree to well inside a rotation's useful precision at that separation.
 */
export function slerp(a: Quaternion, b: Quaternion, t: number): Quaternion {
  let dot = a.w * b.w + a.x * b.x + a.y * b.y + a.z * b.z;

  let end = b;
  if (dot < 0) {
    end = { w: -b.w, x: -b.x, y: -b.y, z: -b.z };
    dot = -dot;
  }

  if (dot > 0.9995) {
    const w = a.w + (end.w - a.w) * t;
    const x = a.x + (end.x - a.x) * t;
    const y = a.y + (end.y - a.y) * t;
    const z = a.z + (end.z - a.z) * t;
    const n = Math.hypot(w, x, y, z) || 1;
    return { w: w / n, x: x / n, y: y / n, z: z / n };
  }

  const theta = Math.acos(Math.min(1, dot));
  const sinTheta = Math.sin(theta);
  const ka = Math.sin((1 - t) * theta) / sinTheta;
  const kb = Math.sin(t * theta) / sinTheta;

  return {
    w: ka * a.w + kb * end.w,
    x: ka * a.x + kb * end.x,
    y: ka * a.y + kb * end.y,
    z: ka * a.z + kb * end.z
  };
}

/**
 * Rotation of `angle` about an arbitrary unit axis, by Rodrigues' formula.
 *
 * Used for jogging: "turn 5 degrees about the tool's own Y" needs a rotation
 * about a direction that is not a coordinate axis, which the rpy helpers cannot
 * express without first decomposing something they would then recompose.
 */
export function rotationAboutAxis(axis: Vector3, angle: number): number[][] {
  const n = Math.hypot(axis.x, axis.y, axis.z) || 1;
  const x = axis.x / n, y = axis.y / n, z = axis.z / n;
  const c = Math.cos(angle);
  const s = Math.sin(angle);
  const t = 1 - c;

  return [
    [t * x * x + c,     t * x * y - s * z, t * x * z + s * y],
    [t * x * y + s * z, t * y * y + c,     t * y * z - s * x],
    [t * x * z - s * y, t * y * z + s * x, t * z * z + c]
  ];
}

/** Angle between two orientations, in radians. */
export function angleBetweenQuat(a: Quaternion, b: Quaternion): number {
  const dot = Math.abs(a.w * b.w + a.x * b.x + a.y * b.y + a.z * b.z);
  return 2 * Math.acos(Math.min(1, dot));
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

/**
 * Eigenvalues and eigenvectors of a small symmetric matrix, by cyclic Jacobi.
 *
 * Returns values ascending, with `vectors[i]` the unit eigenvector for
 * `values[i]` - so `vectors[0]` is the direction the matrix flattens hardest.
 *
 * Jacobi rather than anything cleverer because the matrices here are 6x6 and
 * come from a Jacobian, where the interesting case is the one that is nearly
 * singular: Jacobi is unconditionally stable there and gets small eigenvalues
 * right, which is the whole reason for computing them. Cholesky, next door,
 * simply fails on the same input.
 */
export function symmetricEigen(M: number[][]): { values: number[]; vectors: number[][] } {
  const n = M.length;
  const A = M.map(row => [...row]);
  // Accumulated rotations, as columns; transposed to rows on the way out.
  const V: number[][] = Array.from({ length: n }, (_, i) =>
    Array.from({ length: n }, (_, j) => (i === j ? 1 : 0))
  );

  for (let sweep = 0; sweep < 100; sweep++) {
    let off = 0;
    for (let p = 0; p < n; p++) {
      for (let q = p + 1; q < n; q++) off += A[p][q] * A[p][q];
    }
    if (off < 1e-30) break;

    for (let p = 0; p < n; p++) {
      for (let q = p + 1; q < n; q++) {
        if (Math.abs(A[p][q]) < 1e-300) continue;

        // The rotation that zeroes A[p][q]. Written through t = tan(theta) so
        // that a tiny off-diagonal gives a tiny angle rather than a subtraction
        // of two near-equal numbers.
        const theta = (A[q][q] - A[p][p]) / (2 * A[p][q]);
        const t =
          Math.sign(theta || 1) / (Math.abs(theta) + Math.sqrt(theta * theta + 1));
        const c = 1 / Math.sqrt(t * t + 1);
        const s = t * c;

        for (let k = 0; k < n; k++) {
          const akp = A[k][p];
          const akq = A[k][q];
          A[k][p] = c * akp - s * akq;
          A[k][q] = s * akp + c * akq;
        }
        for (let k = 0; k < n; k++) {
          const apk = A[p][k];
          const aqk = A[q][k];
          A[p][k] = c * apk - s * aqk;
          A[q][k] = s * apk + c * aqk;
        }
        for (let k = 0; k < n; k++) {
          const vkp = V[k][p];
          const vkq = V[k][q];
          V[k][p] = c * vkp - s * vkq;
          V[k][q] = s * vkp + c * vkq;
        }
      }
    }
  }

  const order = Array.from({ length: n }, (_, i) => i).sort((a, b) => A[a][a] - A[b][b]);
  return {
    values: order.map(i => A[i][i]),
    vectors: order.map(i => V.map(row => row[i]))
  };
}
