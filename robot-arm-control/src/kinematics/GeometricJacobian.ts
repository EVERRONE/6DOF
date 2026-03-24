import { Matrix4x4, Rotation3, Vector3 } from './types';
import { ROBOT_KINEMATIC_CHAIN } from './URDFParser';
import { JointFrame } from './UrdfChainKinematics';

const DEG_TO_RAD = Math.PI / 180;

// Local name to avoid collision with the exported JointFrame from UrdfChainKinematics.
type LocalJointFrame = {
  origin: Vector3;
  axis: Vector3;
};

const identity = (): Matrix4x4 => ([
  [1, 0, 0, 0],
  [0, 1, 0, 0],
  [0, 0, 1, 0],
  [0, 0, 0, 1]
]);

const mul4 = (a: Matrix4x4, b: Matrix4x4): Matrix4x4 => {
  const out: Matrix4x4 = [
    [0, 0, 0, 0],
    [0, 0, 0, 0],
    [0, 0, 0, 0],
    [0, 0, 0, 0]
  ];
  for (let i = 0; i < 4; i++) {
    for (let j = 0; j < 4; j++) {
      for (let k = 0; k < 4; k++) {
        out[i][j] += a[i][k] * b[k][j];
      }
    }
  }
  return out;
};

const translation = (x: number, y: number, z: number): Matrix4x4 => ([
  [1, 0, 0, x],
  [0, 1, 0, y],
  [0, 0, 1, z],
  [0, 0, 0, 1]
]);

const rotX = (a: number): Matrix4x4 => {
  const c = Math.cos(a);
  const s = Math.sin(a);
  return [
    [1, 0, 0, 0],
    [0, c, -s, 0],
    [0, s, c, 0],
    [0, 0, 0, 1]
  ];
};

const rotY = (a: number): Matrix4x4 => {
  const c = Math.cos(a);
  const s = Math.sin(a);
  return [
    [c, 0, s, 0],
    [0, 1, 0, 0],
    [-s, 0, c, 0],
    [0, 0, 0, 1]
  ];
};

const rotZ = (a: number): Matrix4x4 => {
  const c = Math.cos(a);
  const s = Math.sin(a);
  return [
    [c, -s, 0, 0],
    [s, c, 0, 0],
    [0, 0, 1, 0],
    [0, 0, 0, 1]
  ];
};

const axisAngle = (axis: Vector3, angleRad: number): Matrix4x4 => {
  const n = Math.hypot(axis.x, axis.y, axis.z);
  if (n < 1e-12 || Math.abs(angleRad) < 1e-12) return identity();
  const x = axis.x / n;
  const y = axis.y / n;
  const z = axis.z / n;
  const c = Math.cos(angleRad);
  const s = Math.sin(angleRad);
  const oc = 1 - c;
  return [
    [c + x * x * oc, x * y * oc - z * s, x * z * oc + y * s, 0],
    [y * x * oc + z * s, c + y * y * oc, y * z * oc - x * s, 0],
    [z * x * oc - y * s, z * y * oc + x * s, c + z * z * oc, 0],
    [0, 0, 0, 1]
  ];
};

const transformDirection = (t: Matrix4x4, v: Vector3): Vector3 => ({
  x: t[0][0] * v.x + t[0][1] * v.y + t[0][2] * v.z,
  y: t[1][0] * v.x + t[1][1] * v.y + t[1][2] * v.z,
  z: t[2][0] * v.x + t[2][1] * v.y + t[2][2] * v.z
});

const normalize = (v: Vector3): Vector3 => {
  const n = Math.hypot(v.x, v.y, v.z);
  if (n < 1e-12) return { x: 0, y: 0, z: 1 };
  return { x: v.x / n, y: v.y / n, z: v.z / n };
};

const cross = (a: Vector3, b: Vector3): Vector3 => ({
  x: a.y * b.z - a.z * b.y,
  y: a.z * b.x - a.x * b.z,
  z: a.x * b.y - a.y * b.x
});

const sub = (a: Vector3, b: Vector3): Vector3 => ({
  x: a.x - b.x,
  y: a.y - b.y,
  z: a.z - b.z
});

const positionFrom = (t: Matrix4x4): Vector3 => ({
  x: t[0][3],
  y: t[1][3],
  z: t[2][3]
});

const originRotation = (rpy: Rotation3): Matrix4x4 => (
  mul4(rotZ(rpy.yaw), mul4(rotY(rpy.pitch), rotX(rpy.roll)))
);

export class GeometricJacobian {
  static compute(jointAnglesDeg: number[]): number[][] {
    if (jointAnglesDeg.length !== ROBOT_KINEMATIC_CHAIN.length) {
      return Array.from({ length: 6 }, () => Array(6).fill(0));
    }

    const qRad = jointAnglesDeg.map((deg) => deg * DEG_TO_RAD);
    const frames: LocalJointFrame[] = [];
    let cumulative = identity();

    for (let i = 0; i < ROBOT_KINEMATIC_CHAIN.length; i++) {
      const joint = ROBOT_KINEMATIC_CHAIN[i];
      const tOrigin = mul4(
        translation(joint.origin.xyz.x, joint.origin.xyz.y, joint.origin.xyz.z),
        originRotation(joint.origin.rpy)
      );

      const beforeJoint = mul4(cumulative, tOrigin);
      const axisWorld = normalize(transformDirection(beforeJoint, joint.axis));
      frames.push({
        origin: positionFrom(beforeJoint),
        axis: axisWorld
      });

      cumulative = mul4(beforeJoint, axisAngle(joint.axis, qRad[i]));
    }

    const pEe = positionFrom(cumulative);
    const jacobian = Array.from({ length: 6 }, () => Array(6).fill(0));

    for (let i = 0; i < 6; i++) {
      const axis = frames[i].axis;
      const delta = sub(pEe, frames[i].origin);
      const linear = cross(axis, delta);

      // FK/IK APIs use degrees, so Jacobian must be in units per degree.
      jacobian[0][i] = linear.x * DEG_TO_RAD;
      jacobian[1][i] = linear.y * DEG_TO_RAD;
      jacobian[2][i] = linear.z * DEG_TO_RAD;
      jacobian[3][i] = axis.x * DEG_TO_RAD;
      jacobian[4][i] = axis.y * DEG_TO_RAD;
      jacobian[5][i] = axis.z * DEG_TO_RAD;
    }

    return jacobian;
  }

  /**
   * Compute Jacobian from pre-computed joint frames (no second chain traversal).
   * Call this after UrdfChainKinematics.solveWithJointFrames() to share the chain
   * traversal work. pEe is the end-effector position from the FK result.
   */
  static computeFromFrames(
    jointFrames: JointFrame[],
    pEe: Vector3
  ): number[][] {
    const jacobian = Array.from({ length: 6 }, () => Array(6).fill(0));

    for (let i = 0; i < jointFrames.length && i < 6; i++) {
      const { origin, axisWorld } = jointFrames[i];
      const dx = pEe.x - origin.x;
      const dy = pEe.y - origin.y;
      const dz = pEe.z - origin.z;

      // Linear velocity: axis × (pEe - origin), scaled to degree-input convention.
      jacobian[0][i] = (axisWorld.y * dz - axisWorld.z * dy) * DEG_TO_RAD;
      jacobian[1][i] = (axisWorld.z * dx - axisWorld.x * dz) * DEG_TO_RAD;
      jacobian[2][i] = (axisWorld.x * dy - axisWorld.y * dx) * DEG_TO_RAD;
      // Angular velocity: world-frame axis, scaled to degree-input convention.
      jacobian[3][i] = axisWorld.x * DEG_TO_RAD;
      jacobian[4][i] = axisWorld.y * DEG_TO_RAD;
      jacobian[5][i] = axisWorld.z * DEG_TO_RAD;
    }

    return jacobian;
  }
}
