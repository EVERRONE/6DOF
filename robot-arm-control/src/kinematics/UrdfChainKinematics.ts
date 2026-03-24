import {
  Matrix4x4,
  Pose,
  Vector3,
  Rotation3
} from './types';
import { ROBOT_KINEMATIC_CHAIN } from './URDFParser';

export interface UrdfFkResult {
  endEffectorPose: Pose;
  jointTransforms: Matrix4x4[];
}

/** Per-joint data recorded BEFORE applying the joint rotation — used by GeometricJacobian. */
export interface JointFrame {
  origin: Vector3;
  axisWorld: Vector3;
}

export interface UrdfFkWithFramesResult extends UrdfFkResult {
  jointFrames: JointFrame[];
}

/**
 * FK kernel aligned with the URDF chain used by the 3D viewer.
 * This is the single Cartesian frame source of truth for FK/IK/UI readouts.
 */
export class UrdfChainKinematics {
  static solveFromDegrees(jointAnglesDeg: number[]): UrdfFkResult {
    const jointAnglesRad = jointAnglesDeg.map((deg) => (deg * Math.PI) / 180);
    return this.solveFromRadians(jointAnglesRad);
  }

  static solveFromRadians(jointAnglesRad: number[]): UrdfFkResult {
    if (jointAnglesRad.length !== 6) {
      throw new Error(`Invalid number of joint angles (expected 6, got ${jointAnglesRad.length})`);
    }

    let cumulative = this.createIdentityMatrix();
    const transforms: Matrix4x4[] = [];

    for (let i = 0; i < ROBOT_KINEMATIC_CHAIN.length; i++) {
      const joint = ROBOT_KINEMATIC_CHAIN[i];
      const jointTransform = this.composeJointTransform(
        joint.origin.xyz,
        joint.origin.rpy,
        joint.axis,
        jointAnglesRad[i]
      );
      cumulative = this.multiplyMatrices(cumulative, jointTransform);
      transforms.push(this.copyMatrix(cumulative));
    }

    return {
      endEffectorPose: this.extractPose(cumulative),
      jointTransforms: transforms
    };
  }

  private static composeJointTransform(
    xyz: Vector3,
    rpy: Rotation3,
    axis: Vector3,
    angleRad: number
  ): Matrix4x4 {
    const translation = this.translationMatrix(xyz.x, xyz.y, xyz.z);
    // URDF RPY is fixed-axis XYZ, equivalent to Rz(yaw)*Ry(pitch)*Rx(roll).
    const originRotation = this.multiplyMatrices(
      this.rotationZMatrix(rpy.yaw),
      this.multiplyMatrices(
        this.rotationYMatrix(rpy.pitch),
        this.rotationXMatrix(rpy.roll)
      )
    );
    const jointRotation = this.axisAngleMatrix(axis, angleRad);

    return this.multiplyMatrices(
      translation,
      this.multiplyMatrices(originRotation, jointRotation)
    );
  }

  private static axisAngleMatrix(axis: Vector3, angleRad: number): Matrix4x4 {
    const magnitude = Math.sqrt(axis.x * axis.x + axis.y * axis.y + axis.z * axis.z);
    if (magnitude < 1e-12 || Math.abs(angleRad) < 1e-12) {
      return this.createIdentityMatrix();
    }

    const x = axis.x / magnitude;
    const y = axis.y / magnitude;
    const z = axis.z / magnitude;
    const c = Math.cos(angleRad);
    const s = Math.sin(angleRad);
    const oneMinusC = 1 - c;

    return [
      [c + x * x * oneMinusC, x * y * oneMinusC - z * s, x * z * oneMinusC + y * s, 0],
      [y * x * oneMinusC + z * s, c + y * y * oneMinusC, y * z * oneMinusC - x * s, 0],
      [z * x * oneMinusC - y * s, z * y * oneMinusC + x * s, c + z * z * oneMinusC, 0],
      [0, 0, 0, 1]
    ];
  }

  private static extractPose(transform: Matrix4x4): Pose {
    const position: Vector3 = {
      x: transform[0][3],
      y: transform[1][3],
      z: transform[2][3]
    };

    const rotationMatrix = [
      [transform[0][0], transform[0][1], transform[0][2]],
      [transform[1][0], transform[1][1], transform[1][2]],
      [transform[2][0], transform[2][1], transform[2][2]]
    ];

    const rotation = this.rotationMatrixToEuler(rotationMatrix);
    return { position, rotation };
  }

  private static rotationMatrixToEuler(rotation: number[][]): Rotation3 {
    const sy = Math.sqrt(rotation[0][0] * rotation[0][0] + rotation[1][0] * rotation[1][0]);
    const singular = sy < 1e-6;

    let roll: number;
    let pitch: number;
    let yaw: number;

    if (!singular) {
      roll = Math.atan2(rotation[2][1], rotation[2][2]);
      pitch = Math.atan2(-rotation[2][0], sy);
      yaw = Math.atan2(rotation[1][0], rotation[0][0]);
    } else {
      roll = Math.atan2(-rotation[1][2], rotation[1][1]);
      pitch = Math.atan2(-rotation[2][0], sy);
      yaw = 0;
    }

    return { roll, pitch, yaw };
  }

  private static createIdentityMatrix(): Matrix4x4 {
    return [
      [1, 0, 0, 0],
      [0, 1, 0, 0],
      [0, 0, 1, 0],
      [0, 0, 0, 1]
    ];
  }

  private static translationMatrix(x: number, y: number, z: number): Matrix4x4 {
    return [
      [1, 0, 0, x],
      [0, 1, 0, y],
      [0, 0, 1, z],
      [0, 0, 0, 1]
    ];
  }

  private static rotationXMatrix(angle: number): Matrix4x4 {
    const c = Math.cos(angle);
    const s = Math.sin(angle);
    return [
      [1, 0, 0, 0],
      [0, c, -s, 0],
      [0, s, c, 0],
      [0, 0, 0, 1]
    ];
  }

  private static rotationYMatrix(angle: number): Matrix4x4 {
    const c = Math.cos(angle);
    const s = Math.sin(angle);
    return [
      [c, 0, s, 0],
      [0, 1, 0, 0],
      [-s, 0, c, 0],
      [0, 0, 0, 1]
    ];
  }

  private static rotationZMatrix(angle: number): Matrix4x4 {
    const c = Math.cos(angle);
    const s = Math.sin(angle);
    return [
      [c, -s, 0, 0],
      [s, c, 0, 0],
      [0, 0, 1, 0],
      [0, 0, 0, 1]
    ];
  }

  private static multiplyMatrices(a: Matrix4x4, b: Matrix4x4): Matrix4x4 {
    const result: Matrix4x4 = [
      [0, 0, 0, 0],
      [0, 0, 0, 0],
      [0, 0, 0, 0],
      [0, 0, 0, 0]
    ];

    for (let i = 0; i < 4; i++) {
      for (let j = 0; j < 4; j++) {
        for (let k = 0; k < 4; k++) {
          result[i][j] += a[i][k] * b[k][j];
        }
      }
    }

    return result;
  }

  private static copyMatrix(matrix: Matrix4x4): Matrix4x4 {
    return matrix.map((row) => [...row]);
  }

  /**
   * Single-pass FK that also records per-joint axis+origin (before joint rotation)
   * for Jacobian computation. Eliminates the duplicate chain traversal when both
   * FK and Jacobian are needed.
   */
  static solveWithJointFrames(jointAnglesDeg: number[]): UrdfFkWithFramesResult {
    const jointAnglesRad = jointAnglesDeg.map((deg) => (deg * Math.PI) / 180);

    let cumulative = this.createIdentityMatrix();
    const transforms: Matrix4x4[] = [];
    const jointFrames: JointFrame[] = [];

    for (let i = 0; i < ROBOT_KINEMATIC_CHAIN.length; i++) {
      const joint = ROBOT_KINEMATIC_CHAIN[i];

      // Apply origin translation then RPY rotation — same steps as composeJointTransform
      // but split so we can capture the frame BEFORE the joint rotation is applied.
      const originTranslation = this.translationMatrix(
        joint.origin.xyz.x, joint.origin.xyz.y, joint.origin.xyz.z
      );
      const originRotation = this.multiplyMatrices(
        this.rotationZMatrix(joint.origin.rpy.yaw),
        this.multiplyMatrices(
          this.rotationYMatrix(joint.origin.rpy.pitch),
          this.rotationXMatrix(joint.origin.rpy.roll)
        )
      );
      const beforeJoint = this.multiplyMatrices(
        cumulative,
        this.multiplyMatrices(originTranslation, originRotation)
      );

      // World-frame joint axis: rotate local axis by the cumulative orientation.
      const ax = joint.axis;
      const axN = Math.hypot(ax.x, ax.y, ax.z) || 1;
      jointFrames.push({
        origin: { x: beforeJoint[0][3], y: beforeJoint[1][3], z: beforeJoint[2][3] },
        axisWorld: {
          x: (beforeJoint[0][0] * ax.x + beforeJoint[0][1] * ax.y + beforeJoint[0][2] * ax.z) / axN,
          y: (beforeJoint[1][0] * ax.x + beforeJoint[1][1] * ax.y + beforeJoint[1][2] * ax.z) / axN,
          z: (beforeJoint[2][0] * ax.x + beforeJoint[2][1] * ax.y + beforeJoint[2][2] * ax.z) / axN
        }
      });

      // Now apply joint rotation and accumulate.
      cumulative = this.multiplyMatrices(beforeJoint, this.axisAngleMatrix(joint.axis, jointAnglesRad[i]));
      transforms.push(this.copyMatrix(cumulative));
    }

    return {
      endEffectorPose: this.extractPose(cumulative),
      jointTransforms: transforms,
      jointFrames
    };
  }
}
