// Forward kinematics, composed directly from the URDF joint chain.
//
// For each joint i the transform from the previous frame is
//   T_i = Translate(xyz_i) * R_rpy(rpy_i) * Rz(q_i)
// and the pose of frame i in base coordinates is the running product. This is
// exactly the chain the 3D viewer builds out of nested THREE.Group nodes, so
// FK, IK and the render always agree.

import { FKResult, Matrix4x4, Pose, Vector3 } from './types';
import {
  fromXyzRpy,
  getAxis,
  getRotation,
  getTranslation,
  identity4,
  matrixToRpy,
  multiply4,
  rotZ4,
  sub,
  cross,
  transformPoint
} from './linalg';
import { NUM_JOINTS, ROBOT_JOINTS, TOOL_OFFSET, degToRad } from './robotModel';

/**
 * Everything FK produces internally, in radians and metres.
 * `frames[i]` is the pose of joint i's frame in base coordinates.
 */
export interface FKFrames {
  /** Pose of each joint frame in base coordinates, J1..J6 */
  frames: Matrix4x4[];
  /** Pose of the tool centre point in base coordinates */
  tcp: Matrix4x4;
  /** TCP position in metres */
  position: Vector3;
  /** TCP orientation as a 3x3 rotation matrix */
  rotation: number[][];
}

export class ForwardKinematics {
  /**
   * Compute FK from joint angles in RADIANS.
   * This is the primitive everything else is built on.
   */
  static solveRad(q: number[]): FKFrames {
    if (q.length !== NUM_JOINTS) {
      throw new Error(`Expected ${NUM_JOINTS} joint angles, got ${q.length}`);
    }

    const frames: Matrix4x4[] = [];
    let T = identity4();

    for (let i = 0; i < NUM_JOINTS; i++) {
      const joint = ROBOT_JOINTS[i];
      T = multiply4(T, fromXyzRpy(joint.origin.xyz, joint.origin.rpy));
      T = multiply4(T, rotZ4(q[i]));
      frames.push(T);
    }

    // Apply the tool offset in the last frame to reach the TCP.
    const tcp =
      TOOL_OFFSET.x === 0 && TOOL_OFFSET.y === 0 && TOOL_OFFSET.z === 0
        ? T
        : multiply4(T, fromXyzRpy(TOOL_OFFSET, { roll: 0, pitch: 0, yaw: 0 }));

    return {
      frames,
      tcp,
      position: getTranslation(tcp),
      rotation: getRotation(tcp)
    };
  }

  /**
   * Compute FK from joint angles in DEGREES.
   *
   * @param jointAngles - [J1..J6] in degrees
   * @returns End-effector pose plus the transform of every joint frame
   */
  static solve(jointAngles: number[]): FKResult {
    try {
      if (jointAngles.length !== NUM_JOINTS) {
        return {
          endEffectorPose: ForwardKinematics.zeroPose(),
          jointTransforms: [],
          success: false,
          error: `Invalid number of joint angles (expected ${NUM_JOINTS})`
        };
      }

      if (!jointAngles.every(Number.isFinite)) {
        return {
          endEffectorPose: ForwardKinematics.zeroPose(),
          jointTransforms: [],
          success: false,
          error: 'Joint angles contain non-finite values'
        };
      }

      const fk = ForwardKinematics.solveRad(degToRad(jointAngles));

      return {
        endEffectorPose: {
          position: fk.position,
          rotation: matrixToRpy(fk.rotation)
        },
        jointTransforms: fk.frames,
        success: true
      };
    } catch (error) {
      return {
        endEffectorPose: ForwardKinematics.zeroPose(),
        jointTransforms: [],
        success: false,
        error: error instanceof Error ? error.message : 'Unknown error'
      };
    }
  }

  /** TCP position only, from angles in degrees. Convenience for hot paths. */
  static position(jointAnglesDeg: number[]): Vector3 {
    return ForwardKinematics.solveRad(degToRad(jointAnglesDeg)).position;
  }

  /**
   * Geometric Jacobian at `q` (radians), as a 6 x 6 matrix in base coordinates.
   *
   * Rows 0-2 map joint velocity to linear TCP velocity (m/rad).
   * Rows 3-5 map joint velocity to angular TCP velocity (rad/rad).
   *
   * For a revolute joint with world-frame axis z_i through point p_i:
   *   linear  column = z_i x (p_tcp - p_i)
   *   angular column = z_i
   *
   * This is analytic, so it costs one FK pass instead of the seven the old
   * numerical-difference version needed, and it carries no truncation noise.
   * It is also expressed per radian, which is what the damping term assumes -
   * the old one differentiated with respect to degrees while the damping was
   * tuned for radians, making the damping 7x larger than the signal.
   */
  static jacobianRad(q: number[], fk?: FKFrames): number[][] {
    const solved = fk ?? ForwardKinematics.solveRad(q);
    const pTcp = solved.position;

    const J: number[][] = Array.from({ length: 6 }, () => Array(NUM_JOINTS).fill(0));

    for (let i = 0; i < NUM_JOINTS; i++) {
      const frame = solved.frames[i];
      // Rz(q_i) does not change the frame's Z axis or its origin, so both can
      // be read straight off the joint frame.
      const axis = getAxis(frame, 2);
      const origin = getTranslation(frame);
      const lever = sub(pTcp, origin);
      const linear = cross(axis, lever);

      J[0][i] = linear.x;
      J[1][i] = linear.y;
      J[2][i] = linear.z;
      J[3][i] = axis.x;
      J[4][i] = axis.y;
      J[5][i] = axis.z;
    }

    return J;
  }

  /**
   * Geometric Jacobian from angles in DEGREES, still expressed per radian.
   * Prefer jacobianRad inside the solver; this exists for callers that hold
   * degrees.
   */
  static jacobian(jointAnglesDeg: number[]): number[][] {
    return ForwardKinematics.jacobianRad(degToRad(jointAnglesDeg));
  }

  /**
   * Position of every joint frame origin plus the TCP, in base coordinates.
   * Handy for drawing the chain or for reach checks.
   */
  static jointOrigins(jointAnglesDeg: number[]): Vector3[] {
    const fk = ForwardKinematics.solveRad(degToRad(jointAnglesDeg));
    const points = fk.frames.map(getTranslation);
    points.push(fk.position);
    return points;
  }

  /** Reachable distance from the base origin at a given pose. */
  static reachAt(jointAnglesDeg: number[]): number {
    const p = ForwardKinematics.position(jointAnglesDeg);
    return Math.sqrt(p.x * p.x + p.y * p.y + p.z * p.z);
  }

  /** Transform a point given in tool coordinates into base coordinates. */
  static toolToBase(jointAnglesDeg: number[], pointInTool: Vector3): Vector3 {
    const fk = ForwardKinematics.solveRad(degToRad(jointAnglesDeg));
    return transformPoint(fk.tcp, pointInTool);
  }

  private static zeroPose(): Pose {
    return {
      position: { x: 0, y: 0, z: 0 },
      rotation: { roll: 0, pitch: 0, yaw: 0 }
    };
  }
}
