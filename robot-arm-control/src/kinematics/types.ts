// Kinematics type definitions for the 6DOF robot arm.
//
// Unit convention, applied everywhere:
//   - Positions and lengths: metres
//   - Angles inside the kinematics core: radians
//   - Angles at the public API boundary (FK input, IK output): degrees
//
// The degrees/radians boundary is spelled out on every function that crosses
// it. Mixing the two silently was the cause of several bugs in the previous
// implementation.

/**
 * 3D Vector (position or direction), metres.
 */
export interface Vector3 {
  x: number;
  y: number;
  z: number;
}

/**
 * Orientation as fixed-axis roll/pitch/yaw in radians, the URDF convention:
 *   R = Rz(yaw) * Ry(pitch) * Rx(roll)
 *
 * This is the same convention THREE.Euler expresses with order 'ZYX', which is
 * what the 3D viewer uses.
 */
export interface Rotation3 {
  roll: number; // about X
  pitch: number; // about Y
  yaw: number; // about Z
}

/**
 * 6DOF pose: position in metres, orientation in radians.
 */
export interface Pose {
  position: Vector3;
  rotation: Rotation3;
}

/**
 * 4x4 homogeneous transformation matrix, row-major.
 */
export type Matrix4x4 = number[][];

/**
 * Forward kinematics result.
 */
export interface FKResult {
  /** TCP pose in base coordinates */
  endEffectorPose: Pose;
  /** Pose of each joint frame in base coordinates, J1..J6 */
  jointTransforms: Matrix4x4[];
  success: boolean;
  error?: string;
}

/**
 * Inverse kinematics result.
 */
export interface IKResult {
  /** Solution in DEGREES, ordered J1..J6. On failure this is the best pose found. */
  jointAngles: number[];
  success: boolean;
  /** Human-readable reason when success is false */
  error?: string;
  /** Total iterations across all seed attempts */
  iterations?: number;
  /** Remaining position error in metres */
  residualError?: number;
  /**
   * Remaining orientation error in radians, when an orientation was asked for.
   * Undefined for a position-only solve, where orientation is not a constraint
   * and any value would be meaningless rather than zero.
   */
  orientationError?: number;
}

/**
 * Jacobian matrix, 6 x 6 for this arm.
 * Rows 0-2 are linear velocity (m/rad), rows 3-5 angular velocity (rad/rad).
 */
export type JacobianMatrix = number[][];
