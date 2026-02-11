// Kinematics type definitions for 6DOF robot arm

/**
 * 3D Vector (position or direction)
 */
export interface Vector3 {
  x: number;
  y: number;
  z: number;
}

/**
 * 3D Rotation (Euler angles in radians)
 */
export interface Rotation3 {
  roll: number;   // Rotation around X axis
  pitch: number;  // Rotation around Y axis
  yaw: number;    // Rotation around Z axis
}

/**
 * 6DOF Pose (position + orientation)
 */
export interface Pose {
  position: Vector3;
  rotation: Rotation3;
}

/**
 * Denavit-Hartenberg Parameters (Modified DH Convention)
 *
 * Modified DH parameters (Craig convention):
 * - alpha: Twist angle between Z(i-1) and Z(i) about X(i-1) [radians]
 * - a: Link length from Z(i-1) to Z(i) along X(i-1) [meters]
 * - d: Link offset from X(i-1) to X(i) along Z(i) [meters]
 * - theta: Joint angle between X(i-1) and X(i) about Z(i) [radians]
 */
export interface DHParameter {
  alpha: number;  // Twist angle (rad)
  a: number;      // Link length (m)
  d: number;      // Link offset (m)
  theta: number;  // Joint angle (rad) - variable for revolute joints
}

/**
 * 4x4 Homogeneous Transformation Matrix
 */
export type Matrix4x4 = number[][];

/**
 * URDF Joint definition
 */
export interface URDFJoint {
  name: string;
  type: 'revolute' | 'continuous' | 'fixed';
  parent: string;
  child: string;
  origin: {
    xyz: Vector3;
    rpy: Rotation3;
  };
  axis: Vector3;
  limit?: {
    lower: number;
    upper: number;
    effort: number;
    velocity: number;
  };
}

/**
 * Forward Kinematics Result
 */
export interface FKResult {
  endEffectorPose: Pose;
  jointTransforms: Matrix4x4[];  // Transform for each joint
  success: boolean;
  error?: string;
}

/**
 * Inverse Kinematics Result
 */
export interface IKResult {
  jointAngles: number[];  // Solution in radians
  success: boolean;
  error?: string;
  iterations?: number;
  residualError?: number;
}

/**
 * IK Solver Configuration
 */
export interface IKConfig {
  maxIterations: number;
  tolerance: number;          // Position tolerance in meters
  dampingFactor: number;      // Damping for Jacobian pseudo-inverse
  jointLimits: {
    min: number[];
    max: number[];
  };
}

/**
 * Jacobian Matrix (6 x n) for velocity kinematics
 * Maps joint velocities to end-effector velocities
 */
export type JacobianMatrix = number[][];
