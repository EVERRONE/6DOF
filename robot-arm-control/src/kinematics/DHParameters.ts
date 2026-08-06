// Denavit-Hartenberg Parameters for the 6DOF Robot Arm
import { DHParameter } from './types';
import { ROBOT_KINEMATIC_CHAIN } from './URDFParser';

/**
 * DH Parameter Table for the 6DOF Robot Arm
 * Uses Modified DH Convention (Craig)
 *
 * Derived from URDF joint transformations
 *
 * Modified DH Convention:
 * - Transform from frame {i-1} to frame {i}
 * - alpha(i-1): Twist angle about X(i-1) [rad]
 * - a(i-1): Link length along X(i-1) [m]
 * - d(i): Link offset along Z(i) [m]
 * - theta(i): Joint angle about Z(i) [rad] - VARIABLE
 *
 * Joint mapping:
 * DH Index | Joint Name    | Description
 * ---------|---------------|-------------
 * 0        | link0_joint   | J1 - Base rotation
 * 1        | Joint2        | J2 - Shoulder pitch
 * 2        | Joint3        | J3 - Elbow pitch
 * 3        | link3_joint   | J4 - Wrist roll
 * 4        | Joint5        | J5 - Wrist pitch
 * 5        | Joint6        | J6 - Wrist rotation
 */

/**
 * Creates DH parameter table from URDF kinematic chain
 * Note: theta parameter will be replaced with actual joint angle during FK
 */
export function createDHTable(jointAngles: number[]): DHParameter[] {
  // DH parameters derived from URDF (in meters and radians)
  //
  // Analysis of URDF transformations:
  //
  // J1 (link0_joint): Base rotation around Z
  //   Origin: xyz=(0, 0, 0.08), rpy=(0, 0, 0)
  //   DH: a=0, alpha=0, d=0.08, theta=q1
  //
  // J2 (Joint2): Shoulder pitch
  //   Origin: xyz=(-0.0375, 0.02, 0.05595), rpy=(-π/2, 0, 0)
  //   DH: a=sqrt(0.0375^2 + 0.02^2) ≈ 0.0425, alpha=-π/2, d=0.05595, theta=q2
  //
  // J3 (Joint3): Elbow pitch
  //   Origin: xyz=(0.00027, -0.16, 0.016), rpy=(-π, 0, 0)
  //   DH: a=0.16, alpha=-π, d=0.016, theta=q3
  //
  // J4 (link3_joint): Wrist roll
  //   Origin: xyz=(-0.035, 0.0151, 0.0364), rpy=(-π/2, 0, π/2)
  //   DH: a=sqrt(0.035^2 + 0.0151^2) ≈ 0.0381, alpha=-π/2, d=0.0364, theta=q4+π/2
  //
  // J5 (Joint5): Wrist pitch
  //   Origin: xyz=(0, -0.01002, 0.10323), rpy=(-π/2, π/2, 0)
  //   DH: a=0.01002, alpha=-π/2, d=0.10323, theta=q5+π/2
  //
  // J6 (Joint6): End-effector rotation
  //   Origin: xyz=(-0.02677, 0, 0.00994), rpy=(π/2, 0, -π/2)
  //   DH: a=0.02677, alpha=π/2, d=0.00994, theta=q6-π/2

  const dhTable: DHParameter[] = [
    // J1 - Base rotation
    {
      alpha: 0.0,
      a: 0.0,
      d: 0.08,
      theta: jointAngles[0]
    },

    // J2 - Shoulder pitch
    {
      alpha: -Math.PI / 2,
      a: Math.sqrt(0.0375 * 0.0375 + 0.02 * 0.02),  // ≈ 0.0425 m
      d: 0.05595,
      theta: jointAngles[1]
    },

    // J3 - Elbow pitch
    {
      alpha: -Math.PI,
      a: 0.16,
      d: 0.016,
      theta: jointAngles[2]
    },

    // J4 - Wrist roll
    {
      alpha: -Math.PI / 2,
      a: Math.sqrt(0.035 * 0.035 + 0.0151 * 0.0151),  // ≈ 0.0381 m
      d: 0.0364,
      theta: jointAngles[3] + Math.PI / 2
    },

    // J5 - Wrist pitch
    {
      alpha: -Math.PI / 2,
      a: 0.01002,
      d: 0.10323,
      theta: jointAngles[4] + Math.PI / 2
    },

    // J6 - End-effector rotation
    {
      alpha: Math.PI / 2,
      a: 0.02677,
      d: 0.00994,
      theta: jointAngles[5] - Math.PI / 2
    }
  ];

  return dhTable;
}

/**
 * Get joint limits from URDF (in radians)
 */
export function getJointLimits(): { min: number[]; max: number[] } {
  const min: number[] = [];
  const max: number[] = [];

  for (const joint of ROBOT_KINEMATIC_CHAIN) {
    if (joint.type === 'continuous') {
      min.push(-2 * Math.PI);
      max.push(2 * Math.PI);
    } else if (joint.limit) {
      min.push(joint.limit.lower);
      max.push(joint.limit.upper);
    } else {
      // Safe fallback when no explicit limit is provided.
      min.push(-2 * Math.PI);
      max.push(2 * Math.PI);
    }
  }

  return { min, max };
}

/**
 * Convert degrees to radians
 */
export function degreesToRadians(degrees: number[]): number[] {
  return degrees.map(d => (d * Math.PI) / 180);
}

/**
 * Convert radians to degrees
 */
export function radiansToDegrees(radians: number[]): number[] {
  return radians.map(r => (r * 180) / Math.PI);
}
