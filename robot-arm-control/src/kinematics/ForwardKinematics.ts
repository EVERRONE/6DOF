// Forward Kinematics Implementation using DH Parameters
import { DHParameter, Matrix4x4, FKResult, Pose, Vector3, Rotation3 } from './types';
import { createDHTable, degreesToRadians } from './DHParameters';

/**
 * Forward Kinematics Solver
 *
 * Computes end-effector pose from joint angles using DH parameters
 */
export class ForwardKinematics {
  /**
   * Compute forward kinematics
   * @param jointAngles - Joint angles in DEGREES [J1, J2, J3, J4, J5, J6]
   * @returns End-effector pose and intermediate transforms
   */
  static solve(jointAngles: number[]): FKResult {
    try {
      if (jointAngles.length !== 6) {
        return {
          endEffectorPose: {
            position: { x: 0, y: 0, z: 0 },
            rotation: { roll: 0, pitch: 0, yaw: 0 }
          },
          jointTransforms: [],
          success: false,
          error: 'Invalid number of joint angles (expected 6)'
        };
      }

      // Convert degrees to radians
      const jointAnglesRad = degreesToRadians(jointAngles);

      // Create DH parameter table
      const dhTable = createDHTable(jointAnglesRad);

      // Compute transformation matrices
      const transforms: Matrix4x4[] = [];
      let T_base_to_ee = this.createIdentityMatrix();

      for (let i = 0; i < dhTable.length; i++) {
        const T_i = this.dhTransform(dhTable[i]);
        T_base_to_ee = this.multiplyMatrices(T_base_to_ee, T_i);
        transforms.push(this.copyMatrix(T_base_to_ee));
      }

      // Extract pose from final transformation matrix
      const pose = this.extractPose(T_base_to_ee);

      return {
        endEffectorPose: pose,
        jointTransforms: transforms,
        success: true
      };
    } catch (error) {
      return {
        endEffectorPose: {
          position: { x: 0, y: 0, z: 0 },
          rotation: { roll: 0, pitch: 0, yaw: 0 }
        },
        jointTransforms: [],
        success: false,
        error: error instanceof Error ? error.message : 'Unknown error'
      };
    }
  }

  /**
   * Create 4x4 transformation matrix from DH parameters
   * Modified DH Convention (Craig)
   */
  private static dhTransform(dh: DHParameter): Matrix4x4 {
    const { alpha, a, d, theta } = dh;

    const ca = Math.cos(alpha);
    const sa = Math.sin(alpha);
    const ct = Math.cos(theta);
    const st = Math.sin(theta);

    // Modified DH transformation matrix
    return [
      [ct, -st, 0, a],
      [st * ca, ct * ca, -sa, -d * sa],
      [st * sa, ct * sa, ca, d * ca],
      [0, 0, 0, 1]
    ];
  }

  /**
   * Multiply two 4x4 matrices
   */
  private static multiplyMatrices(A: Matrix4x4, B: Matrix4x4): Matrix4x4 {
    const result: Matrix4x4 = [
      [0, 0, 0, 0],
      [0, 0, 0, 0],
      [0, 0, 0, 0],
      [0, 0, 0, 0]
    ];

    for (let i = 0; i < 4; i++) {
      for (let j = 0; j < 4; j++) {
        for (let k = 0; k < 4; k++) {
          result[i][j] += A[i][k] * B[k][j];
        }
      }
    }

    return result;
  }

  /**
   * Create 4x4 identity matrix
   */
  private static createIdentityMatrix(): Matrix4x4 {
    return [
      [1, 0, 0, 0],
      [0, 1, 0, 0],
      [0, 0, 1, 0],
      [0, 0, 0, 1]
    ];
  }

  /**
   * Copy matrix
   */
  private static copyMatrix(M: Matrix4x4): Matrix4x4 {
    return M.map(row => [...row]);
  }

  /**
   * Extract position and orientation from transformation matrix
   */
  private static extractPose(T: Matrix4x4): Pose {
    // Position is the last column (translation vector)
    const position: Vector3 = {
      x: T[0][3],
      y: T[1][3],
      z: T[2][3]
    };

    // Rotation matrix is the upper-left 3x3 submatrix
    const R = [
      [T[0][0], T[0][1], T[0][2]],
      [T[1][0], T[1][1], T[1][2]],
      [T[2][0], T[2][1], T[2][2]]
    ];

    // Extract Euler angles (ZYX convention - yaw, pitch, roll)
    const rotation = this.rotationMatrixToEuler(R);

    return { position, rotation };
  }

  /**
   * Convert rotation matrix to Euler angles (ZYX convention)
   * Returns angles in radians
   */
  private static rotationMatrixToEuler(R: number[][]): Rotation3 {
    // ZYX Euler angles (yaw-pitch-roll)
    // See: https://www.geometrictools.com/Documentation/EulerAngles.pdf

    let pitch: number;
    let roll: number;
    let yaw: number;

    // Check for gimbal lock
    const sy = Math.sqrt(R[0][0] * R[0][0] + R[1][0] * R[1][0]);

    const singular = sy < 1e-6;

    if (!singular) {
      roll = Math.atan2(R[2][1], R[2][2]);
      pitch = Math.atan2(-R[2][0], sy);
      yaw = Math.atan2(R[1][0], R[0][0]);
    } else {
      roll = Math.atan2(-R[1][2], R[1][1]);
      pitch = Math.atan2(-R[2][0], sy);
      yaw = 0;
    }

    return { roll, pitch, yaw };
  }

  /**
   * Compute Jacobian matrix for velocity kinematics
   * Maps joint velocities to end-effector velocities
   * Returns 6xn Jacobian matrix
   */
  static computeJacobian(jointAngles: number[]): number[][] {
    const epsilon = 1e-6; // Small perturbation for numerical differentiation
    const n = jointAngles.length;
    const J: number[][] = Array.from({ length: 6 }, () => Array(n).fill(0));

    // Get nominal pose
    const nominalFK = this.solve(jointAngles);
    if (!nominalFK.success) {
      return J;
    }

    const nominalPose = nominalFK.endEffectorPose;

    // Numerical differentiation for each joint
    for (let i = 0; i < n; i++) {
      const perturbedAngles = [...jointAngles];
      perturbedAngles[i] += epsilon;

      const perturbedFK = this.solve(perturbedAngles);
      if (!perturbedFK.success) {
        continue;
      }

      const perturbedPose = perturbedFK.endEffectorPose;

      // Position derivatives
      J[0][i] = (perturbedPose.position.x - nominalPose.position.x) / epsilon;
      J[1][i] = (perturbedPose.position.y - nominalPose.position.y) / epsilon;
      J[2][i] = (perturbedPose.position.z - nominalPose.position.z) / epsilon;

      // Orientation derivatives
      J[3][i] = (perturbedPose.rotation.roll - nominalPose.rotation.roll) / epsilon;
      J[4][i] = (perturbedPose.rotation.pitch - nominalPose.rotation.pitch) / epsilon;
      J[5][i] = (perturbedPose.rotation.yaw - nominalPose.rotation.yaw) / epsilon;
    }

    return J;
  }
}
