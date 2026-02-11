// Inverse Kinematics Implementation using Damped Least Squares
import { IKResult, IKConfig, Vector3, Pose } from './types';
import { ForwardKinematics } from './ForwardKinematics';
import { getJointLimits, radiansToDegrees, degreesToRadians } from './DHParameters';

/**
 * Inverse Kinematics Solver
 *
 * Uses iterative Jacobian-based method with damped least squares (Levenberg-Marquardt)
 * to find joint angles that achieve a desired end-effector position
 */
export class InverseKinematics {
  private config: IKConfig;

  constructor(config?: Partial<IKConfig>) {
    const limits = getJointLimits();

    this.config = {
      maxIterations: config?.maxIterations || 100,
      tolerance: config?.tolerance || 0.001, // 1mm position tolerance
      dampingFactor: config?.dampingFactor || 0.01,
      jointLimits: config?.jointLimits || limits
    };
  }

  /**
   * Solve inverse kinematics for position only (ignore orientation)
   * @param targetPosition - Desired XYZ position in meters
   * @param initialGuess - Initial joint angles in DEGREES (optional)
   * @returns Joint angles solution in DEGREES
   */
  solvePosition(
    targetPosition: Vector3,
    initialGuess?: number[]
  ): IKResult {
    // Use current position as initial guess, or zeros
    let q = initialGuess || [0, 0, 0, 0, 0, 0];
    q = [...q]; // Copy to avoid modifying input

    let iteration = 0;
    let error = Infinity;

    while (iteration < this.config.maxIterations && error > this.config.tolerance) {
      // Compute forward kinematics
      const fk = ForwardKinematics.solve(q);
      if (!fk.success) {
        return {
          jointAngles: q,
          success: false,
          error: 'Forward kinematics failed',
          iterations: iteration
        };
      }

      // Compute position error
      const dx = targetPosition.x - fk.endEffectorPose.position.x;
      const dy = targetPosition.y - fk.endEffectorPose.position.y;
      const dz = targetPosition.z - fk.endEffectorPose.position.z;

      error = Math.sqrt(dx * dx + dy * dy + dz * dz);

      // Check if converged
      if (error < this.config.tolerance) {
        break;
      }

      // Compute Jacobian (position part only - first 3 rows)
      const J_full = ForwardKinematics.computeJacobian(q);
      const J = J_full.slice(0, 3); // Position Jacobian (3x6)

      // Error vector
      const e = [dx, dy, dz];

      // Compute damped pseudo-inverse: (J^T * J + λI)^-1 * J^T
      const dq = this.dampedLeastSquares(J, e);

      // Update joint angles (in degrees)
      for (let i = 0; i < 6; i++) {
        q[i] += dq[i];
      }

      // Apply joint limits
      q = this.clampJointAngles(q);

      iteration++;
    }

    const converged = error < this.config.tolerance;

    return {
      jointAngles: q,
      success: converged,
      error: converged ? undefined : `Failed to converge (error: ${error.toFixed(4)}m)`,
      iterations: iteration,
      residualError: error
    };
  }

  /**
   * Solve inverse kinematics for full pose (position + orientation)
   * @param targetPose - Desired pose (position + orientation)
   * @param initialGuess - Initial joint angles in DEGREES (optional)
   * @returns Joint angles solution in DEGREES
   */
  solvePose(
    targetPose: Pose,
    initialGuess?: number[]
  ): IKResult {
    let q = initialGuess || [0, 0, 0, 0, 0, 0];
    q = [...q];

    let iteration = 0;
    let error = Infinity;

    while (iteration < this.config.maxIterations && error > this.config.tolerance) {
      const fk = ForwardKinematics.solve(q);
      if (!fk.success) {
        return {
          jointAngles: q,
          success: false,
          error: 'Forward kinematics failed',
          iterations: iteration
        };
      }

      // Position error
      const dx = targetPose.position.x - fk.endEffectorPose.position.x;
      const dy = targetPose.position.y - fk.endEffectorPose.position.y;
      const dz = targetPose.position.z - fk.endEffectorPose.position.z;

      // Orientation error (simplified - just difference in Euler angles)
      const droll = targetPose.rotation.roll - fk.endEffectorPose.rotation.roll;
      const dpitch = targetPose.rotation.pitch - fk.endEffectorPose.rotation.pitch;
      const dyaw = targetPose.rotation.yaw - fk.endEffectorPose.rotation.yaw;

      // Combined error (weighted)
      const posError = Math.sqrt(dx * dx + dy * dy + dz * dz);
      const oriError = Math.sqrt(droll * droll + dpitch * dpitch + dyaw * dyaw);
      error = posError + 0.1 * oriError; // Weight orientation less

      if (error < this.config.tolerance) {
        break;
      }

      // Full Jacobian (6x6)
      const J = ForwardKinematics.computeJacobian(q);

      // Error vector
      const e = [dx, dy, dz, droll, dpitch, dyaw];

      // Compute damped pseudo-inverse
      const dq = this.dampedLeastSquares(J, e);

      // Update joint angles
      for (let i = 0; i < 6; i++) {
        q[i] += dq[i];
      }

      q = this.clampJointAngles(q);

      iteration++;
    }

    const converged = error < this.config.tolerance;

    return {
      jointAngles: q,
      success: converged,
      error: converged ? undefined : `Failed to converge (error: ${error.toFixed(4)})`,
      iterations: iteration,
      residualError: error
    };
  }

  /**
   * Damped Least Squares (Levenberg-Marquardt) solver
   * Computes Δq = (J^T * J + λI)^-1 * J^T * e
   */
  private dampedLeastSquares(J: number[][], e: number[]): number[] {
    const m = J.length; // Number of task dimensions
    const n = J[0].length; // Number of joints

    // Compute J^T
    const JT = this.transpose(J);

    // Compute J^T * J
    const JTJ = this.multiplyMatrices(JT, J);

    // Add damping: J^T * J + λI
    for (let i = 0; i < n; i++) {
      JTJ[i][i] += this.config.dampingFactor * this.config.dampingFactor;
    }

    // Compute J^T * e
    const JTe: number[] = [];
    for (let i = 0; i < n; i++) {
      let sum = 0;
      for (let j = 0; j < m; j++) {
        sum += JT[i][j] * e[j];
      }
      JTe.push(sum);
    }

    // Solve (J^T * J + λI) * Δq = J^T * e
    // Use Gauss-Seidel iterative solver for simplicity
    const dq = this.gaussSeidel(JTJ, JTe);

    return dq;
  }

  /**
   * Gauss-Seidel iterative solver for linear system Ax = b
   */
  private gaussSeidel(A: number[][], b: number[]): number[] {
    const n = b.length;
    let x = Array(n).fill(0);

    const iterations = 20;
    for (let iter = 0; iter < iterations; iter++) {
      for (let i = 0; i < n; i++) {
        let sum = b[i];
        for (let j = 0; j < n; j++) {
          if (j !== i) {
            sum -= A[i][j] * x[j];
          }
        }
        x[i] = sum / A[i][i];
      }
    }

    return x;
  }

  /**
   * Matrix transpose
   */
  private transpose(M: number[][]): number[][] {
    const rows = M.length;
    const cols = M[0].length;
    const MT: number[][] = Array.from({ length: cols }, () => Array(rows).fill(0));

    for (let i = 0; i < rows; i++) {
      for (let j = 0; j < cols; j++) {
        MT[j][i] = M[i][j];
      }
    }

    return MT;
  }

  /**
   * Matrix multiplication
   */
  private multiplyMatrices(A: number[][], B: number[][]): number[][] {
    const rowsA = A.length;
    const colsA = A[0].length;
    const colsB = B[0].length;

    const C: number[][] = Array.from({ length: rowsA }, () => Array(colsB).fill(0));

    for (let i = 0; i < rowsA; i++) {
      for (let j = 0; j < colsB; j++) {
        for (let k = 0; k < colsA; k++) {
          C[i][j] += A[i][k] * B[k][j];
        }
      }
    }

    return C;
  }

  /**
   * Clamp joint angles to limits (in degrees)
   */
  private clampJointAngles(q: number[]): number[] {
    const qClamped = [...q];

    // Convert limits from radians to degrees
    const minDeg = radiansToDegrees(this.config.jointLimits.min);
    const maxDeg = radiansToDegrees(this.config.jointLimits.max);

    for (let i = 0; i < q.length; i++) {
      qClamped[i] = Math.max(minDeg[i], Math.min(maxDeg[i], q[i]));
    }

    return qClamped;
  }

  /**
   * Update solver configuration
   */
  updateConfig(config: Partial<IKConfig>): void {
    this.config = { ...this.config, ...config };
  }
}
