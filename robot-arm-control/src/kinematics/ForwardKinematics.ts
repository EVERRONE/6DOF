import { FKResult } from './types';
import { UrdfChainKinematics } from './UrdfChainKinematics';
import { GeometricJacobian } from './GeometricJacobian';

/**
 * Forward kinematics wrapper.
 * Runtime FK is based on the URDF chain so FK/IK/state and 3D viewer share the same frame.
 */
export class ForwardKinematics {
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

      const result = UrdfChainKinematics.solveFromDegrees(jointAngles);
      return {
        endEffectorPose: result.endEffectorPose,
        jointTransforms: result.jointTransforms,
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
   * Compute analytic geometric Jacobian in URDF runtime frame.
   * Joint angles are in degrees, so derivatives are in units per degree.
   */
  static computeJacobian(jointAngles: number[]): number[][] {
    return GeometricJacobian.compute(jointAngles);
  }

  /**
   * Single-pass FK + Jacobian: traverses the kinematic chain once instead of twice.
   * Use this in IK inner loops to halve chain traversal cost.
   */
  static solveWithJacobian(jointAngles: number[]): { fk: FKResult; jacobian: number[][] } {
    const zeroJacobian = (): number[][] => Array.from({ length: 6 }, () => Array(6).fill(0));
    try {
      if (jointAngles.length !== 6) {
        return {
          fk: {
            endEffectorPose: { position: { x: 0, y: 0, z: 0 }, rotation: { roll: 0, pitch: 0, yaw: 0 } },
            jointTransforms: [],
            success: false,
            error: 'Invalid number of joint angles (expected 6)'
          },
          jacobian: zeroJacobian()
        };
      }
      const combined = UrdfChainKinematics.solveWithJointFrames(jointAngles);
      return {
        fk: {
          endEffectorPose: combined.endEffectorPose,
          jointTransforms: combined.jointTransforms,
          success: true
        },
        jacobian: GeometricJacobian.computeFromFrames(
          combined.jointFrames,
          combined.endEffectorPose.position
        )
      };
    } catch (error) {
      return {
        fk: {
          endEffectorPose: { position: { x: 0, y: 0, z: 0 }, rotation: { roll: 0, pitch: 0, yaw: 0 } },
          jointTransforms: [],
          success: false,
          error: error instanceof Error ? error.message : 'Unknown error'
        },
        jacobian: zeroJacobian()
      };
    }
  }
}
