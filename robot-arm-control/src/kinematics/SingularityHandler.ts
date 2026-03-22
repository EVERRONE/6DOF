import { ForwardKinematics } from './ForwardKinematics';

const DEG_TO_RAD = Math.PI / 180;

const clamp = (value: number, min: number, max: number): number => (
  Math.min(max, Math.max(min, value))
);

export class SingularityHandler {
  static detectFlags(
    jointAnglesDeg: number[],
    minSingularValue: number,
    conditionNumber: number,
    jacobian?: number[][]
  ): string[] {
    const flags: string[] = [];
    if (minSingularValue < 5e-5) {
      flags.push('near_singular_spectrum');
    }
    if (conditionNumber > 1e6) {
      flags.push('ill_conditioned_jacobian');
    }
    const j5 = jointAnglesDeg[4] || 0;
    if (Math.abs(Math.sin(j5 * DEG_TO_RAD)) < 0.02) {
      flags.push('wrist_pitch_singularity');
    }
    // Shoulder singularity: J1 has negligible position effect when
    // the wrist center lies on the base Z axis.
    if (jacobian && jacobian.length >= 3 && jacobian[0]?.length >= 1) {
      const j1Sens = Math.hypot(
        jacobian[0][0] || 0,
        jacobian[1][0] || 0,
        jacobian[2][0] || 0
      );
      if (j1Sens < 0.0008) {
        flags.push('shoulder_axis_ambiguity');
      }
    }
    return flags;
  }

  static directionalDamping(baseDamping: number, minSingularValue: number, boundaryDistanceM?: number | null): number {
    const sigmaFloor = 1e-5;
    const singularScale = clamp((0.005 / Math.max(minSingularValue, sigmaFloor)), 1, 30);
    const boundaryScale = boundaryDistanceM !== undefined && boundaryDistanceM !== null
      ? clamp(1 + (0.04 / Math.max(boundaryDistanceM, 1e-4)), 1, 8)
      : 1;
    return baseDamping * singularScale * boundaryScale;
  }

  static applyWristBypass(jointAnglesDeg: number[]): number[] {
    const out = [...jointAnglesDeg];
    const j5 = out[4] || 0;
    if (Math.abs(Math.sin(j5 * DEG_TO_RAD)) >= 0.015) {
      return out;
    }

    const combined = (out[3] || 0) + (out[5] || 0);
    const candidate = [...out];
    candidate[3] = combined;
    candidate[5] = 0;

    // Verify bypass doesn't degrade FK accuracy before applying.
    const fkOriginal = ForwardKinematics.solve(out);
    const fkBypassed = ForwardKinematics.solve(candidate);
    if (fkOriginal.success && fkBypassed.success) {
      const posErr = Math.hypot(
        fkOriginal.endEffectorPose.position.x - fkBypassed.endEffectorPose.position.x,
        fkOriginal.endEffectorPose.position.y - fkBypassed.endEffectorPose.position.y,
        fkOriginal.endEffectorPose.position.z - fkBypassed.endEffectorPose.position.z
      );
      if (posErr > 0.002) return out; // Revert if bypass introduces > 2mm error
    }

    return candidate;
  }

  /**
   * Near wrist-pitch singularity (sin(J5) ~= 0), J4/J6 become weakly observable as an independent pair.
   * Keep their combined angle, but limit J6 drift to avoid oscillatory branch flips between iterations.
   */
  static stabilizeWristStep(
    previousJointAnglesDeg: number[],
    proposedJointAnglesDeg: number[],
    maxJ6DeltaDeg = 0.35
  ): number[] {
    const out = [...proposedJointAnglesDeg];
    const prev = [...previousJointAnglesDeg];
    const prevJ5 = prev[4] || 0;
    const nextJ5 = out[4] || 0;
    const nearSingularity = Math.min(
      Math.abs(Math.sin(prevJ5 * DEG_TO_RAD)),
      Math.abs(Math.sin(nextJ5 * DEG_TO_RAD))
    ) < 0.03;
    if (!nearSingularity) {
      return out;
    }

    const combined = (out[3] || 0) + (out[5] || 0);
    const prevJ6 = prev[5] || 0;
    const limitedJ6 = clamp(out[5] || 0, prevJ6 - maxJ6DeltaDeg, prevJ6 + maxJ6DeltaDeg);
    out[5] = limitedJ6;
    out[3] = combined - limitedJ6;
    return out;
  }

  /**
   * When J1 has very low XYZ leverage (shoulder-axis ambiguity), prevent large base jumps
   * between consecutive iterations to keep branch continuity.
   */
  static stabilizeShoulderStep(
    previousJointAnglesDeg: number[],
    proposedJointAnglesDeg: number[],
    j1PositionSensitivity: number,
    maxJ1DeltaDeg = 0.8,
    sensitivityThreshold = 0.0008
  ): number[] {
    if (!Number.isFinite(j1PositionSensitivity) || j1PositionSensitivity > sensitivityThreshold) {
      return [...proposedJointAnglesDeg];
    }

    const out = [...proposedJointAnglesDeg];
    const prevJ1 = previousJointAnglesDeg[0] || 0;
    out[0] = clamp(out[0] || 0, prevJ1 - maxJ1DeltaDeg, prevJ1 + maxJ1DeltaDeg);
    return out;
  }
}
