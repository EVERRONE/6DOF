import { degreesToRadians } from '../../kinematics/DHParameters';
import {
  getEffectiveUrdfDirections,
  getEffectiveUrdfOffsets
} from '../../kinematics/angleMapping';
import { FirmwareConfig } from '../../types/robot';
import { NormalizedConstraintContract, NormalizedJointConstraint } from './types';

export class ConstraintAdapterService {
  static fromFirmwareConfig(config: FirmwareConfig | null): NormalizedConstraintContract | null {
    if (!config || !Array.isArray(config.joints) || config.joints.length !== 6) {
      return null;
    }

    const offsets = getEffectiveUrdfOffsets(config);
    const directions = getEffectiveUrdfDirections();
    const joints: NormalizedJointConstraint[] = [];

    for (let jointIndex = 0; jointIndex < 6; jointIndex++) {
      const joint = config.joints[jointIndex];
      if (!joint || !Number.isFinite(joint.min) || !Number.isFinite(joint.max)) {
        return null;
      }
      const logicalMinDeg = joint.min;
      const logicalMaxDeg = joint.max;
      const direction = directions[jointIndex] ?? 1;
      const offset = offsets[jointIndex] ?? 0;
      const urdfAtLogicalMin = logicalMinDeg * direction + offset;
      const urdfAtLogicalMax = logicalMaxDeg * direction + offset;
      const urdfMinDeg = Math.min(urdfAtLogicalMin, urdfAtLogicalMax);
      const urdfMaxDeg = Math.max(urdfAtLogicalMin, urdfAtLogicalMax);

      joints.push({
        jointIndex,
        logicalMinDeg,
        logicalMaxDeg,
        urdfMinDeg,
        urdfMaxDeg
      });
    }

    const minUrdfDeg = joints.map((joint) => joint.urdfMinDeg);
    const maxUrdfDeg = joints.map((joint) => joint.urdfMaxDeg);

    return {
      limitsSource: 'firmware',
      angleFrame: 'urdf',
      urdfOffsetsDeg: offsets,
      urdfDirections: directions,
      joints,
      jointLimitsRad: {
        min: degreesToRadians(minUrdfDeg),
        max: degreesToRadians(maxUrdfDeg)
      }
    };
  }
}
