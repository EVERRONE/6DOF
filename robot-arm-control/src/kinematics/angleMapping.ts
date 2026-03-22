import { FirmwareConfig } from '../types/robot';

const ZERO_OFFSETS = [0, 0, 0, 0, 0, 0];
const HOME_POSE_ANCHOR_JOINTS = [1, 2, 3, 4]; // J2..J5
const DEFAULT_URDF_DIRECTIONS = [1, 1, 1, 1, -1, -1]; // J5/J6 inverted vs logical

const toFinite = (value: number, fallback = 0): number => (
  Number.isFinite(value) ? value : fallback
);

const normalizeDirection = (value: number, fallback = 1): number => {
  if (!Number.isFinite(value) || value === 0) return fallback;
  return value >= 0 ? 1 : -1;
};

const getDirectionAt = (directions: number[], index: number): number => (
  normalizeDirection(directions[index], normalizeDirection(DEFAULT_URDF_DIRECTIONS[index], 1))
);

export const getEffectiveUrdfDirections = (): number[] => [...DEFAULT_URDF_DIRECTIONS];

export const getEffectiveUrdfOffsets = (config: FirmwareConfig | null): number[] => {
  if (!config || !Array.isArray(config.joints) || config.joints.length !== 6) {
    return [...ZERO_OFFSETS];
  }

  const directions = getEffectiveUrdfDirections();
  const offsets = config.joints.map((joint) => toFinite(joint.urdfOffsetDeg));
  const homePose = config.homePose?.jointsDeg;

  if (Array.isArray(homePose) && homePose.length === 6) {
    for (const index of HOME_POSE_ANCHOR_JOINTS) {
      // Keep URDF zero anchored at HP target even when a joint uses inverted direction.
      offsets[index] = -toFinite(homePose[index]) * getDirectionAt(directions, index);
    }
  }

  return offsets;
};

export const logicalToUrdfAngles = (
  logicalDeg: number[],
  offsetsDeg: number[],
  directions = DEFAULT_URDF_DIRECTIONS
): number[] => (
  logicalDeg.map((angle, i) => (
    angle * getDirectionAt(directions, i) + toFinite(offsetsDeg[i])
  ))
);

export const urdfToLogicalAngles = (
  urdfDeg: number[],
  offsetsDeg: number[],
  directions = DEFAULT_URDF_DIRECTIONS
): number[] => (
  urdfDeg.map((angle, i) => (
    (angle - toFinite(offsetsDeg[i])) / getDirectionAt(directions, i)
  ))
);
