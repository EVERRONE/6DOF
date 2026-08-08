import { Vector3 } from '../kinematics/types';

/**
 * Turning a spoken direction into a world-frame vector.
 *
 * The runtime world frame is X forward, Y left, Z up (docs/KINEMATICS.md), so
 * base-frame "right" is -Y. But "right" is said by a person, and a person is
 * not always standing behind the base looking along +X — so the view frame
 * rotates the whole basis by the angle the operator is sitting at.
 */

export type AgentDirection = 'right' | 'left' | 'forward' | 'backward' | 'up' | 'down';
export type AgentFrame = 'base' | 'view';

export const AGENT_DIRECTIONS: AgentDirection[] = [
  'right',
  'left',
  'forward',
  'backward',
  'up',
  'down'
];

export const AGENT_FRAMES: AgentFrame[] = ['base', 'view'];

// Float noise turns a clean 0 into 6.1e-17 and a clean 0 into -0, both of which
// look alarming in an API response a human is reading to check their frame.
const tidy = (value: number): number => {
  const rounded = Math.round(value * 1e9) / 1e9;
  return Object.is(rounded, -0) ? 0 : rounded;
};

/**
 * Unit vector for `direction`, expressed in the world frame.
 *
 * `viewYawDeg` is the heading of the operator's "forward" measured in the base
 * frame, so 0 makes the view frame identical to the base frame.
 */
export const resolveDirectionVector = (
  direction: AgentDirection,
  frame: AgentFrame = 'base',
  viewYawDeg = 0
): Vector3 => {
  const yawRad = frame === 'view' ? (viewYawDeg * Math.PI) / 180 : 0;
  const c = Math.cos(yawRad);
  const s = Math.sin(yawRad);

  switch (direction) {
    case 'forward':
      return { x: tidy(c), y: tidy(s), z: 0 };
    case 'backward':
      return { x: tidy(-c), y: tidy(-s), z: 0 };
    case 'left':
      return { x: tidy(-s), y: tidy(c), z: 0 };
    case 'right':
      return { x: tidy(s), y: tidy(-c), z: 0 };
    case 'up':
      return { x: 0, y: 0, z: 1 };
    case 'down':
      return { x: 0, y: 0, z: -1 };
    default: {
      // Exhaustiveness guard: adding a direction without handling it here
      // becomes a compile error rather than a silently wrong move.
      const unreachable: never = direction;
      throw new Error(`Unhandled direction: ${unreachable}`);
    }
  }
};

/** Applies a displacement of `distanceMm` along `direction` to a pose in metres. */
export const applyDirectionalOffset = (
  positionM: Vector3,
  direction: AgentDirection,
  distanceMm: number,
  frame: AgentFrame = 'base',
  viewYawDeg = 0
): { target: Vector3; unitVector: Vector3; deltaMm: Vector3 } => {
  const unitVector = resolveDirectionVector(direction, frame, viewYawDeg);
  const deltaM = distanceMm / 1000;

  return {
    unitVector,
    deltaMm: {
      x: tidy(unitVector.x * distanceMm),
      y: tidy(unitVector.y * distanceMm),
      z: tidy(unitVector.z * distanceMm)
    },
    target: {
      x: positionM.x + unitVector.x * deltaM,
      y: positionM.y + unitVector.y * deltaM,
      z: positionM.z + unitVector.z * deltaM
    }
  };
};
