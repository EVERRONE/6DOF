// Robot model definition - the single source of truth for kinematics.
//
// The kinematic chain is taken verbatim from URDF.md, and so are the joint
// limits: they come from the mechanical design, mapped into firmware angles and
// set inside the stops with margin. They mirror JOINT_MIN / JOINT_MAX in
// firmware/config.h, which is what the firmware itself enforces. Keeping the
// limits in one place matters: if the solver works in a different box than the
// firmware, the firmware silently clamps the solution and the arm ends up
// somewhere the solver never asked for.
//
// IMPORTANT: no DH parameters are used anywhere. Deriving a DH table from URDF
// origins requires a proper frame-alignment algorithm, and the previous
// hand-waved table ("a = sqrt(x^2 + y^2), d = z") described a different robot
// than the one the 3D viewer renders - off by 330-500 mm. Composing the URDF
// transforms directly is exact, simpler, and automatically consistent with the
// viewer, which builds the same chain out of THREE.Group nodes.

import { Vector3, Rotation3 } from './types';

export const NUM_JOINTS = 6;

/**
 * One revolute joint of the chain.
 *
 * The transform from the parent link frame to this joint's frame is
 *   T = Translate(origin.xyz) * R_rpy(origin.rpy) * Rz(q)
 * where q is the joint variable. Every joint of this arm rotates about its
 * local Z axis (URDF axis="0 0 1"), which is why the chain needs no per-joint
 * axis handling.
 */
export interface JointSpec {
  /** URDF joint name */
  name: string;
  /** Short label used in the UI (J1..J6) */
  label: string;
  /** URDF joint type */
  type: 'revolute' | 'continuous';
  parent: string;
  child: string;
  /** Fixed offset from the parent frame to this joint's frame */
  origin: {
    xyz: Vector3;
    rpy: Rotation3;
  };
  /** Mechanical travel limits in degrees, from firmware/config.h */
  limitDeg: {
    min: number;
    max: number;
  };
}

/**
 * The kinematic chain, base to tool:
 *   linkB -(J1)- link0 -(J2)- link1 -(J3)- link2
 *         -(J4)- link3 -(J5)- link4 -(J6)- link5
 */
export const ROBOT_JOINTS: JointSpec[] = [
  {
    name: 'link0_joint',
    label: 'J1',
    type: 'revolute',
    parent: 'linkB',
    child: 'link0',
    origin: {
      xyz: { x: 0.0, y: 0.0, z: 0.08 },
      rpy: { roll: 0.0, pitch: 0.0, yaw: 0.0 }
    },
    limitDeg: { min: -90, max: 90 }
  },
  {
    name: 'Joint2',
    label: 'J2',
    type: 'revolute',
    parent: 'link0',
    child: 'link1',
    origin: {
      xyz: { x: -0.0375, y: 0.02, z: 0.05595 },
      rpy: { roll: -1.5708, pitch: 0.0, yaw: 0.0 }
    },
    limitDeg: { min: 2, max: 86 }
  },
  {
    name: 'Joint3',
    label: 'J3',
    type: 'revolute',
    parent: 'link1',
    child: 'link2',
    origin: {
      xyz: { x: 0.00027, y: -0.16, z: 0.016 },
      rpy: { roll: -3.14159, pitch: 0.0, yaw: 0.0 }
    },
    limitDeg: { min: 2, max: 104 }
  },
  {
    name: 'link3_joint',
    label: 'J4',
    type: 'revolute',
    parent: 'link2',
    child: 'link3',
    origin: {
      xyz: { x: -0.035, y: 0.0151, z: 0.0364 },
      rpy: { roll: -1.5708, pitch: 0.0, yaw: 1.5708 }
    },
    limitDeg: { min: 2, max: 332 }
  },
  {
    name: 'Joint5',
    label: 'J5',
    type: 'revolute',
    parent: 'link3',
    child: 'link4',
    origin: {
      xyz: { x: 0.0, y: -0.01002, z: 0.10323 },
      rpy: { roll: -1.5708, pitch: 1.5708, yaw: 0.0 }
    },
    limitDeg: { min: 2, max: 222 }
  },
  {
    name: 'Joint6',
    label: 'J6',
    type: 'continuous',
    parent: 'link4',
    child: 'link5',
    origin: {
      xyz: { x: -0.02677, y: 0.0, z: 0.00994 },
      rpy: { roll: 1.5708, pitch: 0.0, yaw: -1.5708 }
    },
    // J6 is continuous in the URDF. The firmware allows a full turn either way.
    // Never give it a zero-width range - the old code did exactly that and
    // froze the joint, costing the solver a degree of freedom.
    limitDeg: { min: -360, max: 360 }
  }
];

/**
 * Tool centre point, expressed in the frame of the last joint (link5).
 *
 * Zero means the TCP is the origin of frame 6, which is where the 3D viewer
 * attaches its end-effector axes marker. Set this once the real gripper /
 * tool geometry is known so that FK, IK and the viewer all agree on what
 * "the tip" means.
 */
export const TOOL_OFFSET: Vector3 = { x: 0, y: 0, z: 0 };

/** Joint limits in degrees, ordered J1..J6. */
export const JOINT_LIMITS_DEG = {
  min: ROBOT_JOINTS.map(j => j.limitDeg.min),
  max: ROBOT_JOINTS.map(j => j.limitDeg.max)
};

/** Joint limits in radians, ordered J1..J6. */
export const JOINT_LIMITS_RAD = {
  min: JOINT_LIMITS_DEG.min.map(d => (d * Math.PI) / 180),
  max: JOINT_LIMITS_DEG.max.map(d => (d * Math.PI) / 180)
};

/**
 * Per-joint speed and acceleration ceilings, mirroring MAX_JOINT_SPEED and
 * MAX_JOINT_ACCEL in firmware/config.h.
 *
 * The firmware enforces these on every move regardless of what the host asks
 * for, so planning against the same numbers is what makes the app's duration
 * estimates and path preview match what the arm actually does. Change them in
 * both places together; a test asserts these exact values.
 */
// Bring-up values, mirroring MAX_JOINT_SPEED / MAX_JOINT_ACCEL in
// firmware/config.h. Reduced to roughly a quarter of the original estimates
// after the arm stalled and vibrated on its first run. Raise both sides
// together, by ear, once the arm moves cleanly.
export const JOINT_MAX_SPEED_DEG_S: number[] = [15, 10, 15, 20, 30, 45];
export const JOINT_MAX_ACCEL_DEG_S2: number[] = [40, 25, 40, 60, 75, 100];

/**
 * Post-homing rest pose in degrees, from firmware/config.h POST_HOME_ANGLES.
 * Used as the default IK seed because it is a known-good, non-singular pose.
 */
// Mirrors POST_HOME_ANGLES in firmware/config.h: where the arm parks after
// homing, measured on the machine rather than taken from the project notes.
export const HOME_POSE_DEG: number[] = [0, 15.0, 41.1, 165.0, 131.0, 0];

// ---------------------------------------------------------------------------
// Firmware angles vs URDF angles
// ---------------------------------------------------------------------------
//
// These are two different conventions for the same joint, and conflating them
// was why the 3D view drew the arm collapsed on the floor while the real one
// stood upright.
//
// The firmware counts every switched joint from its endstop, which sits at the
// minimum end of travel and is a property of where a switch happens to be
// bolted. The URDF counts from the model's own zero, which is a CAD choice.
// Nothing makes those the same pose.
//
// The relation is a sign and an offset per joint:
//
//     urdf = URDF_DIRECTION * (logical - HOME_POSE_DEG)
//
// so URDF zero is the pose the arm parks in after homing. That anchor is not
// arbitrary - at all-zero URDF angles this chain puts the upper arm vertical
// and the forearm horizontal at 311 mm, which is the pose the arm is set to
// rest in, and it is the same anchoring an earlier calibration of this machine
// arrived at independently.
//
// The signs come from that earlier work, which recorded J5 and J6 as running
// opposite to the URDF, combined with its joints counting negatively away from
// their endstops where these count positively - which flips J2 and J4 as well.
export const URDF_DIRECTION: number[] = [1, -1, 1, -1, 1, -1];

const HOME_POSE_RAD_INTERNAL = HOME_POSE_DEG.map(d => (d * Math.PI) / 180);

/** Firmware joint angles (radians) to the URDF chain's angles. */
export function logicalToUrdfRad(qLogical: number[]): number[] {
  return qLogical.map(
    (v, i) => URDF_DIRECTION[i] * (v - HOME_POSE_RAD_INTERNAL[i])
  );
}

/**
 * URDF chain angles (radians) back to firmware angles.
 *
 * URDF_DIRECTION is +/-1, so dividing by it is multiplying by it.
 */
export function urdfToLogicalRad(qUrdf: number[]): number[] {
  return qUrdf.map(
    (v, i) => URDF_DIRECTION[i] * v + HOME_POSE_RAD_INTERNAL[i]
  );
}

/** Centre of the joint range, a useful fallback seed. */
export const MID_POSE_DEG: number[] = ROBOT_JOINTS.map(
  j => (j.limitDeg.min + j.limitDeg.max) / 2
);

export function degToRad(degrees: number[]): number[] {
  return degrees.map(d => (d * Math.PI) / 180);
}

export function radToDeg(radians: number[]): number[] {
  return radians.map(r => (r * 180) / Math.PI);
}

/** Clamp joint angles (radians) into the mechanical limits. */
export function clampToLimitsRad(q: number[]): number[] {
  return q.map((v, i) =>
    Math.max(JOINT_LIMITS_RAD.min[i], Math.min(JOINT_LIMITS_RAD.max[i], v))
  );
}

/** Clamp joint angles (degrees) into the mechanical limits. */
export function clampToLimitsDeg(q: number[]): number[] {
  return q.map((v, i) =>
    Math.max(JOINT_LIMITS_DEG.min[i], Math.min(JOINT_LIMITS_DEG.max[i], v))
  );
}

/** True when every joint angle (degrees) is inside its limits. */
export function isWithinLimitsDeg(q: number[], toleranceDeg = 1e-6): boolean {
  return q.every(
    (v, i) =>
      v >= JOINT_LIMITS_DEG.min[i] - toleranceDeg &&
      v <= JOINT_LIMITS_DEG.max[i] + toleranceDeg
  );
}

/**
 * Names of joints that are outside their limits, for error messages.
 */
export function violatedLimits(qDeg: number[]): string[] {
  const out: string[] = [];
  qDeg.forEach((v, i) => {
    if (v < JOINT_LIMITS_DEG.min[i] - 1e-6 || v > JOINT_LIMITS_DEG.max[i] + 1e-6) {
      out.push(ROBOT_JOINTS[i].label);
    }
  });
  return out;
}
