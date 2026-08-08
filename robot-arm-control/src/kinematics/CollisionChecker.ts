// Self-collision checking.
//
// Joint limits and collisions guard different failures, and this arm needs both.
// A limit is the end of a joint's own travel: past it is metal on metal, and an
// open-loop stepper driven there loses steps in silence. A collision is the arm
// reaching a pose where two of its parts occupy the same space while every joint
// is comfortably inside its range - which a per-joint number cannot express,
// because where the arm fouls depends on where the other joints are.
//
// Measured on this arm: 1.7% of poses drawn uniformly from inside the joint
// limits are self-colliding. Small, but not something a limit can catch.

import { ForwardKinematics } from './ForwardKinematics';
import { NUM_JOINTS, degToRad } from './robotModel';
import {
  ALLOWED_PAIRS,
  COLLISION_MARGIN_M,
  LINK_BOXES,
  OrientedBox
} from './collisionModel';
import { ToolShape, getToolShape, getToolShapeRevision } from './toolGeometry';
import { Matrix4x4 } from './types';

export interface CollisionResult {
  colliding: boolean;
  /** Link index pairs found overlapping, empty when clear. */
  pairs: Array<[number, number]>;
  /** Ready to show: which parts of the arm are in each other's way. */
  message: string;
}

const LINK_NAMES = [
  'shoulder',
  'upper arm',
  'elbow',
  'forearm',
  'wrist',
  'tool'
];

/** Pairs to skip, as a lookup. */
const ALLOWED = new Set(ALLOWED_PAIRS.map(([i, j]) => i * NUM_JOINTS + j));

/**
 * Pairs that are only worth checking once something real is on the flange.
 *
 * The table in collisionModel.ts disables every pair involving the tool link,
 * and with the 4 mm stub it was right to: none of them ever touched. Measured
 * again with a tool on, by sampling 4000 poses across the joint limits with a
 * 120 mm slab at increasing distance from the flange face:
 *
 *   distance    shoulder  upper arm  elbow   forearm   wrist
 *   0-25 mm      10.9%      5.9%     11.6%    86.6%     0.0%
 *   50-75 mm      9.0%      8.0%      5.8%     2.1%     0.0%
 *   100-125 mm    6.8%      5.2%      0.5%     0.0%     0.0%
 *   200-225 mm    2.5%      1.0%      0.0%     0.0%     0.0%
 *   325-350 mm    0.0%      0.0%      0.0%     0.0%     0.0%
 *
 * Shoulder, upper arm and elbow carry real information and are switched on here.
 *
 * The forearm is NOT, and the reason has changed. The old note said it was
 * disabled only because the tool was a placeholder and would matter most once a
 * real tool was fitted. The measurement says otherwise: the forearm's boxes stop
 * about 75 mm past the flange, so the only region where they meet a tool is the
 * region where they enclose the wrist mount and overlap it by construction - 86%
 * of poses in the first 25 mm, with or without a tool. Switching it on would
 * refuse nearly every pose while catching nothing.
 *
 * The wrist stays off because the tool is bolted to it.
 */
const TOOL_PAIRS = [0, 1, 2].map(link => link * NUM_JOINTS + 5);

let allowedCache: { revision: number; set: Set<number> } | null = null;

/** The pairs to skip for the tool currently fitted. */
function allowedPairs(shape: ToolShape | null): Set<number> {
  if (!shape) return ALLOWED;

  const revision = getToolShapeRevision();
  if (allowedCache?.revision !== revision) {
    const set = new Set(ALLOWED);
    for (const key of TOOL_PAIRS) set.delete(key);
    allowedCache = { revision, set };
  }
  return allowedCache.set;
}

/** The fitted tool as a box in flange coordinates, or null when running bare. */
function toolBox(shape: ToolShape): OrientedBox {
  return {
    centre: { ...shape.centre },
    // The flange's own axes: the box is entered in the frame it is bolted to.
    axes: [[1, 0, 0], [0, 1, 0], [0, 0, 1]],
    half: [shape.size.x / 2, shape.size.y / 2, shape.size.z / 2]
  };
}

/** A box placed in base coordinates. */
interface WorldBox {
  centre: number[];
  /** Axes as rows, already rotated into base coordinates. */
  axes: number[][];
  half: number[];
}

function place(box: OrientedBox, frame: Matrix4x4, margin: number): WorldBox {
  const c = box.centre;
  const centre = [
    frame[0][0] * c.x + frame[0][1] * c.y + frame[0][2] * c.z + frame[0][3],
    frame[1][0] * c.x + frame[1][1] * c.y + frame[1][2] * c.z + frame[1][3],
    frame[2][0] * c.x + frame[2][1] * c.y + frame[2][2] * c.z + frame[2][3]
  ];

  const axes = box.axes.map(a => [
    frame[0][0] * a[0] + frame[0][1] * a[1] + frame[0][2] * a[2],
    frame[1][0] * a[0] + frame[1][1] * a[1] + frame[1][2] * a[2],
    frame[2][0] * a[0] + frame[2][1] * a[1] + frame[2][2] * a[2]
  ]);

  return { centre, axes, half: box.half.map(h => h + margin) };
}

/**
 * Separating-axis test for two oriented boxes.
 *
 * Fifteen candidate axes: three from each box, and nine cross products. Finding
 * any one that separates them proves no overlap, so the common case - two parts
 * nowhere near each other - exits on the first or second axis.
 */
export function boxesOverlap(a: WorldBox, b: WorldBox): boolean {
  const R: number[][] = [[0, 0, 0], [0, 0, 0], [0, 0, 0]];
  const absR: number[][] = [[0, 0, 0], [0, 0, 0], [0, 0, 0]];

  for (let i = 0; i < 3; i++) {
    for (let j = 0; j < 3; j++) {
      R[i][j] =
        a.axes[i][0] * b.axes[j][0] +
        a.axes[i][1] * b.axes[j][1] +
        a.axes[i][2] * b.axes[j][2];
      // The epsilon keeps parallel boxes, where a cross product is degenerate,
      // from reporting a separating axis that does not exist.
      absR[i][j] = Math.abs(R[i][j]) + 1e-9;
    }
  }

  const d = [
    b.centre[0] - a.centre[0],
    b.centre[1] - a.centre[1],
    b.centre[2] - a.centre[2]
  ];
  const t = [
    d[0] * a.axes[0][0] + d[1] * a.axes[0][1] + d[2] * a.axes[0][2],
    d[0] * a.axes[1][0] + d[1] * a.axes[1][1] + d[2] * a.axes[1][2],
    d[0] * a.axes[2][0] + d[1] * a.axes[2][1] + d[2] * a.axes[2][2]
  ];

  // A's own axes
  for (let i = 0; i < 3; i++) {
    const ra = a.half[i];
    const rb = b.half[0] * absR[i][0] + b.half[1] * absR[i][1] + b.half[2] * absR[i][2];
    if (Math.abs(t[i]) > ra + rb) return false;
  }

  // B's own axes
  for (let j = 0; j < 3; j++) {
    const ra = a.half[0] * absR[0][j] + a.half[1] * absR[1][j] + a.half[2] * absR[2][j];
    const rb = b.half[j];
    const tj = t[0] * R[0][j] + t[1] * R[1][j] + t[2] * R[2][j];
    if (Math.abs(tj) > ra + rb) return false;
  }

  // The nine cross products
  for (let i = 0; i < 3; i++) {
    const i1 = (i + 1) % 3;
    const i2 = (i + 2) % 3;
    for (let j = 0; j < 3; j++) {
      const j1 = (j + 1) % 3;
      const j2 = (j + 2) % 3;
      const ra = a.half[i1] * absR[i2][j] + a.half[i2] * absR[i1][j];
      const rb = b.half[j1] * absR[i][j2] + b.half[j2] * absR[i][j1];
      const tt = Math.abs(t[i2] * R[i1][j] - t[i1] * R[i2][j]);
      if (tt > ra + rb) return false;
    }
  }

  return true;
}

/** Every link's boxes, placed in base coordinates for one pose. */
function placeAll(jointAnglesDeg: number[], margin: number, shape: ToolShape | null): WorldBox[][] {
  const fk = ForwardKinematics.solveRad(degToRad(jointAnglesDeg));
  const placed = LINK_BOXES.map((boxes, i) => boxes.map(b => place(b, fk.frames[i], margin)));

  // frames[5] is the flange, not the TCP - the tool frame moves the latter and
  // must not move this. A box entered as 150 mm long would otherwise sit 150 mm
  // further out again the moment a tool frame was measured.
  if (shape) placed[5].push(place(toolBox(shape), fk.frames[5], margin));

  return placed;
}

/**
 * Check one pose for self-collision.
 *
 * @param jointAnglesDeg firmware joint angles, J1..J6
 * @param margin extra clearance in metres; defaults to the tuned value
 */
export function checkSelfCollision(
  jointAnglesDeg: number[],
  margin: number = COLLISION_MARGIN_M
): CollisionResult {
  if (jointAnglesDeg.length !== NUM_JOINTS || !jointAnglesDeg.every(Number.isFinite)) {
    return { colliding: false, pairs: [], message: '' };
  }

  const shape = getToolShape();
  const placed = placeAll(jointAnglesDeg, margin, shape);
  const allowed = allowedPairs(shape);
  const pairs: Array<[number, number]> = [];

  for (let i = 0; i < placed.length; i++) {
    for (let j = i + 1; j < placed.length; j++) {
      if (allowed.has(i * NUM_JOINTS + j)) continue;

      let hit = false;
      for (const a of placed[i]) {
        for (const b of placed[j]) {
          if (boxesOverlap(a, b)) { hit = true; break; }
        }
        if (hit) break;
      }
      if (hit) pairs.push([i, j]);
    }
  }

  return {
    colliding: pairs.length > 0,
    pairs,
    message: pairs.length
      ? pairs.map(([i, j]) => `${LINK_NAMES[i]} against ${LINK_NAMES[j]}`).join(', ')
      : ''
  };
}

/** True when the pose is free of self-collision. */
export function isCollisionFree(jointAnglesDeg: number[]): boolean {
  return !checkSelfCollision(jointAnglesDeg).colliding;
}

/**
 * Check a whole path, and say where it first goes wrong.
 *
 * Reported as an index rather than a boolean: a path that collides at point 40
 * of 200 is a different problem from one that collides at the first point, and
 * the operator can only act on the difference if they are told it.
 */
export function firstCollidingPoint(
  poses: number[][]
): { index: number; result: CollisionResult } | null {
  for (let i = 0; i < poses.length; i++) {
    const result = checkSelfCollision(poses[i]);
    if (result.colliding) return { index: i, result };
  }
  return null;
}
