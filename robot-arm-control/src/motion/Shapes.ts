// Turning a described figure into waypoints.
//
// Placing a circle by hand means teaching a few dozen points and getting them
// all slightly wrong. Describing it - centre here, radius r, in the XY plane -
// is exact, and the arm's own reachability is the only thing that can go wrong.
//
// Which is why generating is only half the job. A figure that leaves the
// workspace has to say so here, with which part of it failed, rather than
// producing waypoints that fail one by one during execution with the arm
// already moving.

import { InverseKinematics } from '../kinematics/InverseKinematics';
import { checkSelfCollision } from '../kinematics/CollisionChecker';
import { Rotation3, Vector3 } from '../kinematics/types';
import { ShapePlane, Waypoint, WaypointShape } from './types';

export type { ShapePlane };

export interface CircleSpec {
  /** Centre of the circle, in metres. Normally where the tool is now. */
  centre: Vector3;
  /** Radius in metres. */
  radius: number;
  plane: ShapePlane;
  /** Where on the circle to start, in degrees. 0 is the plane's first axis. */
  startAngleDeg?: number;
  /** Direction of travel. Counter-clockwise seen down the plane's normal. */
  clockwise?: boolean;
  /** Tool orientation to hold all the way round, or undefined to leave it free. */
  orientation?: Rotation3;
  /** Feed rate for each point, in the unit the interpolation mode uses. */
  speed: number;
  /** Prefix for the generated labels. */
  label?: string;
}

export interface ShapeResult {
  waypoints: Waypoint[];
  /** Points the arm cannot reach, by index around the figure. */
  unreachable: number[];
  /** Every point reachable. */
  ok: boolean;
  /** What went wrong, ready to show. */
  message: string;
}

/**
 * How far a straight chord is allowed to cut inside the true curve, in metres.
 *
 * A circle approximated by N segments is short of the real radius by
 * r(1 - cos(pi/N)) at the midpoint of each chord. Fixing that error rather than
 * the segment count means a big circle gets more points and a small one does
 * not waste them, and the figure is as round as this tolerance regardless of
 * radius.
 */
const MAX_SAGITTA_M = 0.0005; // 0.5 mm

/** Fewest segments that keep the sagitta within tolerance. */
export function segmentsForCircle(radius: number, maxSagitta = MAX_SAGITTA_M): number {
  if (radius <= maxSagitta) return 8;
  const halfAngle = Math.acos(Math.max(-1, 1 - maxSagitta / radius));
  if (!(halfAngle > 0)) return 8;
  return Math.min(720, Math.max(8, Math.ceil(Math.PI / halfAngle)));
}

/** The two in-plane unit axes for a plane, in base coordinates. */
function planeAxes(plane: ShapePlane): [Vector3, Vector3] {
  switch (plane) {
    case 'XY':
      return [{ x: 1, y: 0, z: 0 }, { x: 0, y: 1, z: 0 }];
    case 'XZ':
      return [{ x: 1, y: 0, z: 0 }, { x: 0, y: 0, z: 1 }];
    case 'YZ':
      return [{ x: 0, y: 1, z: 0 }, { x: 0, y: 0, z: 1 }];
  }
}

/**
 * A point on a circle at fraction `t` of the way round.
 *
 * The primitive the arc interpolator samples: it means the path follows the
 * true circle at whatever density the motion needs, instead of chording between
 * points fixed when the figure was created.
 */
export function pointOnCircle(shape: WaypointShape, centre: Vector3, t: number): Vector3 {
  const [u, v] = planeAxes(shape.plane);
  const start = ((shape.startAngleDeg ?? 0) * Math.PI) / 180;
  const sign = shape.clockwise ? -1 : 1;
  const a = start + sign * 2 * Math.PI * t;
  const c = Math.cos(a) * shape.radius;
  const s = Math.sin(a) * shape.radius;
  return {
    x: centre.x + u.x * c + v.x * s,
    y: centre.y + u.y * c + v.y * s,
    z: centre.z + u.z * c + v.z * s
  };
}

/** Total path length once round. */
export function circleLength(shape: WaypointShape): number {
  return 2 * Math.PI * shape.radius;
}

/** Points around a circle, without asking whether the arm can reach them. */
export function circlePoints(spec: CircleSpec): Vector3[] {
  const segments = segmentsForCircle(spec.radius);
  const [u, v] = planeAxes(spec.plane);
  const start = ((spec.startAngleDeg ?? 0) * Math.PI) / 180;
  const sign = spec.clockwise ? -1 : 1;

  const points: Vector3[] = [];
  // Round trip: the last point repeats the first, so the path closes rather
  // than leaving a gap the width of one segment.
  for (let i = 0; i <= segments; i++) {
    const a = start + sign * (2 * Math.PI * i) / segments;
    const c = Math.cos(a) * spec.radius;
    const s = Math.sin(a) * spec.radius;
    points.push({
      x: spec.centre.x + u.x * c + v.x * s,
      y: spec.centre.y + u.y * c + v.y * s,
      z: spec.centre.z + u.z * c + v.z * s
    });
  }
  return points;
}

/**
 * Check the arm can get round a figure, without committing to a point list.
 *
 * Sampling here is only for the check: the path itself is sampled later, along
 * the true arc, at whatever density the motion needs. Each point is solved from
 * the previous solution so the check follows the route the arm will take - a
 * figure only reachable by reconfiguring halfway comes out as unreachable
 * rather than as a lurch mid-execution.
 */
export function validateCircle(
  shape: WaypointShape,
  centre: Vector3,
  orientation: Rotation3 | undefined,
  seedAngles: number[]
): {
  ok: boolean;
  unreachable: number;
  colliding: number;
  total: number;
  message: string;
} {
  const solver = new InverseKinematics();
  const segments = segmentsForCircle(shape.radius);
  let seed = [...seedAngles];
  let unreachable = 0;
  let colliding = 0;
  let firstCollision = '';

  for (let i = 0; i <= segments; i++) {
    const position = pointOnCircle(shape, centre, i / segments);
    const result = orientation
      ? solver.solvePose({ position, rotation: orientation }, seed)
      : solver.solvePosition(position, seed);

    if (!result.success) {
      unreachable++;
      continue;
    }

    seed = result.jointAngles;

    // Reachable is not the same as safe: a solution can put the tool exactly
    // where it was asked and fold the arm through itself getting there.
    const hit = checkSelfCollision(result.jointAngles);
    if (hit.colliding) {
      colliding++;
      if (!firstCollision) firstCollision = hit.message;
    }
  }

  const total = segments + 1;
  const held = orientation ? ' with the tool held' : '';
  const mm = (shape.radius * 1000).toFixed(0);

  if (unreachable === 0 && colliding > 0) {
    return {
      ok: false,
      unreachable,
      colliding,
      total,
      message:
        `${colliding} of ${total} points on this circle fold the arm into itself ` +
        `(${firstCollision}). Every point is reachable, so a smaller radius will ` +
        'not help on its own - move the centre, or try another plane.'
    };
  }

  return {
    ok: unreachable === 0 && colliding === 0,
    unreachable,
    colliding,
    total,
    message:
      unreachable === 0
        ? `${mm} mm circle in ${shape.plane}${held}`
        : `${unreachable} of ${total} points on this circle are out of reach${held}. ` +
          'Try a smaller radius, a different plane, or move the centre' +
          (orientation ? ', or release the tool lock \u2014 holding it costs reach.' : '.')
  };
}

/**
 * One waypoint describing a whole circle.
 *
 * A figure is a single intent, so it is a single item: deleting it removes the
 * circle rather than forty points of it, the list shows one row, and the
 * planner is free to sample the arc as finely as the motion needs instead of
 * being stuck with points chosen when it was created.
 */
export function makeCircleWaypoint(
  shape: WaypointShape,
  centre: Vector3,
  orientation: Rotation3 | undefined,
  speed: number
): Waypoint {
  return {
    id: `${Date.now()}-${Math.random().toString(36).slice(2, 8)}`,
    position: centre,
    shape,
    orientation,
    speed,
    label: `${(shape.radius * 1000).toFixed(0)} mm circle, ${shape.plane}`
  };
}
