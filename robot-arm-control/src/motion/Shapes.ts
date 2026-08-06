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
import { Rotation3, Vector3 } from '../kinematics/types';
import { Waypoint } from './types';

/** Plane of the figure, in base coordinates. */
export type ShapePlane = 'XY' | 'XZ' | 'YZ';

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
 * Build the waypoints for a circle, and check the arm can actually get round it.
 *
 * Each point is solved from the previous solution, so the check follows the same
 * path the arm will and a figure that is only reachable by reconfiguring
 * halfway shows up as unreachable rather than as a lurch during execution.
 */
export function buildCircle(spec: CircleSpec, seedAngles: number[]): ShapeResult {
  const solver = new InverseKinematics();
  const points = circlePoints(spec);

  const waypoints: Waypoint[] = [];
  const unreachable: number[] = [];
  let seed = [...seedAngles];

  points.forEach((position, index) => {
    const result = spec.orientation
      ? solver.solvePose({ position, rotation: spec.orientation }, seed)
      : solver.solvePosition(position, seed);

    if (result.success) {
      seed = result.jointAngles;
    } else {
      unreachable.push(index);
    }

    waypoints.push({
      id: `${Date.now()}-${index}`,
      position,
      orientation: spec.orientation,
      speed: spec.speed,
      label: `${spec.label ?? 'Circle'} ${index + 1}`
    });
  });

  const ok = unreachable.length === 0;
  const held = spec.orientation ? ' with the tool held' : '';

  return {
    waypoints: ok ? waypoints : [],
    unreachable,
    ok,
    message: ok
      ? `${waypoints.length} points around a ${(spec.radius * 1000).toFixed(0)} mm circle in ${spec.plane}${held}`
      : `${unreachable.length} of ${points.length} points on this circle are out of reach${held}. ` +
        `Try a smaller radius, a different plane, or move the centre` +
        (spec.orientation ? ', or release the tool lock — holding it costs reach.' : '.')
  };
}
