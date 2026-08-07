// Rounding the corners of a polyline.
//
// A path through a sharp corner has to stop there. Carrying speed round it would
// need a step change in a joint's velocity - the jolt the whole velocity profile
// exists to remove - so the firmware's junction rule scales speed by the cosine
// of the turn and takes it to zero at a right angle. Two linear segments meeting
// square cost a full stop, every lap.
//
// Replacing the corner with an arc tangent to both segments removes the
// discontinuity: the direction now changes continuously, so there is nothing for
// the junction rule to stop for. The trade is that the path no longer passes
// through the waypoint - it cuts the corner by up to the blend radius. That is
// the same trade every industrial controller offers, under its own name: ABB
// calls it a zone, Fanuc CNT, KUKA C_DIS.

import { Vector3 } from '../kinematics/types';

/** One piece of a blended path. */
export type PathPiece =
  | { kind: 'line'; from: Vector3; to: Vector3; length: number }
  | {
      kind: 'arc';
      /** Centre of the blend arc. */
      centre: Vector3;
      /** Where the arc leaves the incoming line. */
      from: Vector3;
      /** Where it rejoins the outgoing one. */
      to: Vector3;
      radius: number;
      /** Swept angle, radians. */
      sweep: number;
      length: number;
      /** The waypoint this arc replaced, for reporting what was cut. */
      cornerIndex: number;
      /** How far the path passes from that waypoint, in metres. */
      cutBy: number;
    };

const sub = (a: Vector3, b: Vector3): Vector3 => ({ x: a.x - b.x, y: a.y - b.y, z: a.z - b.z });
const add = (a: Vector3, b: Vector3): Vector3 => ({ x: a.x + b.x, y: a.y + b.y, z: a.z + b.z });
const scale = (a: Vector3, k: number): Vector3 => ({ x: a.x * k, y: a.y * k, z: a.z * k });
const len = (a: Vector3): number => Math.hypot(a.x, a.y, a.z);
const unit = (a: Vector3): Vector3 => {
  const n = len(a);
  return n < 1e-12 ? { x: 0, y: 0, z: 0 } : scale(a, 1 / n);
};
const dot = (a: Vector3, b: Vector3): number => a.x * b.x + a.y * b.y + a.z * b.z;

/**
 * Turn a polyline into lines and blend arcs.
 *
 * `radius` is the largest arc allowed. Each corner gets the largest arc that
 * fits, which may be smaller: the tangent points have to stay inside both
 * adjoining segments, and two corners sharing a segment must not eat into each
 * other. Half of a segment per corner is the bound, so consecutive blends meet
 * at worst in the middle and never cross.
 *
 * A corner that is nearly straight is left alone - there is nothing to round,
 * and the arc radius needed would run off to infinity. So is one that doubles
 * back on itself, where no arc is tangent to both directions.
 */
export function blendPolyline(points: Vector3[], radius: number): PathPiece[] {
  if (points.length < 2) return [];

  const pieces: PathPiece[] = [];

  // How far back from each corner the blend starts. Index i is the corner at
  // points[i], so the ends are always zero.
  const setback = new Array(points.length).fill(0);
  const arcs = new Array<PathPiece | null>(points.length).fill(null);

  if (radius > 1e-9) {
    for (let i = 1; i < points.length - 1; i++) {
      const corner = points[i];
      const u = unit(sub(points[i - 1], corner)); // back along the incoming leg
      const v = unit(sub(points[i + 1], corner)); // forward along the outgoing leg

      const inLen = len(sub(points[i - 1], corner));
      const outLen = len(sub(points[i + 1], corner));
      if (inLen < 1e-9 || outLen < 1e-9) continue;

      // Angle at the corner, between the two legs. Straight through is pi.
      const cosPhi = Math.max(-1, Math.min(1, dot(u, v)));
      const phi = Math.acos(cosPhi);

      // Nearly straight: nothing to round. Nearly doubled back: no arc is
      // tangent to both legs, and the setback would run to infinity.
      if (phi > Math.PI - 1e-3 || phi < 1e-3) continue;

      // An arc of radius r tangent to both legs touches them at
      //   d = r / tan(phi / 2)
      // from the corner, with its centre at r / sin(phi / 2) along the bisector.
      const half = phi / 2;
      const wanted = radius / Math.tan(half);

      // Never take more than half a leg, so the blend at the other end of that
      // leg has room for its own.
      const allowed = Math.min(wanted, inLen / 2, outLen / 2);
      if (allowed < 1e-6) continue;

      // Shrink the radius to whatever the setback actually permitted.
      const actual = allowed * Math.tan(half);

      const from = add(corner, scale(u, allowed));
      const to = add(corner, scale(v, allowed));

      const bisector = unit(add(u, v));
      const centre = add(corner, scale(bisector, actual / Math.sin(half)));

      // The arc turns through the exterior angle - what is left of a straight
      // line - not the interior one.
      const sweep = Math.PI - phi;

      setback[i] = allowed;
      arcs[i] = {
        kind: 'arc',
        centre,
        from,
        to,
        radius: actual,
        sweep,
        length: actual * sweep,
        cornerIndex: i,
        // The path's closest approach to the waypoint it no longer visits.
        cutBy: len(sub(corner, centre)) - actual
      };
    }
  }

  for (let i = 0; i < points.length - 1; i++) {
    const from = i === 0
      ? points[0]
      : (arcs[i] as Extract<PathPiece, { kind: 'arc' }> | null)?.to ?? points[i];
    const to = i + 1 === points.length - 1
      ? points[points.length - 1]
      : (arcs[i + 1] as Extract<PathPiece, { kind: 'arc' }> | null)?.from ?? points[i + 1];

    const length = len(sub(to, from));
    // A leg fully consumed by the blends at both ends leaves nothing to travel.
    if (length > 1e-9) {
      pieces.push({ kind: 'line', from, to, length });
    }

    const arc = arcs[i + 1];
    if (arc) pieces.push(arc);
  }

  return pieces;
}

/**
 * A point at fraction `s` along a piece.
 *
 * The arc is traced by rotating the start radius about the arc's own normal, so
 * it stays exactly on the circle rather than drifting off it the way a chord
 * interpolation would.
 */
export function pointOnPiece(piece: PathPiece, s: number): Vector3 {
  const t = Math.max(0, Math.min(1, s));

  if (piece.kind === 'line') {
    return {
      x: piece.from.x + (piece.to.x - piece.from.x) * t,
      y: piece.from.y + (piece.to.y - piece.from.y) * t,
      z: piece.from.z + (piece.to.z - piece.from.z) * t
    };
  }

  const a = sub(piece.from, piece.centre);
  const b = sub(piece.to, piece.centre);

  // Rotate a toward b about the plane's normal by t * sweep, using Rodrigues.
  const n = unit(cross(a, b));
  const angle = piece.sweep * t;
  const c = Math.cos(angle);
  const sn = Math.sin(angle);

  const rotated = add(
    add(scale(a, c), scale(cross(n, a), sn)),
    scale(n, dot(n, a) * (1 - c))
  );

  return add(piece.centre, rotated);
}

function cross(a: Vector3, b: Vector3): Vector3 {
  return {
    x: a.y * b.z - a.z * b.y,
    y: a.z * b.x - a.x * b.z,
    z: a.x * b.y - a.y * b.x
  };
}
