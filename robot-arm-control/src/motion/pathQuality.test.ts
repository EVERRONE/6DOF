// The five things a path was getting wrong, each pinned by the measurement that
// found it.
//
// These are regression tests in the strict sense: every number below was
// measured on the code as it stood, and the assertion is that it does not come
// back.

import { TrajectoryPlanner, DEFAULT_PLANNER_CONFIG } from './TrajectoryPlanner';
import { PathInterpolator } from './PathInterpolator';
import { blendPolyline, pointOnPiece } from './CornerBlend';
import { Waypoint } from './types';
import { ForwardKinematics } from '../kinematics/ForwardKinematics';
import {
  HOME_POSE_DEG,
  JOINT_MAX_SPEED_DEG_S,
  degToRad
} from '../kinematics/robotModel';
import { matrixToRpy, multiply3, rotationLog, transpose3 } from '../kinematics/linalg';
import { Vector3 } from '../kinematics/types';

const fk = (q: number[]) => ForwardKinematics.solveRad(degToRad(q));

/** The parked pose is the wrist singularity, so tests start clear of it. */
function offSingularity(): number[] {
  const q = [...HOME_POSE_DEG];
  q[4] = 151;
  return q;
}

const angleBetween = (a: number[][], b: number[][]) => {
  const w = rotationLog(multiply3(a, transpose3(b)));
  return (Math.hypot(w.x, w.y, w.z) * 180) / Math.PI;
};

const dist = (a: Vector3, b: Vector3) => Math.hypot(a.x - b.x, a.y - b.y, a.z - b.z);

function waypointAt(position: Vector3, label: string, extra: Partial<Waypoint> = {}): Waypoint {
  return { id: label, label, position, speed: 50, ...extra };
}

// ---------------------------------------------------------------------------
// A. Tool speed
// ---------------------------------------------------------------------------

describe('A. the requested tool speed', () => {
  it('needs a speed per point, because equal Cartesian steps are not equal joint steps', () => {
    const interp = new PathInterpolator();
    const start = offSingularity();
    const from = ForwardKinematics.position(start);

    const seg = interp.interpolateCartesianSpace(
      start, { x: from.x, y: from.y + 0.15, z: from.z }, 50, 200, 10,
      matrixToRpy(fk(start).rotation)
    );

    const travels: number[] = [];
    for (let i = 1; i < seg.points.length; i++) {
      travels.push(Math.max(...seg.points[i].jointAngles.map(
        (v, j) => Math.abs(v - seg.points[i - 1].jointAngles[j])
      )));
    }

    // The premise: 2 mm of tool travel costs between about half a degree and
    // nearly two degrees of joint travel depending on where along the line it
    // falls. One fixed joint speed therefore cannot deliver one tool speed.
    const spread = Math.max(...travels) / Math.min(...travels);
    expect(spread).toBeGreaterThan(2);

    // What the sender now derives, point by point, tracks that spread instead of
    // ignoring it. Constant travel/dt would mean a constant tool speed.
    const speeds = seg.points.slice(1).map((p, i) => {
      const dt = p.time - seg.points[i].time;
      const travel = Math.max(...p.jointAngles.map(
        (v, j) => Math.abs(v - seg.points[i].jointAngles[j])
      ));
      return travel / dt;
    });
    expect(Math.max(...speeds) / Math.min(...speeds)).toBeGreaterThan(2);
  });
});

// ---------------------------------------------------------------------------
// B. Discontinuity
// ---------------------------------------------------------------------------

describe('B. a jump in the joint path', () => {
  it('is avoided by solving the line from its far end, and the wrist turned first', () => {
    const planner = new TrajectoryPlanner({
      interpolationMode: 'linear',
      holdToolOrientation: true
    });

    // Straight from the parked pose, which is the wrist singularity: J4 and J6
    // turn the tool about the same axis there, so solving forwards moves 44
    // degrees from one into the other between two samples 2 mm apart. The far
    // end is 30 degrees clear of the singularity and well conditioned, so
    // walking back from it never needs the reconfiguration at all.
    const start = [...HOME_POSE_DEG];
    const from = ForwardKinematics.position(start);

    const traj = planner.planTrajectory(
      [waypointAt({ x: from.x, y: from.y + 0.1, z: from.z }, 'a')],
      start
    );

    expect(traj.discontinuity).toBeNull();

    // What it costs instead: the wrist has to be turned into position first.
    const entry = traj.segments[0].reconfiguration;
    expect(entry).toBeTruthy();

    // And that turn is free at the tool. J4 and J6 are the same axis here, so
    // counter-rotating them is exactly null-space motion - the arm reconfigures
    // while the tool stands still.
    const swing = Math.max(...entry!.map((v, i) => Math.abs(v - start[i])));
    expect(swing).toBeGreaterThan(20);

    for (let k = 0; k <= 20; k++) {
      const t = k / 20;
      const mid = start.map((v, i) => v + t * (entry![i] - v));
      const p = ForwardKinematics.position(mid);
      expect(dist(p, from) * 1000).toBeLessThan(0.5);
      expect(angleBetween(fk(mid).rotation, fk(start).rotation)).toBeLessThan(0.5);
    }
  });

  it('needs no reconfiguration when the wrist is already conditioned', () => {
    const planner = new TrajectoryPlanner({
      interpolationMode: 'linear',
      holdToolOrientation: true
    });
    const start = offSingularity();
    const from = ForwardKinematics.position(start);

    const traj = planner.planTrajectory(
      [waypointAt({ x: from.x, y: from.y + 0.1, z: from.z }, 'a')],
      start
    );

    expect(traj.discontinuity).toBeNull();
    expect(traj.segments[0].reconfiguration ?? null).toBeNull();
  });

  it('is absent from a path that does not go near the singularity', () => {
    const planner = new TrajectoryPlanner({
      interpolationMode: 'linear',
      holdToolOrientation: true
    });
    const start = offSingularity();
    const from = ForwardKinematics.position(start);

    const traj = planner.planTrajectory(
      [waypointAt({ x: from.x, y: from.y + 0.1, z: from.z }, 'a')],
      start
    );

    expect(traj.discontinuity).toBeNull();
  });
});

// ---------------------------------------------------------------------------
// C. Orientation between waypoints
// ---------------------------------------------------------------------------

describe('C. two waypoints wanting different attitudes', () => {
  it('turns the tool gradually instead of snapping at the boundary', () => {
    const planner = new TrajectoryPlanner({ interpolationMode: 'linear' });
    const start = offSingularity();

    const a = offSingularity(); a[1] = 25;
    const b = offSingularity(); b[1] = 35; b[3] = 120;

    const rotA = matrixToRpy(fk(a).rotation);
    const rotB = matrixToRpy(fk(b).rotation);
    const total = angleBetween(fk(b).rotation, fk(a).rotation);
    expect(total).toBeGreaterThan(30); // the premise: they really do differ

    const traj = planner.planTrajectory(
      [
        waypointAt(ForwardKinematics.position(a), 'a', { orientation: rotA }),
        waypointAt(ForwardKinematics.position(b), 'b', { orientation: rotB })
      ],
      start
    );

    // Largest attitude change between any two consecutive points of the whole
    // path. It used to be the entire 46 degrees, in one step, at the seam.
    const points = TrajectoryPlanner.flattenTrajectory(traj);
    let worst = 0;
    for (let i = 1; i < points.length; i++) {
      worst = Math.max(
        worst,
        angleBetween(fk(points[i].jointAngles).rotation, fk(points[i - 1].jointAngles).rotation)
      );
    }
    expect(worst).toBeLessThan(total / 5);
  });
});

// ---------------------------------------------------------------------------
// D. Speed scale
// ---------------------------------------------------------------------------

describe('D. how much of the arm a path is allowed to use', () => {
  it('defaults to all of it', () => {
    expect(DEFAULT_PLANNER_CONFIG.speedScale).toBe(1);
    // The absolute caps this replaced held J6 - a 360 deg/s joint - at 60.
    expect(JOINT_MAX_SPEED_DEG_S[5]).toBeGreaterThan(60);
  });

  it('scales the whole path rather than clamping one joint', () => {
    const interp = new PathInterpolator();
    const start = offSingularity();
    const end = [...start];
    // Long enough for the profile to reach cruise, so the peak below is the
    // speed cap itself rather than whatever a triangular ramp happened to hit.
    // On a short move halving the scale slows it by sqrt(2), not 2, because a
    // triangular profile's duration depends only on the acceleration.
    end[1] = start[1] + 60;

    const peakOf = (scale: number) => {
      const seg = interp.interpolateJointSpace(start, end, Infinity, Infinity, 40, scale);
      return Math.max(...seg.points.flatMap(p => (p.velocity ?? []).map(Math.abs)));
    };

    // The scale is exactly what it says: a fraction of each joint's own limit.
    expect(peakOf(1)).toBeCloseTo(JOINT_MAX_SPEED_DEG_S[1], 6);
    expect(peakOf(0.5)).toBeCloseTo(JOINT_MAX_SPEED_DEG_S[1] / 2, 6);
  });
});

// ---------------------------------------------------------------------------
// E. Corner blending
// ---------------------------------------------------------------------------

describe('E. rounding a corner', () => {
  const a: Vector3 = { x: 0, y: 0, z: 0 };
  const corner: Vector3 = { x: 0.1, y: 0, z: 0 };
  const c: Vector3 = { x: 0.1, y: 0.1, z: 0 };

  it('replaces a square corner with an arc tangent to both legs', () => {
    const pieces = blendPolyline([a, corner, c], 0.02);
    expect(pieces.map(p => p.kind)).toEqual(['line', 'arc', 'line']);

    const arc = pieces[1] as Extract<(typeof pieces)[number], { kind: 'arc' }>;
    expect(arc.radius).toBeCloseTo(0.02, 6);
    // A right angle: the arc turns through the 90 degrees the corner used to.
    expect((arc.sweep * 180) / Math.PI).toBeCloseTo(90, 3);
    // Every point of the arc is exactly its radius from the centre.
    for (let k = 0; k <= 10; k++) {
      expect(dist(pointOnPiece(arc, k / 10), arc.centre)).toBeCloseTo(arc.radius, 9);
    }
  });

  it('makes the direction continuous, which is what the corner cost', () => {
    // Central difference, clamped at both ends. A forward difference reads zero
    // at s = 1 - the two samples land on the same point - which is the end of
    // every piece and therefore exactly where the junction matters.
    const direction = (p: { kind: string }, s: number) => {
      const h = 1e-4;
      const before = pointOnPiece(p as never, Math.max(0, s - h));
      const after = pointOnPiece(p as never, Math.min(1, s + h));
      const d = { x: after.x - before.x, y: after.y - before.y, z: after.z - before.z };
      const n = Math.hypot(d.x, d.y, d.z) || 1;
      return { x: d.x / n, y: d.y / n, z: d.z / n };
    };

    const square = blendPolyline([a, corner, c], 0);
    const dotSquare =
      direction(square[0], 1).x * direction(square[1], 0).x +
      direction(square[0], 1).y * direction(square[1], 0).y;
    // Unblended: a right angle, so the junction rule takes the speed to zero.
    expect(dotSquare).toBeCloseTo(0, 3);

    const blended = blendPolyline([a, corner, c], 0.02);
    for (let i = 1; i < blended.length; i++) {
      const before = direction(blended[i - 1], 1);
      const after = direction(blended[i], 0);
      const dot = before.x * after.x + before.y * after.y + before.z * after.z;
      // Tangent: nothing left for the junction rule to stop for.
      expect(dot).toBeGreaterThan(0.999);
    }
  });

  it('cuts the corner, and says by how much', () => {
    const arc = blendPolyline([a, corner, c], 0.02)[1] as Extract<
      ReturnType<typeof blendPolyline>[number],
      { kind: 'arc' }
    >;
    // r/sin(45°) - r for a right angle: the path passes this far from the
    // waypoint it no longer visits. Reported rather than silent, because the arm
    // going somewhere other than the taught point is the whole trade.
    expect(arc.cutBy).toBeCloseTo(0.02 * (Math.SQRT2 - 1), 9);
  });

  it('never takes more than half a leg, so neighbouring blends cannot cross', () => {
    // A radius far larger than the legs can give. Each corner takes what it can.
    const pieces = blendPolyline(
      [a, corner, c, { x: 0, y: 0.1, z: 0 }],
      1.0
    );
    const arcs = pieces.filter(p => p.kind === 'arc');
    expect(arcs.length).toBe(2);
    for (const arc of arcs) {
      expect(arc.length).toBeLessThan(0.1 * Math.PI);
    }
    // The legs are consumed but the path still runs start to finish.
    const total = pieces.reduce((sum, p) => sum + p.length, 0);
    expect(total).toBeGreaterThan(0.1);
  });

  it('leaves a nearly straight corner alone', () => {
    const pieces = blendPolyline(
      [a, corner, { x: 0.2, y: 0.00001, z: 0 }],
      0.02
    );
    expect(pieces.every(p => p.kind === 'line')).toBe(true);
  });

  it('plans a real path through rounded corners', () => {
    const planner = new TrajectoryPlanner({
      interpolationMode: 'linear',
      blendRadius: 0.01,
      pointsPerSecond: 10
    });
    const start = offSingularity();
    const p = ForwardKinematics.position(start);

    const traj = planner.planTrajectory(
      [
        waypointAt({ x: p.x, y: p.y + 0.06, z: p.z }, 'a'),
        waypointAt({ x: p.x, y: p.y + 0.06, z: p.z - 0.06 }, 'b'),
        waypointAt({ x: p.x, y: p.y, z: p.z - 0.06 }, 'c')
      ],
      start
    );

    // Three legs and two corners become five pieces.
    expect(traj.segments.length).toBe(5);
    expect(traj.unreachableSamples).toBe(0);
    expect(traj.discontinuity).toBeNull();

    // And the tool path really does round the corners: it passes near the
    // waypoint rather than through it.
    const points = TrajectoryPlanner.flattenTrajectory(traj);
    const corner1 = { x: p.x, y: p.y + 0.06, z: p.z };
    const closest = Math.min(
      ...points.map(pt => dist(ForwardKinematics.position(pt.jointAngles), corner1))
    );
    expect(closest * 1000).toBeGreaterThan(1);
    expect(closest * 1000).toBeLessThan(10);
  });
});

// ---------------------------------------------------------------------------
// F. A path that starts where the wrist is wrong
// ---------------------------------------------------------------------------

describe('F. the reconfiguration a whole path needs', () => {
  it('reaches the trajectory, so execution can turn the wrist before starting', () => {
    const planner = new TrajectoryPlanner({
      interpolationMode: 'linear',
      holdToolOrientation: true
    });

    const start = [...HOME_POSE_DEG];
    const p = ForwardKinematics.position(start);

    const traj = planner.planTrajectory(
      [
        waypointAt({ x: p.x, y: p.y + 0.08, z: p.z }, 'a'),
        waypointAt({ x: p.x + 0.05, y: p.y + 0.08, z: p.z - 0.06 }, 'b')
      ],
      start
    );

    // The first segment is solved from its far end, so it wants the wrist
    // somewhere the arm is not. Held on the segment since the backward pass
    // existed; the trajectory did not carry it, so execution sent the jump as
    // the path's first point instead - unannounced and unchecked.
    expect(traj.segments[0].reconfiguration).toBeTruthy();
    expect(traj.reconfiguration).toEqual(traj.segments[0].reconfiguration);

    const entry = traj.reconfiguration!;
    expect(Math.max(...entry.map((v, i) => Math.abs(v - start[i])))).toBeGreaterThan(20);

    // And it is the same free turn a single move gets: the tool does not move.
    for (let k = 0; k <= 20; k++) {
      const t = k / 20;
      const mid = start.map((v, i) => v + t * (entry[i] - v));
      expect(dist(ForwardKinematics.position(mid), p) * 1000).toBeLessThan(0.5);
    }
  });

  it('is null when the path starts where the wrist is already right', () => {
    const planner = new TrajectoryPlanner({
      interpolationMode: 'linear',
      holdToolOrientation: true
    });
    const start = offSingularity();
    const p = ForwardKinematics.position(start);

    const traj = planner.planTrajectory(
      [waypointAt({ x: p.x, y: p.y + 0.08, z: p.z }, 'a')],
      start
    );

    expect(traj.reconfiguration ?? null).toBeNull();
  });
});
