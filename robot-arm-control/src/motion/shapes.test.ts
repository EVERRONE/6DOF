import {
  circlePoints,
  makeCircleWaypoint,
  pointOnCircle,
  segmentsForCircle,
  validateCircle
} from './Shapes';
import { PathInterpolator } from './PathInterpolator';
import { TrajectoryPlanner } from './TrajectoryPlanner';
import { WaypointShape } from './types';
import { ForwardKinematics } from '../kinematics/ForwardKinematics';
import { HOME_POSE_DEG } from '../kinematics/robotModel';
import { rotationLog, multiply3, transpose3 } from '../kinematics/linalg';
import { Vector3 } from '../kinematics/types';

function dist(a: Vector3, b: Vector3): number {
  return Math.hypot(a.x - b.x, a.y - b.y, a.z - b.z);
}

function tumbleDeg(from: number[], to: number[]): number {
  const Ra = ForwardKinematics.solveRad(from.map(d => (d * Math.PI) / 180)).rotation;
  const Rb = ForwardKinematics.solveRad(to.map(d => (d * Math.PI) / 180)).rotation;
  const w = rotationLog(multiply3(Rb, transpose3(Ra)));
  return (Math.hypot(w.x, w.y, w.z) * 180) / Math.PI;
}

const seed = [...HOME_POSE_DEG];
const home = ForwardKinematics.solve(HOME_POSE_DEG);
const centre = home.endEffectorPose.position;
const rotation = home.endEffectorPose.rotation;

describe('circle geometry', () => {
  it('samples densely enough to stay round, whatever the radius', () => {
    for (const radius of [0.01, 0.03, 0.1, 0.25]) {
      const n = segmentsForCircle(radius);
      expect(radius * (1 - Math.cos(Math.PI / n))).toBeLessThanOrEqual(0.0005 + 1e-12);
    }
  });

  it('puts every point on the circle, in the right plane', () => {
    const c = { x: 0.1, y: -0.05, z: 0.3 };
    const planes = [['XY', 'z'], ['XZ', 'y'], ['YZ', 'x']] as const;

    for (const [plane, fixed] of planes) {
      const shape: WaypointShape = { kind: 'circle', radius: 0.04, plane };
      for (let i = 0; i <= 32; i++) {
        const p = pointOnCircle(shape, c, i / 32);
        expect(dist(p, c)).toBeCloseTo(0.04, 9);
        expect(p[fixed]).toBeCloseTo(c[fixed], 9);
      }
    }
  });

  it('closes the loop', () => {
    const shape: WaypointShape = { kind: 'circle', radius: 0.05, plane: 'XY' };
    const c = { x: 0, y: 0, z: 0.3 };
    expect(dist(pointOnCircle(shape, c, 0), pointOnCircle(shape, c, 1))).toBeCloseTo(0, 9);
  });

  it('runs the other way round when asked', () => {
    const c = { x: 0, y: 0, z: 0.3 };
    const ccw = pointOnCircle({ kind: 'circle', radius: 0.05, plane: 'XY' }, c, 0.25);
    const cw = pointOnCircle({ kind: 'circle', radius: 0.05, plane: 'XY', clockwise: true }, c, 0.25);
    expect(cw.y).toBeCloseTo(-ccw.y, 9);
  });
});

describe('a figure is one waypoint', () => {
  const shape: WaypointShape = { kind: 'circle', radius: 0.03, plane: 'XY' };

  it('adds one item, not one per point', () => {
    const wp = makeCircleWaypoint(shape, centre, undefined, 50);
    expect(wp.shape).toEqual(shape);
    // The centre is what the list shows and what the 3D view marks.
    expect(wp.position).toEqual(centre);
    expect(wp.label).toMatch(/30 mm circle/);
  });

  it('is entered at the rim, not at the centre', () => {
    // The centre is where the figure is described from and is not on the path.
    const planner = new TrajectoryPlanner({ interpolationMode: 'joint' });
    const wp = makeCircleWaypoint(shape, centre, undefined, 50);
    const traj = planner.planTrajectory([wp], seed);

    const pts = TrajectoryPlanner.flattenTrajectory(traj);
    const last = ForwardKinematics.position(pts[pts.length - 1].jointAngles);
    expect(dist(last, centre)).toBeCloseTo(shape.radius, 3);
  });

  it('is sampled by the motion, not by the description', () => {
    // The whole reason for holding a shape rather than a point list: the path
    // gets as many points as it needs, not as many as were chosen up front.
    const planner = new TrajectoryPlanner({ interpolationMode: 'joint', pointsPerSecond: 10 });
    const wp = makeCircleWaypoint(shape, centre, undefined, 50);
    const traj = planner.planTrajectory([wp], seed);

    const arc = traj.segments[traj.segments.length - 1];
    expect(arc.points.length).toBeGreaterThan(segmentsForCircle(shape.radius));
  });

  it('follows the true circle all the way round', () => {
    const planner = new TrajectoryPlanner({ interpolationMode: 'joint', pointsPerSecond: 10 });
    const wp = makeCircleWaypoint(shape, centre, undefined, 50);
    const traj = planner.planTrajectory([wp], seed);

    const arc = traj.segments[traj.segments.length - 1];
    let worstRadial = 0;
    let worstPlane = 0;
    for (const p of arc.points) {
      const tcp = ForwardKinematics.position(p.jointAngles);
      worstRadial = Math.max(
        worstRadial,
        Math.abs(Math.hypot(tcp.x - centre.x, tcp.y - centre.y) - shape.radius)
      );
      worstPlane = Math.max(worstPlane, Math.abs(tcp.z - centre.z));
    }
    expect(worstRadial).toBeLessThan(0.0005);
    expect(worstPlane).toBeLessThan(0.0005);
  });
});

describe('reachability is settled before anything is added', () => {
  it('accepts a circle that fits', () => {
    const check = validateCircle(
      { kind: 'circle', radius: 0.03, plane: 'XY' }, centre, undefined, seed
    );
    expect(check.ok).toBe(true);
    expect(check.unreachable).toBe(0);
  });

  it('rejects one that does not, and says how much of it', () => {
    const check = validateCircle(
      { kind: 'circle', radius: 0.5, plane: 'XY' }, centre, undefined, seed
    );
    expect(check.ok).toBe(false);
    expect(check.unreachable).toBeGreaterThan(0);
    expect(check.message).toMatch(/out of reach/);
  });

  it('mentions the lock when holding the tool is what does not fit', () => {
    const check = validateCircle(
      { kind: 'circle', radius: 0.5, plane: 'XY' }, centre, rotation, seed
    );
    expect(check.ok).toBe(false);
    expect(check.message).toMatch(/tool lock/);
  });
});

describe('holding the tool along a path', () => {
  const interp = new PathInterpolator();

  it('holds it round a whole circle', () => {
    const shape: WaypointShape = { kind: 'circle', radius: 0.02, plane: 'XY' };
    const entry = pointOnCircle(shape, centre, 0);
    const start = interp.resolveWaypointAngles(
      { id: 'e', position: entry, orientation: rotation, speed: 50 }, seed
    )!;

    const arc = interp.interpolateArc(start, shape, centre, 50, 100, 10, rotation);
    const worst = Math.max(...arc.points.map(p => tumbleDeg(start, p.jointAngles)));
    expect(worst).toBeLessThan(0.5);
  });

  it('lets it tip when nothing asks for it', () => {
    const shape: WaypointShape = { kind: 'circle', radius: 0.02, plane: 'XY' };
    const entry = pointOnCircle(shape, centre, 0);
    const start = interp.resolveWaypointAngles(
      { id: 'e', position: entry, speed: 50 }, seed
    )!;

    const arc = interp.interpolateArc(start, shape, centre, 50, 100, 10);
    const worst = Math.max(...arc.points.map(p => tumbleDeg(start, p.jointAngles)));
    expect(worst).toBeGreaterThan(1);
  });

  it('holds it across a whole path when the planner is told to', () => {
    // The path-level setting, independent of what any waypoint carries: taught
    // waypoints have joint angles and no orientation at all, so without this
    // there was no way to ask for a held tool along a taught path.
    const taught = [
      { id: 'a', position: { ...centre, x: centre.x + 0.02 }, speed: 50 },
      { id: 'b', position: { ...centre, x: centre.x + 0.02, y: centre.y + 0.02 }, speed: 50 },
      { id: 'c', position: { ...centre, y: centre.y + 0.02 }, speed: 50 }
    ];

    const held = new TrajectoryPlanner({
      interpolationMode: 'linear',
      holdToolOrientation: true,
      pointsPerSecond: 10
    }).planTrajectory(taught, seed);

    const free = new TrajectoryPlanner({
      interpolationMode: 'linear',
      holdToolOrientation: false,
      pointsPerSecond: 10
    }).planTrajectory(taught, seed);

    const worst = (t: typeof held) =>
      Math.max(
        ...TrajectoryPlanner.flattenTrajectory(t).map(p => tumbleDeg(HOME_POSE_DEG, p.jointAngles))
      );

    expect(worst(held)).toBeLessThan(0.5);
    expect(worst(free)).toBeGreaterThan(worst(held));
  });
});

describe('the old point-list helper still describes the same circle', () => {
  it('agrees with the parametric form', () => {
    const spec = { centre, radius: 0.03, plane: 'XY' as const, speed: 50 };
    const pts = circlePoints(spec);
    const shape: WaypointShape = { kind: 'circle', radius: 0.03, plane: 'XY' };

    pts.forEach((p, i) => {
      const q = pointOnCircle(shape, centre, i / (pts.length - 1));
      expect(dist(p, q)).toBeLessThan(1e-12);
    });
  });
});
