import { buildCircle, circlePoints, segmentsForCircle } from './Shapes';
import { PathInterpolator } from './PathInterpolator';
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

describe('circle geometry', () => {
  it('samples densely enough to stay round, whatever the radius', () => {
    // The count follows from a chord tolerance rather than being a magic
    // number, so a big circle gets more points and a small one does not waste
    // them. What must hold is the resulting roundness.
    for (const radius of [0.01, 0.03, 0.1, 0.25]) {
      const n = segmentsForCircle(radius);
      const sagitta = radius * (1 - Math.cos(Math.PI / n));
      expect(sagitta).toBeLessThanOrEqual(0.0005 + 1e-12);
    }
  });

  it('puts every point on the circle, in the right plane', () => {
    const centre = { x: 0.1, y: -0.05, z: 0.3 };
    const radius = 0.04;

    const planes = [
      ['XY', 'z'],
      ['XZ', 'y'],
      ['YZ', 'x']
    ] as const;

    for (const [plane, fixedAxis] of planes) {
      const pts = circlePoints({ centre, radius, plane, speed: 50 });

      for (const p of pts) {
        expect(dist(p, centre)).toBeCloseTo(radius, 9);
        // The axis normal to the plane must not move.
        expect(p[fixedAxis]).toBeCloseTo(centre[fixedAxis], 9);
      }
    }
  });

  it('closes the loop', () => {
    const pts = circlePoints({
      centre: { x: 0, y: 0, z: 0.3 },
      radius: 0.05,
      plane: 'XY',
      speed: 50
    });
    expect(dist(pts[0], pts[pts.length - 1])).toBeCloseTo(0, 9);
  });

  it('runs the other way round when asked', () => {
    const spec = { centre: { x: 0, y: 0, z: 0.3 }, radius: 0.05, plane: 'XY' as const, speed: 50 };
    const ccw = circlePoints(spec);
    const cw = circlePoints({ ...spec, clockwise: true });

    expect(cw[0]).toEqual(ccw[0]); // same start
    expect(cw[1].y).toBeCloseTo(-ccw[1].y, 9); // opposite first step
  });
});

describe('circles on the real arm', () => {
  const seed = [...HOME_POSE_DEG];
  const home = ForwardKinematics.solve(HOME_POSE_DEG);
  const centre = home.endEffectorPose.position;

  it('builds a reachable circle around the parked pose', () => {
    const result = buildCircle(
      { centre, radius: 0.03, plane: 'XY', speed: 50 },
      seed
    );

    expect(result.ok).toBe(true);
    expect(result.waypoints.length).toBeGreaterThan(8);
    result.waypoints.forEach(w => expect(dist(w.position, centre)).toBeCloseTo(0.03, 9));
  });

  it('refuses one that leaves the workspace, and says how much of it does', () => {
    const result = buildCircle(
      { centre, radius: 0.5, plane: 'XY', speed: 50 },
      seed
    );

    expect(result.ok).toBe(false);
    // Nothing is handed back: a partly reachable figure is not a figure.
    expect(result.waypoints).toHaveLength(0);
    expect(result.unreachable.length).toBeGreaterThan(0);
    expect(result.message).toMatch(/out of reach/);
  });

  it('carries the tool orientation onto every waypoint', () => {
    const rotation = home.endEffectorPose.rotation;
    const result = buildCircle(
      { centre, radius: 0.02, plane: 'XY', speed: 50, orientation: rotation },
      seed
    );

    expect(result.ok).toBe(true);
    result.waypoints.forEach(w => expect(w.orientation).toEqual(rotation));
  });

  it('mentions the lock when a held circle will not fit', () => {
    const rotation = home.endEffectorPose.rotation;
    const result = buildCircle(
      { centre, radius: 0.5, plane: 'XY', speed: 50, orientation: rotation },
      seed
    );

    expect(result.ok).toBe(false);
    expect(result.message).toMatch(/tool lock/);
  });
});

describe('paths hold the tool when the waypoint asks for it', () => {
  const interp = new PathInterpolator();
  const home = ForwardKinematics.solve(HOME_POSE_DEG);

  it('keeps the tool still along a straight line', () => {
    const p = home.endEffectorPose.position;
    const target = { ...p, z: p.z + 0.02 };

    const held = interp.interpolateCartesianSpace(
      [...HOME_POSE_DEG], target, 50, 100, 10, home.endEffectorPose.rotation
    );
    const free = interp.interpolateCartesianSpace(
      [...HOME_POSE_DEG], target, 50, 100, 10
    );

    const worst = (seg: typeof held) =>
      Math.max(...seg.points.map(pt => tumbleDeg(HOME_POSE_DEG, pt.jointAngles)));

    expect(worst(held)).toBeLessThan(0.5);
    expect(worst(free)).toBeGreaterThan(3);
  });

  it('resolves a waypoint against its orientation when it carries one', () => {
    const p = home.endEffectorPose.position;
    const rotation = home.endEffectorPose.rotation;

    const q = interp.resolveWaypointAngles(
      { id: 'a', position: { ...p, z: p.z + 0.02 }, orientation: rotation, speed: 50 },
      [...HOME_POSE_DEG]
    );

    expect(q).not.toBeNull();
    expect(tumbleDeg(HOME_POSE_DEG, q!)).toBeLessThan(0.5);
  });
});
