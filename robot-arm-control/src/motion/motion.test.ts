// Tests for the motion layer.
//
// The interesting property is geometric, not temporal: the firmware owns the
// velocity profile now, so what this layer has to get right is that its samples
// lie on the intended path. That is exactly what was broken - joint-space
// interpolation produced a bowed path rather than a straight one - and there was
// no test here to catch it.

import { PathInterpolator } from './PathInterpolator';
import { TrajectoryPlanner, DEFAULT_PLANNER_CONFIG } from './TrajectoryPlanner';
import { VelocityProfile } from './VelocityProfile';
import { Waypoint } from './types';
import { ForwardKinematics } from '../kinematics/ForwardKinematics';
import {
  HOME_POSE_DEG,
  JOINT_LIMITS_DEG,
  JOINT_MAX_ACCEL_DEG_S2,
  JOINT_MAX_SPEED_DEG_S,
  NUM_JOINTS,
  isWithinLimitsDeg
} from '../kinematics/robotModel';
import { Vector3 } from '../kinematics/types';

/** Fraction of its travel each moving joint has covered at a sample. */
function progressFractions(
  sample: number[],
  start: number[],
  distances: number[]
): number[] {
  const out: number[] = [];
  for (let i = 0; i < NUM_JOINTS; i++) {
    if (Math.abs(distances[i]) < 1e-9) continue;
    out.push((sample[i] - start[i]) / distances[i]);
  }
  return out;
}

function dist(a: Vector3, b: Vector3): number {
  return Math.hypot(a.x - b.x, a.y - b.y, a.z - b.z);
}

/** Perpendicular distance from `p` to the segment through `a` and `b`. */
function distanceToLine(p: Vector3, a: Vector3, b: Vector3): number {
  const ab = { x: b.x - a.x, y: b.y - a.y, z: b.z - a.z };
  const ap = { x: p.x - a.x, y: p.y - a.y, z: p.z - a.z };
  const len2 = ab.x ** 2 + ab.y ** 2 + ab.z ** 2;
  if (len2 < 1e-18) return dist(p, a);

  const t = (ap.x * ab.x + ap.y * ab.y + ap.z * ab.z) / len2;
  const clamped = Math.max(0, Math.min(1, t));
  const closest = {
    x: a.x + ab.x * clamped,
    y: a.y + ab.y * clamped,
    z: a.z + ab.z * clamped
  };
  return dist(p, closest);
}

// ---------------------------------------------------------------------------
// Model mirroring
// ---------------------------------------------------------------------------

describe('motion limits mirror the firmware', () => {
  it('matches MAX_JOINT_SPEED and MAX_JOINT_ACCEL from firmware/config.h', () => {
    // Planning against different numbers than the firmware enforces makes every
    // duration estimate and path preview a fiction.
    // Bring-up values. Must match MAX_JOINT_SPEED / MAX_JOINT_ACCEL in
    // firmware/config.h exactly - if the solver plans in a faster box than the
    // firmware will execute, the firmware silently scales the move down and the
    // arm arrives late relative to everything the app thinks it timed.
    expect(JOINT_MAX_SPEED_DEG_S).toEqual([15, 10, 15, 20, 30, 45]);
    expect(JOINT_MAX_ACCEL_DEG_S2).toEqual([40, 25, 40, 60, 75, 100]);
  });
});

// ---------------------------------------------------------------------------
// Joint-space interpolation
// ---------------------------------------------------------------------------

describe('joint-space interpolation', () => {
  const interp = new PathInterpolator();

  it('keeps every joint in lockstep, so the path is straight in joint space', () => {
    // Regression: a 40 degree J2 move against a 5 degree J3 move used to leave
    // J3 11% ahead of J2 part way through, because each joint got its own
    // trapezoid. The samples were then not collinear, which also cost speed at
    // every one of them.
    const start = [...HOME_POSE_DEG];
    const end = [0, 45, 60, 129, 131, 0];
    const distances = end.map((v, i) => v - start[i]);

    const segment = interp.interpolateJointSpace(start, end, 60, 120, 10);
    expect(segment.points.length).toBeGreaterThan(3);

    let worstSpread = 0;
    for (const point of segment.points) {
      const fractions = progressFractions(point.jointAngles, start, distances);
      worstSpread = Math.max(worstSpread, Math.max(...fractions) - Math.min(...fractions));
    }

    expect(worstSpread).toBeLessThan(1e-9);
  });

  it('holds lockstep across very lopsided travel ratios', () => {
    const start = [...HOME_POSE_DEG];
    const end = [-30, 6, 55, 250, 160, 300];
    const distances = end.map((v, i) => v - start[i]);

    const segment = interp.interpolateJointSpace(start, end, 60, 120, 20);

    let worstSpread = 0;
    for (const point of segment.points) {
      const fractions = progressFractions(point.jointAngles, start, distances);
      worstSpread = Math.max(worstSpread, Math.max(...fractions) - Math.min(...fractions));
    }
    expect(worstSpread).toBeLessThan(1e-9);
  });

  it('progresses monotonically from start to end', () => {
    const start = [...HOME_POSE_DEG];
    const end = [0, 45, 60, 129, 131, 0];
    const segment = interp.interpolateJointSpace(start, end, 60, 120, 10);

    const first = segment.points[0];
    const last = segment.points[segment.points.length - 1];

    first.jointAngles.forEach((v, i) => expect(v).toBeCloseTo(start[i], 9));
    last.jointAngles.forEach((v, i) => expect(v).toBeCloseTo(end[i], 9));

    let previous = -Infinity;
    for (const point of segment.points) {
      const fraction = (point.jointAngles[1] - start[1]) / (end[1] - start[1]);
      expect(fraction).toBeGreaterThanOrEqual(previous - 1e-12);
      previous = fraction;
    }
    expect(previous).toBeCloseTo(1, 9);
  });

  it('respects the per-joint speed and acceleration limits', () => {
    const start = [...HOME_POSE_DEG];
    // Ask for far more than any joint can deliver.
    const end = JOINT_LIMITS_DEG.max.map(v => v * 10);

    const segment = interp.interpolateJointSpace(start, end, 10000, 10000, 200);

    for (const point of segment.points) {
      (point.velocity ?? []).forEach((v, i) => {
        expect(Math.abs(v)).toBeLessThanOrEqual(JOINT_MAX_SPEED_DEG_S[i] * 1.001);
      });
    }

    // Acceleration, from the sampled velocities.
    for (let k = 1; k < segment.points.length; k++) {
      const dt = segment.points[k].time - segment.points[k - 1].time;
      if (dt <= 1e-9) continue;

      const a = segment.points[k].velocity ?? [];
      const b = segment.points[k - 1].velocity ?? [];
      for (let i = 0; i < NUM_JOINTS; i++) {
        const accel = Math.abs(((a[i] ?? 0) - (b[i] ?? 0)) / dt);
        expect(accel).toBeLessThanOrEqual(JOINT_MAX_ACCEL_DEG_S2[i] * 1.05);
      }
    }
  });

  it('honours a requested speed below the hardware limit', () => {
    const start = [...HOME_POSE_DEG];
    const end = [0, 45, 55, 129, 131, 0];

    // Both speeds have to sit below the axis limit, or the planner clamps them
    // to the same value and the comparison proves nothing. J2 is the only joint
    // that moves here, so scale off its limit instead of hardcoding a figure -
    // hardcoded speeds silently stopped testing anything when the bring-up
    // values were lowered.
    const j2Limit = JOINT_MAX_SPEED_DEG_S[1];
    const fast = interp.interpolateJointSpace(start, end, j2Limit * 0.8, 120, 10);
    const slow = interp.interpolateJointSpace(start, end, j2Limit * 0.2, 120, 10);

    expect(slow.duration).toBeGreaterThan(fast.duration * 2);
  });

  it('never leaves the joint limits', () => {
    const segment = interp.interpolateJointSpace(
      [...HOME_POSE_DEG],
      // The extremes themselves, read from the limits rather than copied.
      [...JOINT_LIMITS_DEG.max.slice(0, 5), 0],
      60,
      120,
      20
    );
    for (const point of segment.points) {
      expect(isWithinLimitsDeg(point.jointAngles, 1e-6)).toBe(true);
    }
  });

  it('returns a single point when there is nothing to do', () => {
    const segment = interp.interpolateJointSpace(
      [...HOME_POSE_DEG],
      [...HOME_POSE_DEG],
      60,
      120,
      10
    );
    expect(segment.points).toHaveLength(1);
    expect(segment.duration).toBe(0);
  });
});

// ---------------------------------------------------------------------------
// Cartesian interpolation
// ---------------------------------------------------------------------------

describe('linear (Cartesian) interpolation', () => {
  const interp = new PathInterpolator();
  const start = [...HOME_POSE_DEG];
  const startPos = ForwardKinematics.position(start);

  it('keeps the tool on the straight line between start and target', () => {
    // A nearby, reachable target: the pose with J2 five degrees further on.
    const target = ForwardKinematics.position([0, 10, 55, 129, 131, 0]);

    const segment = interp.interpolateCartesianSpace(start, target, 50, 100, 10);
    expect(segment.unreachableSamples ?? 0).toBe(0);

    let worstDeviation = 0;
    for (const point of segment.points) {
      const achieved = ForwardKinematics.position(point.jointAngles);
      worstDeviation = Math.max(worstDeviation, distanceToLine(achieved, startPos, target));
    }

    // Bounded by the IK position tolerance, not by the sampling.
    expect(worstDeviation).toBeLessThan(0.001);
  });

  it('bounds the chord between consecutive samples', () => {
    const target = ForwardKinematics.position([0, 10, 55, 129, 131, 0]);
    const segment = interp.interpolateCartesianSpace(start, target, 50, 100, 1);

    let worstChord = 0;
    for (let k = 1; k < segment.points.length; k++) {
      const a = ForwardKinematics.position(segment.points[k - 1].jointAngles);
      const b = ForwardKinematics.position(segment.points[k].jointAngles);
      worstChord = Math.max(worstChord, dist(a, b));
    }

    // 2 mm chord limit, plus twice the IK tolerance.
    expect(worstChord).toBeLessThan(0.002 + 0.001);
  });

  it('samples by arc length, not by time', () => {
    // Uniform-in-time sampling bunches points at the ends and spreads them at
    // cruise, which is where the deviation from the line would be worst.
    const target = ForwardKinematics.position([0, 12, 55, 129, 131, 0]);
    const segment = interp.interpolateCartesianSpace(start, target, 50, 100, 10);

    const chords: number[] = [];
    for (let k = 1; k < segment.points.length; k++) {
      const a = ForwardKinematics.position(segment.points[k - 1].jointAngles);
      const b = ForwardKinematics.position(segment.points[k].jointAngles);
      chords.push(dist(a, b));
    }

    const longest = Math.max(...chords);
    const shortest = Math.min(...chords);
    // Even spacing: the ratio would be large if sampling followed the velocity.
    expect(longest / Math.max(shortest, 1e-9)).toBeLessThan(4);
  });

  it('reports how many samples were unreachable instead of silently cutting the corner', () => {
    const unreachable: Vector3 = { x: 2, y: 2, z: 2 };
    const segment = interp.interpolateCartesianSpace(start, unreachable, 50, 100, 10);

    expect(segment.unreachableSamples ?? 0).toBeGreaterThan(0);
    for (const point of segment.points) {
      expect(isWithinLimitsDeg(point.jointAngles, 1e-6)).toBe(true);
    }
  });

  it('returns a single point when already at the target', () => {
    const segment = interp.interpolateCartesianSpace(start, startPos, 50, 100, 10);
    expect(segment.points).toHaveLength(1);
  });
});

// ---------------------------------------------------------------------------
// Velocity profile
// ---------------------------------------------------------------------------

describe('velocity profile', () => {
  it('reaches the requested peak on a long move', () => {
    const profile = new VelocityProfile(100, 10, 5);
    expect(profile.getProfileType()).toBe('trapezoidal');
    expect(profile.getPeakVelocity()).toBeCloseTo(10, 9);
    expect(profile.getPosition(profile.getDuration())).toBeCloseTo(100, 9);
  });

  it('falls back to a triangle when the move is too short to reach cruise', () => {
    const profile = new VelocityProfile(1, 100, 5);
    expect(profile.getProfileType()).toBe('triangular');
    expect(profile.getPeakVelocity()).toBeLessThan(100);
    expect(profile.getPosition(profile.getDuration())).toBeCloseTo(1, 9);
  });

  it('is monotone and starts and ends at rest', () => {
    const profile = new VelocityProfile(50, 10, 20);
    const duration = profile.getDuration();

    expect(profile.getVelocity(0)).toBeCloseTo(0, 9);
    expect(profile.getVelocity(duration)).toBeCloseTo(0, 9);

    let previous = -Infinity;
    for (let k = 0; k <= 200; k++) {
      const t = (duration * k) / 200;
      const p = profile.getPosition(t);
      expect(p).toBeGreaterThanOrEqual(previous - 1e-12);
      previous = p;
    }
  });

  it('handles a negative distance by mirroring', () => {
    const profile = new VelocityProfile(-40, 10, 20);
    expect(profile.getPosition(profile.getDuration())).toBeCloseTo(-40, 9);
    expect(profile.getVelocity(profile.getDuration() / 2)).toBeLessThan(0);
  });

  it('clamps queries outside its duration', () => {
    const profile = new VelocityProfile(10, 5, 10);
    expect(profile.getPosition(-1)).toBe(0);
    expect(profile.getPosition(profile.getDuration() + 5)).toBeCloseTo(10, 9);
    expect(profile.getVelocity(-1)).toBe(0);
    expect(profile.getVelocity(profile.getDuration() + 5)).toBe(0);
  });
});

// ---------------------------------------------------------------------------
// Trajectory planner
// ---------------------------------------------------------------------------

describe('trajectory planner', () => {
  const makeWaypoint = (angles: number[], label: string): Waypoint => ({
    id: label,
    position: ForwardKinematics.position(angles),
    jointAngles: [...angles],
    speed: DEFAULT_PLANNER_CONFIG.defaultSpeed,
    label
  });

  it('chains segments through every waypoint', () => {
    const planner = new TrajectoryPlanner({ interpolationMode: 'joint' });
    const waypoints = [
      makeWaypoint([0, 15, 55, 129, 131, 0], 'a'),
      makeWaypoint([0, 30, 60, 129, 131, 0], 'b'),
      makeWaypoint([-10, 30, 60, 140, 131, 0], 'c')
    ];

    const trajectory = planner.planTrajectory(waypoints, [...HOME_POSE_DEG]);

    expect(trajectory.segments).toHaveLength(3);
    expect(trajectory.totalDuration).toBeGreaterThan(0);
    expect(trajectory.pointCount).toBeGreaterThan(3);

    // The last point of each segment is the waypoint it targets.
    trajectory.segments.forEach((segment, index) => {
      const last = segment.points[segment.points.length - 1];
      last.jointAngles.forEach((v, joint) => {
        expect(v).toBeCloseTo(waypoints[index].jointAngles![joint], 6);
      });
    });
  });

  it('produces a monotone timeline when flattened', () => {
    const planner = new TrajectoryPlanner({ interpolationMode: 'joint' });
    const waypoints = [
      makeWaypoint([0, 15, 55, 129, 131, 0], 'a'),
      makeWaypoint([0, 30, 60, 129, 131, 0], 'b')
    ];

    const trajectory = planner.planTrajectory(waypoints, [...HOME_POSE_DEG]);
    const points = TrajectoryPlanner.flattenTrajectory(trajectory);

    expect(points.length).toBeGreaterThan(2);
    for (let k = 1; k < points.length; k++) {
      expect(points[k].time).toBeGreaterThan(points[k - 1].time);
    }
  });

  it('keeps every planned point inside the joint limits', () => {
    const planner = new TrajectoryPlanner({ interpolationMode: 'joint' });
    const waypoints = [
      makeWaypoint([...JOINT_LIMITS_DEG.max.slice(0, 5), 0], 'extreme'),
      makeWaypoint([30, 0, 0, 0, 0, 0], 'other end')
    ];

    const trajectory = planner.planTrajectory(waypoints, [...HOME_POSE_DEG]);
    for (const point of TrajectoryPlanner.flattenTrajectory(trajectory)) {
      expect(isWithinLimitsDeg(point.jointAngles, 1e-6)).toBe(true);
    }
  });

  it('gives a Cartesian preview that follows the tool path', () => {
    const planner = new TrajectoryPlanner({ interpolationMode: 'joint' });
    const waypoints = [makeWaypoint([0, 20, 55, 129, 131, 0], 'a')];

    const trajectory = planner.planTrajectory(waypoints, [...HOME_POSE_DEG]);
    const positions = TrajectoryPlanner.getTrajectoryPositions(trajectory);

    expect(positions.length).toBeGreaterThan(1);
    const expected = ForwardKinematics.position([0, 20, 55, 129, 131, 0]);
    expect(dist(positions[positions.length - 1], expected)).toBeLessThan(1e-9);
  });

  it('skips waypoints it cannot resolve, and says which', () => {
    const planner = new TrajectoryPlanner({ interpolationMode: 'joint' });
    const reachable = makeWaypoint([0, 20, 55, 129, 131, 0], 'ok');
    const unreachable: Waypoint = {
      id: 'bad',
      position: { x: 5, y: 5, z: 5 },
      speed: 30,
      label: 'bad'
    };

    const trajectory = planner.planTrajectory([reachable, unreachable], [...HOME_POSE_DEG]);
    expect(trajectory.segments).toHaveLength(1);
    expect(trajectory.skippedWaypoints).toEqual(['bad']);
  });

  it('keeps later segments aligned with the right waypoint after a skip', () => {
    // Regression: the segment loop read waypoints[i] while the joint angles came
    // from the filtered list, so one skipped waypoint shifted every later segment
    // onto the wrong waypoint's speed - and, in linear mode, onto the wrong
    // target position entirely.
    const planner = new TrajectoryPlanner({ interpolationMode: 'joint' });

    const first = makeWaypoint([0, 15, 55, 129, 131, 0], 'first');
    first.speed = 60;

    const broken: Waypoint = {
      id: 'broken',
      position: { x: 5, y: 5, z: 5 },
      speed: 60,
      label: 'broken'
    };

    const last = makeWaypoint([0, 35, 55, 129, 131, 0], 'last');
    last.speed = 4; // deliberately slow, so a mix-up shows up in the duration

    const trajectory = planner.planTrajectory([first, broken, last], [...HOME_POSE_DEG]);

    expect(trajectory.segments).toHaveLength(2);
    expect(trajectory.skippedWaypoints).toEqual(['broken']);

    // 20 degrees of J2 at 4 deg/s is about 5 s; at the 60 deg/s the buggy version
    // would have picked up it would be under a second.
    expect(trajectory.segments[1].duration).toBeGreaterThan(4);
  });

  it('reports unreachable samples along a linear segment', () => {
    const planner = new TrajectoryPlanner({ interpolationMode: 'linear' });
    const bad: Waypoint = {
      id: 'far',
      position: { x: 3, y: 3, z: 3 },
      speed: 50,
      label: 'far'
    };

    const trajectory = planner.planTrajectory([bad], [...HOME_POSE_DEG]);
    // The waypoint has no jointAngles, so it is dropped at resolve time.
    expect(trajectory.skippedWaypoints).toEqual(['far']);
    expect(trajectory.segments).toHaveLength(0);
  });

  it('round-trips a saved path', () => {
    const waypoints = [makeWaypoint([0, 20, 55, 129, 131, 0], 'a')];
    const saved = TrajectoryPlanner.exportPath('demo', waypoints, DEFAULT_PLANNER_CONFIG);

    expect(TrajectoryPlanner.validateSavedPath(saved)).toBe(true);
    expect(TrajectoryPlanner.validateSavedPath({ name: 'x' })).toBe(false);
    expect(TrajectoryPlanner.validateSavedPath(null)).toBe(false);
    expect(
      TrajectoryPlanner.validateSavedPath({ name: 'x', waypoints: [], version: '1.0' })
    ).toBe(false);
  });

  it('returns an empty trajectory for no waypoints', () => {
    const planner = new TrajectoryPlanner();
    const trajectory = planner.planTrajectory([], [...HOME_POSE_DEG]);
    expect(trajectory.segments).toHaveLength(0);
    expect(trajectory.totalDuration).toBe(0);
  });
});
