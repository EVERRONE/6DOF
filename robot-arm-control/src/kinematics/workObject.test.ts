// Work objects: the frames taught points are measured in.
//
// The property everything here exists for is one sentence: move the fixture,
// re-teach three points, and the whole program follows. The last test in this
// file is that sentence.

import {
  BASE_FRAME,
  WorkObject,
  danglingFrames,
  frameById,
  pointFromBase,
  pointToBase,
  rotationFromBase,
  rotationToBase,
  teachFromThreePoints
} from './workObject';
import { resolveWaypoint } from '../motion/TrajectoryPlanner';
import { Vector3 } from './types';
import { multiply3, rotationLog, rpyToMatrix, transpose3 } from './linalg';

const DEG = Math.PI / 180;

const dist = (a: Vector3, b: Vector3) => Math.hypot(a.x - b.x, a.y - b.y, a.z - b.z);
const angle = (a: { roll: number; pitch: number; yaw: number }, b: typeof a) => {
  const w = rotationLog(multiply3(rpyToMatrix(a), transpose3(rpyToMatrix(b))));
  return (Math.hypot(w.x, w.y, w.z) * 180) / Math.PI;
};

/** A frame 100 mm out in X, 200 in Y, turned 30 degrees about Z. */
const fixture: WorkObject = {
  id: 'jig',
  name: 'Jig',
  origin: { x: 0.1, y: 0.2, z: 0.05 },
  rpy: { roll: 0, pitch: 0, yaw: 30 * DEG }
};

describe('transforming between a frame and the base', () => {
  it('round-trips a point', () => {
    const p = { x: 0.03, y: -0.01, z: 0.02 };
    expect(dist(pointFromBase(fixture, pointToBase(fixture, p)), p)).toBeLessThan(1e-12);
  });

  it('round-trips an attitude', () => {
    const r = { roll: 10 * DEG, pitch: -20 * DEG, yaw: 45 * DEG };
    expect(angle(rotationFromBase(fixture, rotationToBase(fixture, r)), r)).toBeLessThan(1e-9);
  });

  it('places the frame origin where the frame says', () => {
    const atOrigin = pointToBase(fixture, { x: 0, y: 0, z: 0 });
    expect(dist(atOrigin, fixture.origin)).toBeLessThan(1e-12);
  });

  it('turns a direction by the frame rotation, not just shifts it', () => {
    // 100 mm along the frame's X, with the frame turned 30 degrees about Z.
    const p = pointToBase(fixture, { x: 0.1, y: 0, z: 0 });
    expect((p.x - fixture.origin.x) * 1000).toBeCloseTo(100 * Math.cos(30 * DEG), 6);
    expect((p.y - fixture.origin.y) * 1000).toBeCloseTo(100 * Math.sin(30 * DEG), 6);
  });

  it('leaves everything alone in the base frame', () => {
    const p = { x: 0.1, y: -0.2, z: 0.3 };
    expect(dist(pointToBase(BASE_FRAME, p), p)).toBeLessThan(1e-12);
    expect(dist(pointFromBase(BASE_FRAME, p), p)).toBeLessThan(1e-12);
  });
});

describe('teaching a frame from three touched points', () => {
  it('puts the origin on the first point and X through the second', () => {
    const result = teachFromThreePoints(
      { x: 0.1, y: 0.2, z: 0.05 },
      { x: 0.2, y: 0.2, z: 0.05 }, // +X
      { x: 0.1, y: 0.3, z: 0.05 }  // +Y side
    );
    expect(result.ok).toBe(true);
    if (!result.ok) return;

    const frame: WorkObject = { id: 't', name: 't', origin: result.origin, rpy: result.rpy };
    expect(dist(result.origin, { x: 0.1, y: 0.2, z: 0.05 })).toBeLessThan(1e-12);

    // A point 50 mm along the frame's own X has to land on the line towards p2.
    const alongX = pointToBase(frame, { x: 0.05, y: 0, z: 0 });
    expect(dist(alongX, { x: 0.15, y: 0.2, z: 0.05 })).toBeLessThan(1e-9);

    // And 50 mm along its Y, towards p3.
    const alongY = pointToBase(frame, { x: 0, y: 0.05, z: 0 });
    expect(dist(alongY, { x: 0.1, y: 0.25, z: 0.05 })).toBeLessThan(1e-9);
  });

  it('recovers Y from the cross product, so the result is square', () => {
    // p3 deliberately not perpendicular to p1->p2. The frame must still come out
    // orthogonal - the operator touching three points on a real fixture never
    // gets a right angle, and a frame that inherited their error would skew
    // every point taught in it.
    const result = teachFromThreePoints(
      { x: 0, y: 0, z: 0 },
      { x: 0.1, y: 0, z: 0 },
      { x: 0.07, y: 0.04, z: 0 }
    );
    expect(result.ok).toBe(true);
    if (!result.ok) return;

    const R = rpyToMatrix(result.rpy);
    const col = (k: number) => ({ x: R[0][k], y: R[1][k], z: R[2][k] });
    const dot = (a: Vector3, b: Vector3) => a.x * b.x + a.y * b.y + a.z * b.z;

    for (let i = 0; i < 3; i++) {
      expect(Math.hypot(col(i).x, col(i).y, col(i).z)).toBeCloseTo(1, 12);
      for (let j = i + 1; j < 3; j++) {
        expect(Math.abs(dot(col(i), col(j)))).toBeLessThan(1e-12);
      }
    }
  });

  it('refuses two identical points', () => {
    const r = teachFromThreePoints({ x: 0, y: 0, z: 0 }, { x: 0, y: 0, z: 0 }, { x: 1, y: 1, z: 0 });
    expect(r.ok).toBe(false);
    if (!r.ok) expect(r.reason).toMatch(/same/i);
  });

  it('refuses three points on a line', () => {
    const r = teachFromThreePoints(
      { x: 0, y: 0, z: 0 },
      { x: 0.1, y: 0, z: 0 },
      { x: 0.2, y: 0, z: 0 }
    );
    expect(r.ok).toBe(false);
    if (!r.ok) expect(r.reason).toMatch(/line/i);
  });

  it('judges collinearity by shape, not by units', () => {
    // The same three points expressed in millimetres rather than metres must get
    // the same answer. An absolute threshold would call one of them degenerate.
    const metres = teachFromThreePoints(
      { x: 0, y: 0, z: 0 },
      { x: 0.001, y: 0, z: 0 },
      { x: 0, y: 0.001, z: 0 }
    );
    const bigger = teachFromThreePoints(
      { x: 0, y: 0, z: 0 },
      { x: 1, y: 0, z: 0 },
      { x: 0, y: 1, z: 0 }
    );
    expect(metres.ok).toBe(true);
    expect(bigger.ok).toBe(true);
  });
});

describe('resolving a waypoint', () => {
  const base = { id: 'w', speed: 50, position: { x: 0.01, y: 0.02, z: 0.03 } };

  it('leaves a waypoint with no frame alone', () => {
    const resolved = resolveWaypoint(base, [fixture]);
    expect(resolved).toBe(base);
  });

  it('lifts position and orientation into base coordinates', () => {
    const w = {
      ...base,
      frame: 'jig',
      orientation: { roll: 0, pitch: 0, yaw: 10 * DEG }
    };
    const resolved = resolveWaypoint(w, [fixture]);

    expect(dist(resolved.position, pointToBase(fixture, base.position))).toBeLessThan(1e-12);
    // The frame is turned 30 degrees, the point 10 within it, so 40 in base.
    expect(resolved.orientation!.yaw / DEG).toBeCloseTo(40, 9);
  });

  it('falls back to base for a frame that no longer exists', () => {
    const w = { ...base, frame: 'deleted' };
    const resolved = resolveWaypoint(w, [fixture]);
    expect(dist(resolved.position, base.position)).toBeLessThan(1e-12);
    // ...and is reported, because that is somewhere real and wrong.
    expect(danglingFrames([fixture], ['jig', 'deleted', undefined])).toEqual(['deleted']);
  });

  it('resolves the base frame by name', () => {
    expect(frameById([fixture], BASE_FRAME.id)).toBe(BASE_FRAME);
    expect(frameById([fixture], null)).toBe(BASE_FRAME);
    expect(frameById([fixture], 'jig')).toBe(fixture);
  });
});

// ---------------------------------------------------------------------------

describe('the whole point', () => {
  it('moves every taught point when the fixture moves', () => {
    // Teach three points on a jig, then teach a working point relative to it.
    const taught = teachFromThreePoints(
      { x: 0.10, y: 0.20, z: 0.05 },
      { x: 0.20, y: 0.20, z: 0.05 },
      { x: 0.10, y: 0.30, z: 0.05 }
    );
    expect(taught.ok).toBe(true);
    if (!taught.ok) return;

    const jig: WorkObject = { id: 'jig', name: 'Jig', origin: taught.origin, rpy: taught.rpy };

    // A hole 30 mm along the jig's X and 40 along its Y.
    const hole = { x: 0.03, y: 0.04, z: 0 };
    const waypoint = { id: 'hole', speed: 50, position: hole, frame: 'jig' };

    const before = resolveWaypoint(waypoint, [jig]).position;
    expect(dist(before, { x: 0.13, y: 0.24, z: 0.05 })).toBeLessThan(1e-9);

    // Now somebody shifts the jig 50 mm in X and turns it 90 degrees. Re-teach
    // the same three features on it, in their new places.
    const retaught = teachFromThreePoints(
      { x: 0.15, y: 0.20, z: 0.05 },
      { x: 0.15, y: 0.30, z: 0.05 }, // the old +X feature, now pointing +Y
      { x: 0.05, y: 0.20, z: 0.05 }  // the old +Y feature, now pointing -X
    );
    expect(retaught.ok).toBe(true);
    if (!retaught.ok) return;

    const moved: WorkObject = { id: 'jig', name: 'Jig', origin: retaught.origin, rpy: retaught.rpy };

    // The waypoint was never edited. It now points at the hole in its new place:
    // 30 mm along the jig's X, which is now +Y, and 40 along its Y, now -X.
    const after = resolveWaypoint(waypoint, [moved]).position;
    expect(dist(after, { x: 0.11, y: 0.23, z: 0.05 })).toBeLessThan(1e-9);

    // And it really did move - this is not a test that passes on a no-op.
    expect(dist(before, after) * 1000).toBeGreaterThan(20);
  });
});
