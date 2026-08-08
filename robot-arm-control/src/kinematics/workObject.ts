// Work objects: named frames that taught points are expressed in.
//
// Everything the program holds is otherwise in the robot's own base frame, which
// makes every taught point a statement about where a thing was *that day*. Nudge
// the fixture five millimetres and the whole program is wrong, with no way to
// say so and nothing to do but teach it again.
//
// A work object is the fix, and it is what every industrial controller has:
// ABB calls it a wobj, KUKA a BASE, Fanuc a UFRAME. Points are stored relative to
// the frame; the frame is measured against the robot. Move the fixture, re-teach
// the three points that define its frame, and every point in the program follows
// it - because the points never described the robot's world in the first place.

import { Rotation3, Vector3 } from './types';
import { matrixToRpy, multiply3, rpyToMatrix, transpose3 } from './linalg';

/** A named frame, expressed in the robot's base frame. */
export interface WorkObject {
  id: string;
  name: string;
  /** Origin of the frame, in base coordinates, metres. */
  origin: Vector3;
  /** Orientation of the frame relative to base, fixed-axis rpy in radians. */
  rpy: Rotation3;
}

/**
 * The base frame itself, for when no work object is selected.
 *
 * Represented rather than special-cased so that every transform below has
 * something to work on, and "no work object" is not a separate code path that
 * can drift from the real one.
 */
export const BASE_FRAME: WorkObject = {
  id: 'base',
  name: 'Robot base',
  origin: { x: 0, y: 0, z: 0 },
  rpy: { roll: 0, pitch: 0, yaw: 0 }
};

const sub = (a: Vector3, b: Vector3): Vector3 => ({ x: a.x - b.x, y: a.y - b.y, z: a.z - b.z });
const cross = (a: Vector3, b: Vector3): Vector3 => ({
  x: a.y * b.z - a.z * b.y,
  y: a.z * b.x - a.x * b.z,
  z: a.x * b.y - a.y * b.x
});
const len = (a: Vector3): number => Math.hypot(a.x, a.y, a.z);
const scale = (a: Vector3, k: number): Vector3 => ({ x: a.x * k, y: a.y * k, z: a.z * k });

function unit(a: Vector3): Vector3 {
  const n = len(a);
  return n < 1e-12 ? { x: 0, y: 0, z: 0 } : scale(a, 1 / n);
}

/** Apply a 3x3 to a vector. */
function apply(R: number[][], v: Vector3): Vector3 {
  return {
    x: R[0][0] * v.x + R[0][1] * v.y + R[0][2] * v.z,
    y: R[1][0] * v.x + R[1][1] * v.y + R[1][2] * v.z,
    z: R[2][0] * v.x + R[2][1] * v.y + R[2][2] * v.z
  };
}

/** A point given in the work object's frame, expressed in base coordinates. */
export function pointToBase(frame: WorkObject, p: Vector3): Vector3 {
  const rotated = apply(rpyToMatrix(frame.rpy), p);
  return {
    x: rotated.x + frame.origin.x,
    y: rotated.y + frame.origin.y,
    z: rotated.z + frame.origin.z
  };
}

/** A point given in base coordinates, expressed in the work object's frame. */
export function pointFromBase(frame: WorkObject, p: Vector3): Vector3 {
  return apply(transpose3(rpyToMatrix(frame.rpy)), sub(p, frame.origin));
}

/**
 * An attitude given in the work object's frame, expressed in base coordinates.
 *
 * Orientations are relative to the frame too, and have to be: turning a fixture
 * ninety degrees should turn the attitude the tool approaches it at by ninety
 * degrees. Storing the attitude in base and only transforming the position would
 * leave a re-taught program reaching the right places the wrong way round.
 */
export function rotationToBase(frame: WorkObject, rpy: Rotation3): Rotation3 {
  return matrixToRpy(multiply3(rpyToMatrix(frame.rpy), rpyToMatrix(rpy)));
}

/** An attitude given in base coordinates, expressed in the work object's frame. */
export function rotationFromBase(frame: WorkObject, rpy: Rotation3): Rotation3 {
  return matrixToRpy(multiply3(transpose3(rpyToMatrix(frame.rpy)), rpyToMatrix(rpy)));
}

/** Why a three-point teach could not produce a frame. */
export type TeachFailure =
  | 'The first two points are the same — they have to be apart to give an X direction'
  | 'The third point is on the line through the first two — it has to be off it to give a plane';

export type TeachResult =
  | { ok: true; origin: Vector3; rpy: Rotation3 }
  | { ok: false; reason: TeachFailure };

/**
 * Build a frame from three touched points.
 *
 * The standard method, and the reason it is three points rather than six numbers
 * is that touching a fixture with the tool is something an operator can do
 * accurately, while measuring its rotation against the robot's base with a rule
 * is not.
 *
 *   p1  the origin
 *   p2  anywhere on the +X axis
 *   p3  anywhere in the +Y half of the XY plane
 *
 * Z comes from the cross product, so the frame is right-handed by construction
 * and p3 only has to be roughly placed - its distance from the line does not
 * matter, only which side of it it is on. Y is then recovered from Z and X
 * rather than taken from p3, which is what makes the result orthogonal even
 * though the three touched points never are.
 *
 * !! The tool frame has to be set before this is any good. These points come
 * !! from where the arm says the tool tip is, and on a bare flange that is the
 * !! flange face - so a frame taught with an unmeasured tool is offset by
 * !! however far the real tip sticks out.
 */
export function teachFromThreePoints(p1: Vector3, p2: Vector3, p3: Vector3): TeachResult {
  const alongX = sub(p2, p1);
  if (len(alongX) < 1e-6) {
    return {
      ok: false,
      reason: 'The first two points are the same — they have to be apart to give an X direction'
    };
  }

  const xAxis = unit(alongX);
  const inPlane = sub(p3, p1);
  const normal = cross(xAxis, inPlane);

  // Collinear points leave no plane to define Y in. Judged against the distance
  // from p1, so the test means "how far off the line", not "how big are the
  // numbers" - the same three points in millimetres and metres must agree.
  if (len(normal) < 1e-6 * Math.max(len(inPlane), 1e-6)) {
    return {
      ok: false,
      reason:
        'The third point is on the line through the first two — it has to be off it to give a plane'
    };
  }

  const zAxis = unit(normal);
  const yAxis = cross(zAxis, xAxis);

  // Columns are the frame's axes expressed in base coordinates.
  const R = [
    [xAxis.x, yAxis.x, zAxis.x],
    [xAxis.y, yAxis.y, zAxis.y],
    [xAxis.z, yAxis.z, zAxis.z]
  ];

  return { ok: true, origin: { ...p1 }, rpy: matrixToRpy(R) };
}

/**
 * Look a frame up by id, falling back to the base frame.
 *
 * A missing id resolves to base rather than throwing. A waypoint naming a work
 * object that has been deleted is a real state to be in - loading an old path,
 * or deleting a frame that is still referenced - and running it against the base
 * frame is wrong, but it is visibly wrong. Refusing to resolve it at all would
 * take the path out of the operator's hands entirely; `danglingFrames` below is
 * how the UI says so.
 */
export function frameById(objects: WorkObject[], id: string | null | undefined): WorkObject {
  if (!id || id === BASE_FRAME.id) return BASE_FRAME;
  return objects.find(o => o.id === id) ?? BASE_FRAME;
}

/** Frame ids referenced by something that no longer exists. */
export function danglingFrames(
  objects: WorkObject[],
  referenced: Array<string | undefined>
): string[] {
  const known = new Set(objects.map(o => o.id));
  known.add(BASE_FRAME.id);
  return Array.from(
    new Set(referenced.filter((id): id is string => !!id && !known.has(id)))
  );
}
