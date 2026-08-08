// What is bolted to the flange, as far as the collision checker is concerned.
//
// The link geometry in collisionModel.ts is generated from the STL meshes, and
// its last entry is a 4 mm stub sized for a bare flange. Everything the checker
// says is therefore about an arm that ends at the flange face. Fit a gripper, a
// pen, a stage - anything - and the checker keeps clearing poses that drive it
// into the shoulder, because nothing in the model knows it is there.
//
// A tool frame does NOT fix this. It moves where the TCP is, so FK, IK and the
// viewer follow it, but the boxes are geometry and know nothing about it. A tool
// can be measured, fitted and driven around while the checker still believes the
// arm stops at the flange - which is exactly the state this arm was in.
//
// Held as one box rather than a mesh. The tool is whatever the operator bolted
// on this morning; there is no model of it to load, and a bounding box is the
// most anybody can be asked to type. It over-reports rather than under-reports,
// which is the right direction for a guard.

import { Vector3 } from './types';

export interface ToolShape {
  /** Full size along the flange frame's own axes, metres. */
  size: Vector3;
  /**
   * Centre of the box in flange coordinates, metres.
   *
   * The flange face is the origin and +Z points away from the arm, so a tool
   * sitting flush on the flange has `centre.z = size.z / 2`. Held separately
   * from the size because a tool is not always centred on the bolt circle - an
   * L-bracket hangs to one side, and a box centred on the flange would claim
   * space that is empty and miss the space that is not.
   */
  centre: Vector3;
}

/**
 * Bumped on every change, so anything that caches a result derived from the
 * geometry can tell that it moved. Mirrors the tool frame's revision counter for
 * the same reason.
 */
let revision = 0;
let current: ToolShape | null = null;

/** The tool box, or null when the arm is running bare. */
export function getToolShape(): ToolShape | null {
  return current ? { size: { ...current.size }, centre: { ...current.centre } } : null;
}

export function getToolShapeRevision(): number {
  return revision;
}

/** Largest tool this accepts, metres - past it the numbers are a typo, not a tool. */
export const MAX_TOOL_EXTENT_M = 1.0;

export type ToolShapeResult = { ok: true } | { ok: false; reason: string };

export function setToolShape(shape: ToolShape): ToolShapeResult {
  const dims = [shape.size.x, shape.size.y, shape.size.z];
  if (!dims.every(d => Number.isFinite(d) && d > 0)) {
    return { ok: false, reason: 'Every side has to be a positive length.' };
  }
  if (!Object.values(shape.centre).every(Number.isFinite)) {
    return { ok: false, reason: 'The centre has to be three numbers.' };
  }
  // A box bigger than the arm would refuse every pose, and the operator would
  // reasonably conclude the checker is broken rather than that the entry is.
  const reach = Math.max(
    ...dims,
    ...Object.values(shape.centre).map(Math.abs)
  );
  if (reach > MAX_TOOL_EXTENT_M) {
    return {
      ok: false,
      reason: `${(reach * 1000).toFixed(0)} mm is longer than the arm. Check the units — these are millimetres.`
    };
  }

  current = { size: { ...shape.size }, centre: { ...shape.centre } };
  revision++;
  return { ok: true };
}

/** Go back to a bare flange. */
export function clearToolShape(): void {
  current = null;
  revision++;
}

/** A box sitting flush on the flange face, which is what most tools do. */
export function flushOnFlange(size: Vector3): ToolShape {
  return { size: { ...size }, centre: { x: 0, y: 0, z: size.z / 2 } };
}
