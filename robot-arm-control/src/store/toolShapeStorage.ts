// Remembering what is bolted to the flange between sessions.
//
// Same reasoning as the tool frame next door: this describes hardware, and
// re-typing it at every page load would guarantee it is sometimes wrong. The
// consequence of forgetting is worse here, though - a forgotten tool frame makes
// positions wrong, a forgotten tool box makes the collision checker clear poses
// that drive the tool into the arm.

import { ToolShape } from '../kinematics/toolGeometry';

const KEY = 'robot-arm.toolShape.v1';

function isToolShape(value: unknown): value is ToolShape {
  if (typeof value !== 'object' || value === null) return false;
  const { size, centre } = value as Partial<ToolShape>;
  if (typeof size !== 'object' || size === null) return false;
  if (typeof centre !== 'object' || centre === null) return false;
  return (
    [size.x, size.y, size.z].every(v => typeof v === 'number' && Number.isFinite(v) && v > 0) &&
    [centre.x, centre.y, centre.z].every(v => typeof v === 'number' && Number.isFinite(v))
  );
}

/**
 * The remembered tool box, or null for a bare flange.
 *
 * Anything unreadable is discarded rather than repaired. Half a tool box is
 * worse than none: it claims space that may be empty and, worse, leaves space
 * unclaimed that is not.
 */
export function loadToolShape(): ToolShape | null {
  try {
    const raw = window.localStorage?.getItem(KEY);
    if (!raw) return null;

    const parsed = JSON.parse(raw);
    if (!isToolShape(parsed)) {
      window.localStorage?.removeItem(KEY);
      return null;
    }

    return { size: { ...parsed.size }, centre: { ...parsed.centre } };
  } catch {
    // No storage at all - a test environment, or a browser with it disabled.
    return null;
  }
}

export function saveToolShape(shape: ToolShape | null): void {
  try {
    if (shape) window.localStorage?.setItem(KEY, JSON.stringify(shape));
    else window.localStorage?.removeItem(KEY);
  } catch {
    // Storage full or unavailable. The shape is still in force for this session,
    // which is what matters for the guard.
  }
}
