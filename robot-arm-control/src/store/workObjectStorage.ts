// Remembering work objects between sessions.
//
// A work object describes where a fixture stands. That survives a page reload in
// reality, so it has to survive one here - re-teaching three points every time
// the browser is closed would make the feature worse than not having it.
//
// Saved paths carry their own copy, which is what makes a path portable between
// machines. This is only the working set.

import { WorkObject } from '../kinematics/workObject';

const KEY = 'robot-arm.workObjects.v1';

function isWorkObject(value: unknown): value is WorkObject {
  if (typeof value !== 'object' || value === null) return false;
  const o = value as Partial<WorkObject>;
  if (typeof o.id !== 'string' || typeof o.name !== 'string') return false;
  if (typeof o.origin !== 'object' || o.origin === null) return false;
  if (typeof o.rpy !== 'object' || o.rpy === null) return false;
  return [o.origin.x, o.origin.y, o.origin.z, o.rpy.roll, o.rpy.pitch, o.rpy.yaw].every(
    v => typeof v === 'number' && Number.isFinite(v)
  );
}

/**
 * The remembered work objects, or none.
 *
 * Anything unreadable is dropped rather than repaired. A half-understood frame
 * puts every point taught in it somewhere nobody chose, and the arm would go
 * there without complaint.
 */
export function loadWorkObjects(): WorkObject[] {
  try {
    const raw = window.localStorage?.getItem(KEY);
    if (!raw) return [];

    const parsed = JSON.parse(raw);
    if (!Array.isArray(parsed)) {
      window.localStorage?.removeItem(KEY);
      return [];
    }
    return parsed.filter(isWorkObject);
  } catch {
    // No storage at all - a test environment, or a browser with it disabled.
    return [];
  }
}

export function saveWorkObjects(objects: WorkObject[]): void {
  try {
    window.localStorage?.setItem(KEY, JSON.stringify(objects));
  } catch {
    // Storage full or unavailable. The frames are still in force for this
    // session, which is what matters.
  }
}
