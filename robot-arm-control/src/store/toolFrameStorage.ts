// Remembering the tool frame between sessions.
//
// The tool frame describes hardware: where the tool is bolted and which way it
// points. Re-typing a measurement at every page load would guarantee it is
// sometimes wrong, so it is remembered.
//
// It is remembered and not owned. robotModel.ts stays the source of truth, the
// same way config.h does for the firmware's limits - a value that only exists in
// a browser's local storage is one nobody can review, that differs between two
// machines driving the same arm, and that quietly disagrees with the code. The
// panel shows the line to paste into robotModel.ts, and says plainly when what
// is in force differs from what is compiled in.

import { DEFAULT_TOOL_FRAME, ToolFrame } from '../kinematics/robotModel';

const KEY = 'robot-arm.toolFrame.v1';

/** Every field, finite, or it is not a tool frame. */
function isToolFrame(value: unknown): value is ToolFrame {
  if (typeof value !== 'object' || value === null) return false;
  const { xyz, rpy } = value as Partial<ToolFrame>;
  if (typeof xyz !== 'object' || xyz === null) return false;
  if (typeof rpy !== 'object' || rpy === null) return false;
  return (
    [xyz.x, xyz.y, xyz.z, rpy.roll, rpy.pitch, rpy.yaw].every(
      v => typeof v === 'number' && Number.isFinite(v)
    )
  );
}

/**
 * The remembered tool frame, or the bare flange.
 *
 * Anything unreadable is discarded rather than repaired. A half-understood tool
 * frame is worse than none: it puts the TCP somewhere nobody chose, and every
 * position the arm reports is then measured to that point.
 */
export function loadToolFrame(): ToolFrame {
  try {
    const raw = window.localStorage?.getItem(KEY);
    if (!raw) return { xyz: { ...DEFAULT_TOOL_FRAME.xyz }, rpy: { ...DEFAULT_TOOL_FRAME.rpy } };

    const parsed = JSON.parse(raw);
    if (!isToolFrame(parsed)) {
      window.localStorage?.removeItem(KEY);
      return { xyz: { ...DEFAULT_TOOL_FRAME.xyz }, rpy: { ...DEFAULT_TOOL_FRAME.rpy } };
    }

    return { xyz: { ...parsed.xyz }, rpy: { ...parsed.rpy } };
  } catch {
    // No storage at all - a test environment, or a browser with it disabled.
    return { xyz: { ...DEFAULT_TOOL_FRAME.xyz }, rpy: { ...DEFAULT_TOOL_FRAME.rpy } };
  }
}

export function saveToolFrame(frame: ToolFrame): void {
  try {
    window.localStorage?.setItem(KEY, JSON.stringify(frame));
  } catch {
    // Storage full or unavailable. The frame is still in force for this session,
    // which is what matters; losing it on reload is a smaller problem than
    // failing the call that set it.
  }
}

export function clearStoredToolFrame(): void {
  try {
    window.localStorage?.removeItem(KEY);
  } catch {
    // As above.
  }
}
