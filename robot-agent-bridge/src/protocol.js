/**
 * Wire contract between the broker and whatever is executing commands.
 *
 * The executor is currently the web app in a browser tab (over WebSocket), but
 * nothing here assumes that. A future headless Node executor implements the
 * same two message shapes and the broker does not change.
 *
 * Keep this file in sync with robot-arm-control/src/agent/bridgeProtocol.ts.
 */

export const PROTOCOL_VERSION = 1;

/**
 * Commands the broker may send to an executor.
 *
 * `stop` can only ever reduce motion, so it is never gated behind arming.
 */
export const COMMAND_TYPES = Object.freeze([
  'get_status',
  'preview_move',
  'stop',
  'move_relative',
  'move_to'
]);

/**
 * Commands refused unless the executor reports itself armed.
 *
 * The executor enforces this too, and its answer is the authoritative one — it
 * sits closest to the hardware and owns the human's decision. Checking here
 * only turns a slow round-trip into an immediate, clearer refusal.
 */
export const COMMANDS_REQUIRING_ARM = Object.freeze(['move_relative', 'move_to']);

/** Commands that bypass the busy guard and the arming check entirely. */
export const PRIVILEGED_COMMANDS = Object.freeze(['stop']);

export const DIRECTIONS = Object.freeze([
  'right',
  'left',
  'forward',
  'backward',
  'up',
  'down'
]);

export const FRAMES = Object.freeze(['base', 'view']);

/** Hard ceiling on any single commanded displacement, in millimetres. */
export const MAX_DISTANCE_MM = 150;

/** Used when the agent names a direction without a distance. */
export const DEFAULT_DISTANCE_MM = 50;

export class ValidationError extends Error {
  constructor(message) {
    super(message);
    this.name = 'ValidationError';
  }
}

const isFiniteNumber = (value) => typeof value === 'number' && Number.isFinite(value);

/**
 * Validates and normalizes the params for a command.
 *
 * Throws ValidationError with a message written for a language model to read:
 * it says what was wrong and what the accepted values are.
 */
export function normalizeCommandParams(type, rawParams) {
  const params = rawParams && typeof rawParams === 'object' ? rawParams : {};

  if (type === 'get_status' || type === 'stop') {
    return {};
  }

  if (type === 'preview_move' || type === 'move_relative') {
    const { direction, frame = 'base' } = params;

    if (!DIRECTIONS.includes(direction)) {
      throw new ValidationError(
        `Unknown direction "${direction}". Use one of: ${DIRECTIONS.join(', ')}.`
      );
    }

    if (!FRAMES.includes(frame)) {
      throw new ValidationError(
        `Unknown frame "${frame}". Use one of: ${FRAMES.join(', ')}.`
      );
    }

    let distanceMm = params.distance_mm ?? DEFAULT_DISTANCE_MM;
    if (!isFiniteNumber(distanceMm)) {
      throw new ValidationError('distance_mm must be a number in millimetres.');
    }
    if (distanceMm <= 0) {
      throw new ValidationError('distance_mm must be greater than zero.');
    }

    // Clamp rather than reject: an agent asking for too much should still get a
    // useful answer, with the clamp reported back so it can see what happened.
    const clamped = Math.min(distanceMm, MAX_DISTANCE_MM);

    return {
      direction,
      frame,
      distanceMm: clamped,
      requestedDistanceMm: distanceMm,
      clamped: clamped !== distanceMm,
      keepOrientation: params.keep_orientation !== false,
      wait: params.wait !== false
    };
  }

  if (type === 'move_to') {
    const axes = { x: params.x_mm, y: params.y_mm, z: params.z_mm };
    for (const [axis, value] of Object.entries(axes)) {
      if (!isFiniteNumber(value)) {
        throw new ValidationError(
          `${axis}_mm must be a number in millimetres, measured from the robot base.`
        );
      }
    }

    return {
      targetMm: { x: axes.x, y: axes.y, z: axes.z },
      keepOrientation: params.keep_orientation !== false,
      wait: params.wait !== false
    };
  }

  throw new ValidationError(
    `Unknown command "${type}". Use one of: ${COMMAND_TYPES.join(', ')}.`
  );
}

export function buildCommand(id, type, params) {
  return { v: PROTOCOL_VERSION, id, type, params };
}

/** Shape check for anything arriving from an executor. */
export function parseExecutorMessage(raw) {
  let msg;
  try {
    msg = JSON.parse(raw);
  } catch {
    return { kind: 'invalid', reason: 'not valid JSON' };
  }

  if (!msg || typeof msg !== 'object') {
    return { kind: 'invalid', reason: 'not an object' };
  }

  if (msg.type === 'hello') {
    return { kind: 'hello', app: String(msg.app ?? 'unknown'), version: String(msg.version ?? '') };
  }

  if (msg.type === 'telemetry') {
    return { kind: 'telemetry', state: msg.state ?? {} };
  }

  if (typeof msg.id === 'string') {
    return {
      kind: 'response',
      id: msg.id,
      ok: msg.ok === true,
      result: msg.result ?? null,
      error: msg.error ?? null
    };
  }

  return { kind: 'invalid', reason: 'unrecognised message' };
}
