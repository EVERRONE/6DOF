import { timingSafeEqual } from 'node:crypto';
import { normalizeCommandParams, ValidationError, PRIVILEGED_COMMANDS } from './protocol.js';

const MAX_BODY_BYTES = 64 * 1024;

const sendJson = (res, status, payload) => {
  const body = JSON.stringify(payload, null, 2);
  res.writeHead(status, {
    'content-type': 'application/json; charset=utf-8',
    'content-length': Buffer.byteLength(body),
    // Nothing here should ever be cached, and no browser should be reading it.
    'cache-control': 'no-store'
  });
  res.end(body);
};

const tokensMatch = (given, expected) => {
  if (typeof given !== 'string') return false;
  const a = Buffer.from(given);
  const b = Buffer.from(expected);
  if (a.length !== b.length) return false;
  return timingSafeEqual(a, b);
};

const readBody = (req) =>
  new Promise((resolve, reject) => {
    let size = 0;
    const chunks = [];
    req.on('data', (chunk) => {
      size += chunk.length;
      if (size > MAX_BODY_BYTES) {
        reject(new Error('Request body too large'));
        req.destroy();
        return;
      }
      chunks.push(chunk);
    });
    req.on('end', () => resolve(Buffer.concat(chunks).toString('utf8')));
    req.on('error', reject);
  });

export function createHttpApi({ config, state, executorLink, log }) {
  // Phase 1+2 ships nothing that can move the arm, so nothing requires arming
  // yet. The hook exists so that adding move_relative in phase 3 is a one-line
  // change here rather than a new code path.
  const commandNeedsArm = (type) => ['move_relative', 'move_to', 'move_joints', 'home'].includes(type);

  const runCommand = async (type, rawParams) => {
    let params;
    try {
      params = normalizeCommandParams(type, rawParams);
    } catch (err) {
      if (err instanceof ValidationError) {
        return { status: 400, payload: { ok: false, error: { code: 'INVALID_PARAMS', message: err.message } } };
      }
      throw err;
    }

    // Arming is enforced by the executor, which sits closest to the hardware
    // and owns the human's decision. Checking here too just turns a slow
    // round-trip into an immediate, clearer refusal.
    if (!PRIVILEGED_COMMANDS.includes(type) && commandNeedsArm(type) && !state.isArmed()) {
      const payload = {
        ok: false,
        error: {
          code: 'NOT_ARMED',
          message:
            'AI control is not armed. Someone has to arm it in the Agent Control panel of the robot app, at the machine.'
        }
      };
      state.recordAudit({ type, params, outcome: 'refused', reason: 'not armed' });
      return { status: 409, payload };
    }

    const outcome = await executorLink.send(type, params);

    state.recordAudit({
      type,
      params,
      outcome: outcome.ok ? 'ok' : 'error',
      reason: outcome.ok ? undefined : outcome.error?.message
    });

    if (outcome.ok) {
      return { status: 200, payload: { ok: true, result: outcome.result } };
    }

    const status = outcome.error?.code === 'NO_EXECUTOR' ? 503 : 502;
    return { status, payload: { ok: false, error: outcome.error } };
  };

  return async function handleRequest(req, res) {
    let url;
    try {
      url = new URL(req.url, `http://${req.headers.host ?? 'localhost'}`);
    } catch {
      sendJson(res, 400, { ok: false, error: { code: 'BAD_REQUEST', message: 'Malformed URL' } });
      return;
    }

    if (url.pathname === '/health') {
      sendJson(res, 200, {
        ok: true,
        result: {
          service: 'robot-agent-bridge',
          executorConnected: executorLink.isConnected(),
          armed: state.isArmed()
        }
      });
      return;
    }

    // A local HTTP server is reachable from any page the user visits. Real
    // clients (Hermes, curl) send no Origin header; browsers always do. So the
    // mere presence of Origin means this is a page trying to reach the robot.
    if (req.headers.origin) {
      sendJson(res, 403, {
        ok: false,
        error: {
          code: 'ORIGIN_REJECTED',
          message: 'Browser-originated requests are not accepted by the bridge API.'
        }
      });
      log(`rejected browser request from origin ${req.headers.origin}`);
      return;
    }

    const authHeader = req.headers.authorization ?? '';
    const bearer = authHeader.startsWith('Bearer ') ? authHeader.slice(7) : null;
    if (!tokensMatch(bearer, config.token)) {
      sendJson(res, 401, {
        ok: false,
        error: { code: 'UNAUTHORIZED', message: 'Missing or invalid bearer token.' }
      });
      return;
    }

    if (req.method === 'GET' && url.pathname === '/api/audit') {
      sendJson(res, 200, { ok: true, result: { entries: state.getAudit() } });
      return;
    }

    if (req.method === 'GET' && url.pathname === '/api/telemetry') {
      const cached = state.getTelemetry();
      sendJson(res, 200, {
        ok: true,
        result: {
          executorConnected: executorLink.isConnected(),
          telemetry: cached?.state ?? null,
          receivedAt: cached ? new Date(cached.receivedAt).toISOString() : null
        }
      });
      return;
    }

    const routes = {
      'GET /api/status': 'get_status',
      'POST /api/preview_move': 'preview_move',
      'POST /api/stop': 'stop'
    };

    const route = routes[`${req.method} ${url.pathname}`];
    if (!route) {
      sendJson(res, 404, {
        ok: false,
        error: {
          code: 'NOT_FOUND',
          message: `No such endpoint: ${req.method} ${url.pathname}. Available: GET /api/status, POST /api/preview_move, POST /api/stop, GET /api/telemetry, GET /api/audit.`
        }
      });
      return;
    }

    let params = {};
    if (req.method === 'POST') {
      try {
        const body = await readBody(req);
        params = body.trim().length > 0 ? JSON.parse(body) : {};
      } catch (err) {
        sendJson(res, 400, {
          ok: false,
          error: { code: 'BAD_BODY', message: `Could not read request body: ${err.message}` }
        });
        return;
      }
    }

    try {
      const { status, payload } = await runCommand(route, params);
      sendJson(res, status, payload);
    } catch (err) {
      log(`unhandled error on ${route}: ${err.stack ?? err.message}`);
      sendJson(res, 500, {
        ok: false,
        error: { code: 'INTERNAL', message: 'The bridge hit an unexpected error. Check its log.' }
      });
    }
  };
}
