import { test, before, after, describe } from 'node:test';
import assert from 'node:assert/strict';
import { WebSocket } from 'ws';
import { startBridge } from '../src/index.js';
import { normalizeCommandParams, ValidationError, MAX_DISTANCE_MM } from '../src/protocol.js';

const TOKEN = 'test-token-value';

let bridge;
let baseUrl;

const call = (method, path, { body, token = TOKEN, headers = {} } = {}) =>
  fetch(`${baseUrl}${path}`, {
    method,
    headers: {
      ...(token ? { authorization: `Bearer ${token}` } : {}),
      ...(body ? { 'content-type': 'application/json' } : {}),
      ...headers
    },
    body: body ? JSON.stringify(body) : undefined
  });

/** A stand-in executor that answers commands the way the browser app will. */
const connectExecutor = (handler) =>
  new Promise((resolve, reject) => {
    const ws = new WebSocket(`${baseUrl.replace('http', 'ws')}/executor?token=${TOKEN}`);
    ws.on('open', () => {
      ws.send(JSON.stringify({ type: 'hello', app: 'fake-executor', version: '1' }));
      resolve(ws);
    });
    ws.on('error', reject);
    ws.on('message', (raw) => {
      const cmd = JSON.parse(raw.toString());
      const reply = handler(cmd);
      if (reply) ws.send(JSON.stringify({ id: cmd.id, ...reply }));
    });
  });

const waitFor = async (predicate, timeoutMs = 2000) => {
  const deadline = Date.now() + timeoutMs;
  while (Date.now() < deadline) {
    if (await predicate()) return true;
    await new Promise((r) => setTimeout(r, 25));
  }
  throw new Error('condition not met in time');
};

before(async () => {
  bridge = await startBridge({
    BRIDGE_PORT: '0',
    BRIDGE_HOST: '127.0.0.1',
    BRIDGE_TOKEN: TOKEN,
    BRIDGE_COMMAND_TIMEOUT_MS: '1000'
  });
  baseUrl = `http://127.0.0.1:${bridge.server.address().port}`;
});

after(() => {
  bridge.server.close();
});

describe('parameter validation', () => {
  test('rejects an unknown direction with a message naming the valid ones', () => {
    assert.throws(
      () => normalizeCommandParams('preview_move', { direction: 'sideways' }),
      (err) => err instanceof ValidationError && /right, left, forward/.test(err.message)
    );
  });

  test('clamps an over-long distance instead of rejecting it', () => {
    const params = normalizeCommandParams('preview_move', { direction: 'right', distance_mm: 900 });
    assert.equal(params.distanceMm, MAX_DISTANCE_MM);
    assert.equal(params.requestedDistanceMm, 900);
    assert.equal(params.clamped, true);
  });

  test('applies the default distance when none is given', () => {
    const params = normalizeCommandParams('preview_move', { direction: 'right' });
    assert.equal(params.distanceMm, 50);
    assert.equal(params.clamped, false);
  });
});

describe('authentication', () => {
  test('rejects a missing token', async () => {
    const res = await call('GET', '/api/status', { token: null });
    assert.equal(res.status, 401);
  });

  test('rejects a wrong token', async () => {
    const res = await call('GET', '/api/status', { token: 'nope-wrong-length-x' });
    assert.equal(res.status, 401);
  });

  test('rejects any request carrying an Origin header', async () => {
    const res = await call('GET', '/api/status', { headers: { origin: 'https://evil.example' } });
    assert.equal(res.status, 403);
    const body = await res.json();
    assert.equal(body.error.code, 'ORIGIN_REJECTED');
  });

  test('serves /health without a token', async () => {
    const res = await fetch(`${baseUrl}/health`);
    assert.equal(res.status, 200);
    assert.equal((await res.json()).ok, true);
  });
});

describe('with no executor connected', () => {
  test('reports 503 and says what to do about it', async () => {
    const res = await call('GET', '/api/status');
    assert.equal(res.status, 503);
    const body = await res.json();
    assert.equal(body.error.code, 'NO_EXECUTOR');
    assert.match(body.error.message, /Agent Control panel/);
  });
});

describe('with an executor connected', () => {
  let ws;

  after(() => ws?.close());

  test('round-trips get_status', async () => {
    ws = await connectExecutor((cmd) =>
      cmd.type === 'get_status'
        ? { ok: true, result: { connected: true, motorsEnabled: false } }
        : { ok: false, error: { message: 'unexpected' } }
    );
    await waitFor(async () => (await fetch(`${baseUrl}/health`).then((r) => r.json())).result.executorConnected);

    const res = await call('GET', '/api/status');
    assert.equal(res.status, 200);
    const body = await res.json();
    assert.equal(body.ok, true);
    assert.equal(body.result.motorsEnabled, false);
  });

  test('passes executor errors through verbatim', async () => {
    ws.close();
    await waitFor(async () => !(await fetch(`${baseUrl}/health`).then((r) => r.json())).result.executorConnected);

    ws = await connectExecutor(() => ({
      ok: false,
      error: { code: 'NOT_READY', message: 'Home J2..J5 before Cartesian move' }
    }));
    await waitFor(async () => (await fetch(`${baseUrl}/health`).then((r) => r.json())).result.executorConnected);

    const res = await call('POST', '/api/preview_move', { body: { direction: 'right' } });
    assert.equal(res.status, 502);
    const body = await res.json();
    assert.equal(body.error.message, 'Home J2..J5 before Cartesian move');
  });

  test('times out rather than hanging when the executor goes quiet', async () => {
    ws.close();
    await waitFor(async () => !(await fetch(`${baseUrl}/health`).then((r) => r.json())).result.executorConnected);

    ws = await connectExecutor(() => null); // never answers
    await waitFor(async () => (await fetch(`${baseUrl}/health`).then((r) => r.json())).result.executorConnected);

    const res = await call('GET', '/api/status');
    assert.equal(res.status, 502);
    assert.equal((await res.json()).error.code, 'TIMEOUT');
  });

  test('records commands in the audit log', async () => {
    const res = await call('GET', '/api/audit');
    const body = await res.json();
    assert.ok(body.result.entries.length > 0);
    assert.ok(body.result.entries.every((e) => typeof e.at === 'string' && typeof e.type === 'string'));
  });
});
