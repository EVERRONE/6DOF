import { spawn, ChildProcess } from 'child_process';
import http from 'http';
import path from 'path';
import { AgentBridgeClient } from '../AgentBridgeClient';
import { AgentCommandExecutor } from '../AgentCommandExecutor';
import { useRobotStore } from '../../store/robotStore';
import { ConnectionStatus, RobotState } from '../../types/robot';

/**
 * End-to-end across the process boundary: a real broker in a child process,
 * the real browser client over a real WebSocket, driving the real store.
 *
 * Everything else in this repo tests one side against a stand-in for the other,
 * which leaves a symmetric blind spot — both halves can agree with my
 * assumptions and disagree with each other. In particular this is the only
 * thing that forces robot-agent-bridge/src/protocol.js and
 * src/agent/bridgeProtocol.ts to actually match: the broker normalises the
 * parameters and the client consumes them, so a rename on one side fails here.
 *
 * HTTP requests deliberately use node:http rather than fetch, because a browser
 * request carries an Origin header and the broker refuses those by design.
 */

const BROKER_ENTRY = path.resolve(__dirname, '../../../../robot-agent-bridge/src/index.js');
const TOKEN = 'integration-test-token';

jest.setTimeout(30_000);

let broker: ChildProcess;
let brokerPort: number;

const startBroker = (): Promise<number> =>
  new Promise((resolve, reject) => {
    broker = spawn(process.execPath, [BROKER_ENTRY], {
      env: {
        ...process.env,
        BRIDGE_PORT: '0',
        BRIDGE_HOST: '127.0.0.1',
        BRIDGE_TOKEN: TOKEN,
        BRIDGE_COMMAND_TIMEOUT_MS: '5000',
        BRIDGE_MOTION_TIMEOUT_MS: '10000'
      },
      stdio: ['ignore', 'pipe', 'pipe']
    });

    const timer = setTimeout(() => reject(new Error('broker did not start in time')), 15_000);

    broker.stdout?.on('data', (chunk: Buffer) => {
      const match = /listening on 127\.0\.0\.1:(\d+)/.exec(chunk.toString());
      if (match) {
        clearTimeout(timer);
        resolve(Number(match[1]));
      }
    });

    broker.on('error', (err) => {
      clearTimeout(timer);
      reject(err);
    });
  });

interface ApiResponse {
  status: number;
  body: any;
}

const api = (method: string, pathname: string, body?: unknown): Promise<ApiResponse> =>
  new Promise((resolve, reject) => {
    const payload = body === undefined ? undefined : JSON.stringify(body);
    const req = http.request(
      {
        host: '127.0.0.1',
        port: brokerPort,
        method,
        path: pathname,
        headers: {
          authorization: `Bearer ${TOKEN}`,
          ...(payload ? { 'content-type': 'application/json', 'content-length': Buffer.byteLength(payload) } : {})
        }
      },
      (res) => {
        let raw = '';
        res.on('data', (c) => (raw += c));
        res.on('end', () => {
          try {
            resolve({ status: res.statusCode ?? 0, body: raw ? JSON.parse(raw) : null });
          } catch (err) {
            reject(new Error(`bad JSON from ${pathname}: ${raw}`));
          }
        });
      }
    );
    req.on('error', reject);
    if (payload) req.write(payload);
    req.end();
  });

const waitFor = async (predicate: () => boolean | Promise<boolean>, timeoutMs = 8000): Promise<void> => {
  const deadline = Date.now() + timeoutMs;
  while (Date.now() < deadline) {
    if (await predicate()) return;
    await new Promise((r) => setTimeout(r, 50));
  }
  throw new Error('condition not met in time');
};

const firmwareConfig = {
  version: 1,
  capabilities: { trajectoryQueue: true, trajectoryMaxPoints: 256, trajectoryPointFormat: 'hermite_v1' },
  joints: Array.from({ length: 6 }, () => ({
    min: -120,
    max: 120,
    stepsPerDeg: 100,
    invertDir: false,
    hasEndstop: true,
    homeTowardMin: false,
    homeLogicalDeg: 0,
    postHomeOffsetDeg: 0,
    urdfOffsetDeg: 0,
    cal: { scale: 1, offset: 0 }
  })),
  pulseWidthUs: 3,
  dirSetupUs: 5,
  endstopDebounceMs: 20
} as any;

let client: AgentBridgeClient;
let executor: AgentCommandExecutor;
let moveSpy: jest.Mock;
let stopSpy: jest.Mock;

const resetStore = () => {
  moveSpy = jest.fn().mockResolvedValue(undefined);
  stopSpy = jest.fn().mockResolvedValue(undefined);
  useRobotStore.setState({
    connectionStatus: ConnectionStatus.CONNECTED,
    robotState: RobotState.IDLE,
    motorsEnabled: true,
    homedJoints: { J1: true, J2: true, J3: true, J4: true, J5: true, J6: true },
    kinematicsFrameReady: true,
    firmwareConfig,
    currentAngles: { J1: 0, J2: -45, J3: 60, J4: -90, J5: -90, J6: 0 },
    moveInProgress: false,
    planningState: 'idle',
    planningNotes: [],
    ikStatus: null,
    moveToPosition: moveSpy,
    emergencyStop: stopSpy
  } as any);
};

beforeAll(async () => {
  brokerPort = await startBroker();
});

afterAll(async () => {
  client?.stop();
  broker?.kill();
  await new Promise((r) => setTimeout(r, 100));
});

beforeEach(async () => {
  resetStore();
  executor = new AgentCommandExecutor({ settleTimeoutMs: 4000 });
  client = new AgentBridgeClient({
    url: `ws://127.0.0.1:${brokerPort}/executor`,
    token: TOKEN,
    executor
  });
  client.start();
  await waitFor(async () => (await api('GET', '/health')).body.result.executorConnected);
});

afterEach(async () => {
  client.stop();
  await waitFor(async () => !(await api('GET', '/health')).body.result.executorConnected);
});

describe('the broker and the browser client, end to end', () => {
  it('answers get_status from the live store', async () => {
    const res = await api('GET', '/api/status');

    expect(res.status).toBe(200);
    expect(res.body.ok).toBe(true);
    expect(res.body.result.connected).toBe(true);
    expect(res.body.result.motorsEnabled).toBe(true);
    // Proves FK ran in the executor rather than a canned value coming back.
    expect(res.body.result.tcpPositionMm).toEqual(
      expect.objectContaining({ x: expect.any(Number), y: expect.any(Number), z: expect.any(Number) })
    );
  });

  it('previews a move without commanding anything', async () => {
    const res = await api('POST', '/api/preview_move', { direction: 'right', distance_mm: 5 });

    expect(res.status).toBe(200);
    // The broker's normalised params survived the hop and the client read them
    // correctly -- this is the assertion that catches protocol drift.
    expect(res.body.result.resolved.worldDeltaMm).toEqual({ x: 0, y: -5, z: 0 });
    expect(res.body.result.executed).toBe(false);
    expect(moveSpy).not.toHaveBeenCalled();
  });

  it('applies the default distance when the agent names none', async () => {
    const res = await api('POST', '/api/preview_move', { direction: 'up' });
    expect(res.body.result.distanceMm).toBe(50);
    expect(res.body.result.resolved.worldDeltaMm).toEqual({ x: 0, y: 0, z: 50 });
  });

  it('clamps an over-long request end to end and says it clamped', async () => {
    const res = await api('POST', '/api/preview_move', { direction: 'right', distance_mm: 4000 });
    expect(res.body.result.distanceMm).toBe(150);
    expect(res.body.result.clampedFrom).toBe(4000);
  });

  it('refuses a move when nobody has armed it at the machine', async () => {
    const res = await api('POST', '/api/move_relative', { direction: 'right', distance_mm: 5 });

    expect(res.status).toBe(409);
    expect(res.body.error.code).toBe('NOT_ARMED');
    expect(moveSpy).not.toHaveBeenCalled();
  });

  it('executes a move once armed and reports arrival after the completion signal', async () => {
    executor.arm(5);
    await waitFor(async () => (await api('GET', '/health')).body.result.armed);

    const pending = api('POST', '/api/move_relative', { direction: 'right', distance_mm: 5 });

    await waitFor(() => moveSpy.mock.calls.length > 0);

    // The queue would be running at this point; nothing has arrived yet.
    useRobotStore.setState({ moveInProgress: true, robotState: RobotState.MOVING } as any);
    useRobotStore.setState({ moveInProgress: false, robotState: RobotState.IDLE } as any);

    const res = await pending;
    expect(res.status).toBe(200);
    expect(res.body.result.outcome).toBe('arrived');
    expect(res.body.result.resolved.worldDeltaMm).toEqual({ x: 0, y: -5, z: 0 });

    // keep_orientation defaults to true, which must reach moveToPosition as an
    // explicit rotation -- that is what forces pose_lock.
    expect(moveSpy.mock.calls[0][1]).toBeDefined();
  });

  it('passes a planner failure back to the agent verbatim', async () => {
    executor.arm(5);
    await waitFor(async () => (await api('GET', '/health')).body.result.armed);

    const pending = api('POST', '/api/move_relative', { direction: 'right', distance_mm: 5 });
    await waitFor(() => moveSpy.mock.calls.length > 0);

    useRobotStore.setState({
      planningState: 'failed',
      ikStatus: { success: false, error: 'Target is outside the robot\'s reachable workspace (joint limit).' },
      planningNotes: ['Mode=pose_lock']
    } as any);

    const res = await pending;
    expect(res.status).toBe(502);
    expect(res.body.error.message).toBe(
      "Target is outside the robot's reachable workspace (joint limit)."
    );
  });

  it('refuses a move while an emergency stop is latched', async () => {
    executor.arm(5);
    await waitFor(async () => (await api('GET', '/health')).body.result.armed);
    useRobotStore.setState({ robotState: RobotState.ESTOPPED } as any);

    const res = await api('POST', '/api/move_relative', { direction: 'right', distance_mm: 5 });

    expect(res.status).toBe(502);
    expect(res.body.error.code).toBe('ESTOP_LATCHED');
    expect(moveSpy).not.toHaveBeenCalled();
  });

  it('stops without arming, and disarms as a side effect', async () => {
    executor.arm(5);
    await waitFor(async () => (await api('GET', '/health')).body.result.armed);

    const res = await api('POST', '/api/stop');

    expect(res.status).toBe(200);
    expect(stopSpy).toHaveBeenCalled();
    expect(executor.isArmed()).toBe(false);
    // The panel reads the executor, so the UI cannot keep showing "Armed".
    expect(executor.getArmedUntil()).toBeNull();
  });

  it('rejects a bad token at the API', async () => {
    const res = await new Promise<ApiResponse>((resolve, reject) => {
      const req = http.request(
        {
          host: '127.0.0.1',
          port: brokerPort,
          method: 'GET',
          path: '/api/status',
          headers: { authorization: 'Bearer wrong-token-entirely' }
        },
        (r) => {
          let raw = '';
          r.on('data', (c) => (raw += c));
          r.on('end', () => resolve({ status: r.statusCode ?? 0, body: raw ? JSON.parse(raw) : null }));
        }
      );
      req.on('error', reject);
      req.end();
    });

    expect(res.status).toBe(401);
  });

  it('records the exchange in the audit log', async () => {
    await api('POST', '/api/preview_move', { direction: 'left', distance_mm: 10 });
    const res = await api('GET', '/api/audit');

    const entry = res.body.result.entries.find((e: any) => e.type === 'preview_move');
    expect(entry).toBeDefined();
    expect(entry.outcome).toBe('ok');
  });
});

describe('a client with the wrong token', () => {
  it('is told why, and stops retrying instead of looping forever', async () => {
    const states: string[] = [];
    const details: string[] = [];

    const badClient = new AgentBridgeClient({
      url: `ws://127.0.0.1:${brokerPort}/executor`,
      token: 'not-the-right-token',
      onLinkStateChange: (state, detail) => {
        states.push(state);
        if (detail) details.push(detail);
      }
    });

    badClient.start();
    await waitFor(() => states.includes('error'), 8000);

    expect(details.join(' ')).toMatch(/token/i);
    // 'reconnecting' would mean it is going to keep hammering a door that will
    // never open.
    expect(states).not.toContain('reconnecting');

    badClient.stop();
  });
});
