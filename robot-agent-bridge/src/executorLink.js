import { randomUUID } from 'node:crypto';
import { WebSocketServer } from 'ws';
import { buildCommand, parseExecutorMessage } from './protocol.js';

/** Application close codes, mirrored in src/agent/AgentBridgeClient.ts. */
export const CLOSE_UNAUTHORIZED = 4001;
export const CLOSE_ALREADY_CONNECTED = 4002;

/**
 * Holds the WebSocket link to the executor and correlates request/response.
 *
 * The executor dials *in* to us. That is deliberate: it means the machine with
 * the serial port needs no inbound port, no port forwarding and no stable
 * address, while the always-on machine hosting the agent holds the endpoint.
 *
 * Exactly one executor at a time. A second connection is refused rather than
 * silently taking over — two things driving one arm is not a state worth
 * supporting.
 */
export function createExecutorLink({ server, token, state, commandTimeoutMs, log }) {
  const wss = new WebSocketServer({ noServer: true });
  const pending = new Map();

  let socket = null;

  const settle = (id, outcome) => {
    const entry = pending.get(id);
    if (!entry) return;
    pending.delete(id);
    clearTimeout(entry.timer);
    entry.resolve(outcome);
  };

  const failAllPending = (message) => {
    for (const id of Array.from(pending.keys())) {
      settle(id, { ok: false, error: { message, code: 'EXECUTOR_LOST' } });
    }
  };

  server.on('upgrade', (request, rawSocket, head) => {
    let url;
    try {
      url = new URL(request.url, 'http://localhost');
    } catch {
      rawSocket.destroy();
      return;
    }

    if (url.pathname !== '/executor') {
      rawSocket.destroy();
      return;
    }

    // Browsers cannot set headers on a WebSocket handshake, so the token
    // travels as a query parameter. It stays inside the LAN and never reaches
    // a third-party server, but it does mean the URL should be treated as a
    // secret (it is not logged below).
    // A browser cannot see why an HTTP upgrade failed — it reports a generic
    // close either way. So rather than refusing the handshake, accept it and
    // close with a code and reason the client can actually display. Otherwise a
    // mistyped token looks identical to a broker that is not running.
    const rejection = url.searchParams.get('token') !== token
      ? { code: CLOSE_UNAUTHORIZED, reason: 'Invalid bridge token.', log: 'bad token' }
      : socket
        ? {
            code: CLOSE_ALREADY_CONNECTED,
            reason: 'Another robot app is already connected to this bridge.',
            log: 'one is already connected'
          }
        : null;

    if (rejection) {
      wss.handleUpgrade(request, rawSocket, head, (ws) => {
        ws.close(rejection.code, rejection.reason);
      });
      log(`executor rejected: ${rejection.log}`);
      return;
    }

    wss.handleUpgrade(request, rawSocket, head, (ws) => {
      socket = ws;
      state.setExecutor({ connectedAt: Date.now(), app: 'pending-hello' });
      log('executor connected');

      ws.on('message', (data) => {
        const msg = parseExecutorMessage(data.toString());

        switch (msg.kind) {
          case 'hello':
            state.setExecutor({ connectedAt: Date.now(), app: msg.app, version: msg.version });
            log(`executor identified: ${msg.app} ${msg.version}`);
            break;
          case 'telemetry':
            state.setTelemetry(msg.state);
            break;
          case 'response':
            settle(msg.id, { ok: msg.ok, result: msg.result, error: msg.error });
            break;
          default:
            log(`ignored executor message: ${msg.reason}`);
        }
      });

      const drop = (reason) => {
        if (socket !== ws) return;
        socket = null;
        state.setExecutor(null);
        failAllPending(`Executor disconnected (${reason}) before the command completed.`);
        log(`executor disconnected: ${reason}`);
      };

      ws.on('close', () => drop('closed'));
      ws.on('error', (err) => drop(err.message));
    });
  });

  return {
    isConnected: () => socket !== null,

    /**
     * Sends a command and waits for the matching response.
     *
     * Never rejects — a timeout or a lost executor comes back as a normal
     * failure result, because "I do not know what happened" is information the
     * agent needs, not an exception to swallow.
     */
    send(type, params, timeoutMs = commandTimeoutMs) {
      if (!socket) {
        return Promise.resolve({
          ok: false,
          error: {
            code: 'NO_EXECUTOR',
            message:
              'No robot app is connected to the bridge. Open the control app in Chrome and enable the agent bridge in the Agent Control panel.'
          }
        });
      }

      const id = randomUUID();

      return new Promise((resolve) => {
        const timer = setTimeout(() => {
          pending.delete(id);
          resolve({
            ok: false,
            error: {
              code: 'TIMEOUT',
              message: `The robot app did not answer within ${timeoutMs} ms. The command may or may not have taken effect — read /api/status before retrying.`
            }
          });
        }, timeoutMs);

        pending.set(id, { resolve, timer });

        try {
          socket.send(JSON.stringify(buildCommand(id, type, params)));
        } catch (err) {
          settle(id, {
            ok: false,
            error: { code: 'SEND_FAILED', message: `Could not reach the robot app: ${err.message}` }
          });
        }
      });
    }
  };
}
