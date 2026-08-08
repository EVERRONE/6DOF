import { AgentCommandExecutor, BridgeCommandError } from './AgentCommandExecutor';
import { BRIDGE_PROTOCOL_VERSION, BridgeCommand, BridgeError, BridgeLinkState } from './bridgeProtocol';

/**
 * Transport only: opens the WebSocket to the broker, pushes telemetry, and
 * hands each command to the executor.
 *
 * The client dials *out*, so the machine holding the serial port needs no
 * inbound port and no stable address.
 */

const RECONNECT_DELAYS_MS = [1000, 2000, 4000, 8000, 15000];
const TELEMETRY_INTERVAL_MS = 1000;

/**
 * Close codes the broker uses to explain a rejected handshake.
 *
 * A failed HTTP upgrade is invisible to a browser — it sees only a generic
 * close — so the broker accepts the socket and closes it with a reason instead.
 * Without this, a mistyped token looks exactly like a broker that is not
 * running, which is a miserable thing to debug at the bench.
 */
export const CLOSE_UNAUTHORIZED = 4001;
export const CLOSE_ALREADY_CONNECTED = 4002;

/** Reasons it is pointless to keep retrying. */
const FATAL_CLOSE_CODES = new Set<number>([CLOSE_UNAUTHORIZED]);

export interface AgentActivityEntry {
  at: number;
  type: string;
  ok: boolean;
  summary: string;
}

export interface AgentBridgeClientOptions {
  url: string;
  token: string;
  executor?: AgentCommandExecutor;
  onLinkStateChange?: (state: BridgeLinkState, detail?: string) => void;
  onActivity?: (entry: AgentActivityEntry) => void;
}

/**
 * Tolerates a trailing slash or a stray query string, which the broker's exact
 * path match would otherwise reject as a confusing silent failure.
 */
export const normalizeBridgeUrl = (raw: string): string => {
  const trimmed = raw.trim().replace(/\?.*$/, '').replace(/\/+$/, '');
  return trimmed.length > 0 ? trimmed : raw.trim();
};

export class AgentBridgeClient {
  readonly executor: AgentCommandExecutor;

  private socket: WebSocket | null = null;
  private options: AgentBridgeClientOptions;
  private enabled = false;
  private reconnectAttempt = 0;
  private reconnectTimer: ReturnType<typeof setTimeout> | null = null;
  private telemetryTimer: ReturnType<typeof setInterval> | null = null;

  constructor(options: AgentBridgeClientOptions) {
    this.options = options;
    this.executor = options.executor ?? new AgentCommandExecutor();
  }

  start(): void {
    if (this.enabled) return;
    this.enabled = true;
    this.reconnectAttempt = 0;
    this.openSocket();
  }

  stop(): void {
    this.enabled = false;
    this.clearTimers();
    if (this.socket) {
      const socket = this.socket;
      this.socket = null;
      try {
        socket.close();
      } catch {
        // Already closing — nothing useful to do.
      }
    }
    this.options.onLinkStateChange?.('disabled');
  }

  private clearTimers(): void {
    if (this.reconnectTimer) {
      clearTimeout(this.reconnectTimer);
      this.reconnectTimer = null;
    }
    if (this.telemetryTimer) {
      clearInterval(this.telemetryTimer);
      this.telemetryTimer = null;
    }
  }

  private openSocket(): void {
    if (!this.enabled) return;

    this.options.onLinkStateChange?.(this.reconnectAttempt === 0 ? 'connecting' : 'reconnecting');

    let socket: WebSocket;
    try {
      // The token travels in the query string because a browser cannot set
      // headers on a WebSocket handshake. It stays on the LAN.
      const base = normalizeBridgeUrl(this.options.url);
      socket = new WebSocket(`${base}?token=${encodeURIComponent(this.options.token)}`);
    } catch (error) {
      this.scheduleReconnect(error instanceof Error ? error.message : String(error));
      return;
    }

    this.socket = socket;

    socket.onopen = () => {
      this.reconnectAttempt = 0;
      this.options.onLinkStateChange?.('connected');
      this.send({ type: 'hello', app: 'robot-arm-control', version: BRIDGE_PROTOCOL_VERSION });
      this.pushTelemetry();
      this.telemetryTimer = setInterval(() => this.pushTelemetry(), TELEMETRY_INTERVAL_MS);
    };

    socket.onmessage = (event) => {
      void this.handleCommand(event.data);
    };

    socket.onerror = () => {
      // onclose always follows; reconnect is handled there so it happens once.
    };

    socket.onclose = (event) => {
      if (this.telemetryTimer) {
        clearInterval(this.telemetryTimer);
        this.telemetryTimer = null;
      }
      this.socket = null;
      if (!this.enabled) return;

      if (FATAL_CLOSE_CODES.has(event.code)) {
        this.enabled = false;
        this.options.onLinkStateChange?.(
          'error',
          event.reason || 'The bridge rejected this token. Check it against the broker log.'
        );
        return;
      }

      this.scheduleReconnect(event.reason || `connection closed (${event.code})`);
    };
  }

  private scheduleReconnect(detail: string): void {
    if (!this.enabled) return;
    const delay = RECONNECT_DELAYS_MS[Math.min(this.reconnectAttempt, RECONNECT_DELAYS_MS.length - 1)];
    this.reconnectAttempt += 1;
    this.options.onLinkStateChange?.('reconnecting', detail);
    this.reconnectTimer = setTimeout(() => this.openSocket(), delay);
  }

  private send(payload: unknown): void {
    if (this.socket?.readyState !== WebSocket.OPEN) return;
    this.socket.send(JSON.stringify(payload));
  }

  private pushTelemetry(): void {
    this.send({ type: 'telemetry', state: this.executor.buildTelemetry() });
  }

  private async handleCommand(raw: unknown): Promise<void> {
    let command: BridgeCommand;
    try {
      command = JSON.parse(String(raw));
    } catch {
      return;
    }
    if (typeof command?.id !== 'string') return;

    try {
      const result = await this.executor.run(command.type, command.params);
      this.send({ v: BRIDGE_PROTOCOL_VERSION, id: command.id, ok: true, result });
      this.options.onActivity?.({
        at: Date.now(),
        type: command.type,
        ok: true,
        summary: describeResult(command, result)
      });
    } catch (error) {
      const bridgeError: BridgeError =
        error instanceof BridgeCommandError
          ? { code: error.code, message: error.message, notes: error.notes }
          : { code: 'EXECUTOR_ERROR', message: error instanceof Error ? error.message : String(error) };

      this.send({ v: BRIDGE_PROTOCOL_VERSION, id: command.id, ok: false, error: bridgeError });
      this.options.onActivity?.({
        at: Date.now(),
        type: command.type,
        ok: false,
        summary: bridgeError.message
      });
    } finally {
      this.pushTelemetry();
    }
  }
}

const describeResult = (command: BridgeCommand, result: unknown): string => {
  const r = result as { toMm?: { x: number; y: number; z: number }; outcome?: string };
  if (command.type === 'preview_move' && r?.toMm) {
    return `preview → (${r.toMm.x}, ${r.toMm.y}, ${r.toMm.z}) mm`;
  }
  if (command.type === 'move_relative' || command.type === 'move_to') {
    return `${r?.outcome ?? 'done'}`;
  }
  if (command.type === 'stop') return 'emergency stop sent';
  return 'status read';
};
