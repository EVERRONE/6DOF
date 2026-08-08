import { useRobotStore } from '../store/robotStore';
import { ConnectionStatus } from '../types/robot';
import {
  classifyReachability,
  getDefaultReachabilityAtlas,
  isReachabilityAtlasReady
} from '../kinematics/reachabilityAtlas';
import { applyDirectionalOffset } from './frames';
import {
  BRIDGE_PROTOCOL_VERSION,
  BridgeCommand,
  BridgeError,
  BridgeLinkState,
  BridgeTelemetry,
  PreviewMoveParams
} from './bridgeProtocol';

/**
 * Executes bridge commands against the running app.
 *
 * This is deliberately thin. It does no kinematics, speaks no serial, and knows
 * nothing about trajectories — it reads the store, resolves a direction into a
 * vector, and (from phase 3 onward) calls the same store action the UI buttons
 * call. Everything that makes the arm move correctly already exists.
 *
 * The client dials *out* to the broker, so the machine holding the serial port
 * needs no inbound port and no stable address.
 */

const RECONNECT_DELAYS_MS = [1000, 2000, 4000, 8000, 15000];
const TELEMETRY_INTERVAL_MS = 1000;

export interface AgentBridgeClientOptions {
  url: string;
  token: string;
  onLinkStateChange?: (state: BridgeLinkState, detail?: string) => void;
  onActivity?: (entry: AgentActivityEntry) => void;
}

export interface AgentActivityEntry {
  at: number;
  type: string;
  ok: boolean;
  summary: string;
}

const toMm = (metres: number): number => Math.round(metres * 10000) / 10;

export class AgentBridgeClient {
  private socket: WebSocket | null = null;
  private options: AgentBridgeClientOptions;
  private enabled = false;
  private reconnectAttempt = 0;
  private reconnectTimer: ReturnType<typeof setTimeout> | null = null;
  private telemetryTimer: ReturnType<typeof setInterval> | null = null;

  /** Arming lives here, in the executor, because it is the human's decision at the machine. */
  private armedUntil: number | null = null;

  constructor(options: AgentBridgeClientOptions) {
    this.options = options;
  }

  isArmed(): boolean {
    return this.armedUntil !== null && this.armedUntil > Date.now();
  }

  getArmedUntil(): number | null {
    return this.isArmed() ? this.armedUntil : null;
  }

  arm(durationMinutes: number): void {
    this.armedUntil = Date.now() + durationMinutes * 60_000;
    this.pushTelemetry();
  }

  disarm(): void {
    this.armedUntil = null;
    this.pushTelemetry();
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
      socket = new WebSocket(`${this.options.url}?token=${encodeURIComponent(this.options.token)}`);
    } catch (error) {
      this.scheduleReconnect(error instanceof Error ? error.message : String(error));
      return;
    }

    this.socket = socket;

    socket.onopen = () => {
      this.reconnectAttempt = 0;
      this.options.onLinkStateChange?.('connected');
      this.send({
        type: 'hello',
        app: 'robot-arm-control',
        version: BRIDGE_PROTOCOL_VERSION
      });
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
      this.scheduleReconnect(event.reason || `closed (${event.code})`);
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
    const state = useRobotStore.getState();
    const telemetry: BridgeTelemetry = {
      armed: this.isArmed(),
      armedUntil: this.getArmedUntil(),
      connected: state.connectionStatus === ConnectionStatus.CONNECTED,
      motorsEnabled: state.motorsEnabled,
      cartesianReady: state.isCartesianReady(),
      robotState: state.robotState,
      planningState: state.planningState,
      busy: state.moveInProgress || state.planningState === 'stage1_fast' || state.planningState === 'stage2_refine'
    };
    this.send({ type: 'telemetry', state: telemetry });
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
      const result = await this.runCommand(command);
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

  private async runCommand(command: BridgeCommand): Promise<unknown> {
    switch (command.type) {
      case 'get_status':
        return this.buildStatus();
      case 'preview_move':
        return this.previewMove(command.params as unknown as PreviewMoveParams);
      case 'stop':
        return this.stopMotion();
      default:
        throw new BridgeCommandError(
          'UNKNOWN_COMMAND',
          `This app does not implement "${command.type}". Motion commands arrive in a later phase.`
        );
    }
  }

  private buildStatus(): unknown {
    const state = useRobotStore.getState();
    const position = state.currentPosition;

    return {
      armed: this.isArmed(),
      armedUntil: this.getArmedUntil() ? new Date(this.getArmedUntil()!).toISOString() : null,
      connected: state.connectionStatus === ConnectionStatus.CONNECTED,
      motorsEnabled: state.motorsEnabled,
      homed: {
        J2: state.homedJoints.J2,
        J3: state.homedJoints.J3,
        J4: state.homedJoints.J4,
        J5: state.homedJoints.J5
      },
      cartesianReady: state.isCartesianReady(),
      readyToMove: this.describeReadiness(state),
      robotState: state.robotState,
      planningState: state.planningState,
      busy: state.moveInProgress,
      tcpPositionMm: position
        ? { x: toMm(position.x), y: toMm(position.y), z: toMm(position.z) }
        : null,
      jointAnglesDeg: { ...state.currentAngles },
      lastError: state.ikStatus?.success === false ? state.ikStatus.error ?? null : null,
      planningNotes: state.planningNotes.slice(0, 6)
    };
  }

  /** Preconditions in words, because an agent cannot recover from what it cannot see. */
  private describeReadiness(state: ReturnType<typeof useRobotStore.getState>): string {
    if (state.connectionStatus !== ConnectionStatus.CONNECTED) return 'Not connected to the robot.';
    if (!state.kinematicsFrameReady) return 'Waiting for Cartesian frame sync.';
    if (!state.motorsEnabled) return 'Motors are disabled.';
    if (!state.homedJoints.J2 || !state.homedJoints.J3 || !state.homedJoints.J4 || !state.homedJoints.J5) {
      return 'J2..J5 must be homed first.';
    }
    if (!state.firmwareConfig?.capabilities?.trajectoryQueue) {
      return 'Firmware does not support the trajectory queue (TQ).';
    }
    if (!this.isArmed()) return 'Ready, but AI control is not armed.';
    return 'Ready.';
  }

  /**
   * Dry run. Resolves the direction and reports where the tool would end up,
   * without touching the store or the robot. This is the cheapest way to catch
   * a wrong frame — the alternative is watching the arm move the wrong way.
   */
  private previewMove(params: PreviewMoveParams): unknown {
    const state = useRobotStore.getState();
    const position = state.currentPosition;

    if (!position || !state.kinematicsFrameReady) {
      throw new BridgeCommandError(
        'NO_POSE',
        'The current tool position is not known yet. Connect to the robot and wait for Cartesian frame sync.'
      );
    }

    const { target, unitVector, deltaMm } = applyDirectionalOffset(
      position,
      params.direction,
      params.distanceMm,
      params.frame,
      0
    );

    const atlas = isReachabilityAtlasReady() ? getDefaultReachabilityAtlas() : null;
    const reachability = atlas ? classifyReachability(target, atlas) : null;

    return {
      wouldMove: true,
      resolved: {
        direction: params.direction,
        frame: params.frame,
        viewYawDeg: 0,
        unitVector,
        worldDeltaMm: deltaMm
      },
      distanceMm: params.distanceMm,
      ...(params.clamped
        ? { clampedFrom: params.requestedDistanceMm, note: `Distance clamped to the ${params.distanceMm} mm per-command limit.` }
        : {}),
      fromMm: { x: toMm(position.x), y: toMm(position.y), z: toMm(position.z) },
      toMm: { x: toMm(target.x), y: toMm(target.y), z: toMm(target.z) },
      reachability: reachability
        ? { likelyReachable: reachability.likelyReachable, reason: reachability.reason }
        : { likelyReachable: null, reason: 'atlas_unavailable' },
      readyToMove: this.describeReadiness(state),
      executed: false,
      note: 'Preview only — nothing was sent to the robot.'
    };
  }

  /** Privileged: works whether or not AI control is armed. */
  private async stopMotion(): Promise<unknown> {
    await useRobotStore.getState().emergencyStop();
    // A stop is also a good moment to revoke standing permission: whatever the
    // agent was doing, a human almost certainly wants to re-approve first.
    this.disarm();
    return {
      stopped: true,
      armed: false,
      note: 'Emergency stop sent. AI control has been disarmed and must be re-armed at the machine.'
    };
  }
}

class BridgeCommandError extends Error {
  code: string;
  notes?: string[];

  constructor(code: string, message: string, notes?: string[]) {
    super(message);
    this.name = 'BridgeCommandError';
    this.code = code;
    this.notes = notes;
  }
}

const describeResult = (command: BridgeCommand, result: unknown): string => {
  if (command.type === 'preview_move') {
    const r = result as { toMm?: { x: number; y: number; z: number } };
    return r?.toMm ? `preview → (${r.toMm.x}, ${r.toMm.y}, ${r.toMm.z}) mm` : 'preview';
  }
  if (command.type === 'stop') return 'emergency stop sent';
  return 'status read';
};
