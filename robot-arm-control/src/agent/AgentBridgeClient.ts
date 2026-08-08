import { useRobotStore } from '../store/robotStore';
import { ConnectionStatus, RobotState } from '../types/robot';
import {
  classifyReachability,
  getDefaultReachabilityAtlas,
  isReachabilityAtlasReady
} from '../kinematics/reachabilityAtlas';
import { ForwardKinematics } from '../kinematics/ForwardKinematics';
import { getEffectiveUrdfOffsets, logicalToUrdfAngles } from '../kinematics/angleMapping';
import { Rotation3, Vector3 } from '../kinematics/types';
import { applyDirectionalOffset } from './frames';
import {
  BRIDGE_PROTOCOL_VERSION,
  BridgeCommand,
  BridgeError,
  BridgeLinkState,
  BridgeTelemetry,
  MoveToParams,
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

/**
 * Executor-side displacement ceiling, in millimetres.
 *
 * The broker clamps too, but this is the copy that matters: it is the one
 * closest to the hardware and the one that still applies if anything ever
 * speaks to the executor directly.
 */
const MAX_DISPLACEMENT_MM = 150;

/**
 * How long to wait for a move to finish before giving up on knowing.
 *
 * Planning alone can take 20 s, and then the arm has to travel. On timeout we
 * report "unknown", never "done" — a move that is still running must not be
 * reported as arrived.
 */
const MOVE_SETTLE_TIMEOUT_MS = 90_000;

type MoveOutcome =
  | { status: 'arrived' }
  | { status: 'failed'; error: string; notes: string[] }
  | { status: 'stopped' }
  | { status: 'unknown'; reason: string };

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
      case 'move_relative':
        return this.moveRelative(command.params as unknown as PreviewMoveParams);
      case 'move_to':
        return this.moveToAbsolute(command.params as unknown as MoveToParams);
      case 'stop':
        return this.stopMotion();
      default:
        throw new BridgeCommandError(
          'UNKNOWN_COMMAND',
          `This app does not implement "${command.type}".`
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

  /**
   * Full tool pose from a single FK traversal.
   *
   * The store exposes `currentPosition` but not the rotation, and a pose-locked
   * move needs both — taking them from one traversal keeps them consistent.
   */
  private computeCurrentPose(): { position: Vector3; rotation: Rotation3 } | null {
    const state = useRobotStore.getState();
    const config = state.firmwareConfig;
    if (!config || !state.kinematicsFrameReady) return null;

    const offsets = getEffectiveUrdfOffsets(config);
    const a = state.currentAngles;
    const fk = ForwardKinematics.solve(
      logicalToUrdfAngles([a.J1, a.J2, a.J3, a.J4, a.J5, a.J6], offsets)
    );
    if (!fk.success) return null;

    return {
      position: fk.endEffectorPose.position,
      rotation: fk.endEffectorPose.rotation
    };
  }

  /**
   * Watches the store for the real outcome of a move.
   *
   * This exists because `moveToPosition` cannot tell you either thing you need:
   * its promise resolves when `TQ RUN` is acknowledged rather than when the arm
   * arrives, and it never throws — every failure is reported by setting
   * `planningState: 'failed'`. So the outcome has to be observed, not awaited.
   *
   * Must be started *before* `moveToPosition` is called, or the early
   * transitions are missed.
   */
  private watchMoveOutcome(timeoutMs: number): Promise<MoveOutcome> {
    return new Promise((resolve) => {
      let settled = false;
      let sawMotion = false;
      let unsubscribe: (() => void) | null = null;
      let timer: ReturnType<typeof setTimeout> | null = null;

      const finish = (outcome: MoveOutcome): void => {
        if (settled) return;
        settled = true;
        if (timer) clearTimeout(timer);
        unsubscribe?.();
        resolve(outcome);
      };

      unsubscribe = useRobotStore.subscribe((state) => {
        if (state.planningState === 'failed') {
          finish({
            status: 'failed',
            error: state.ikStatus?.error ?? 'Cartesian planning failed',
            notes: state.planningNotes.slice(0, 8)
          });
          return;
        }

        if (state.robotState === RobotState.ESTOPPED) {
          finish({ status: 'stopped' });
          return;
        }

        if (state.moveInProgress) {
          sawMotion = true;
          return;
        }

        // TQ_DONE clears moveInProgress and returns robotState to IDLE in one
        // update, so this is the arrival signal.
        if (sawMotion && state.robotState === RobotState.IDLE) {
          finish({ status: 'arrived' });
        }
      });

      timer = setTimeout(
        () =>
          finish({
            status: 'unknown',
            reason: `No completion signal within ${timeoutMs} ms. The arm may still be moving.`
          }),
        timeoutMs
      );
    });
  }

  /** Refuses for reasons the agent can act on, before anything is committed. */
  private assertCanMove(): void {
    const state = useRobotStore.getState();

    if (!this.isArmed()) {
      throw new BridgeCommandError(
        'NOT_ARMED',
        'AI control is not armed. Someone has to arm it in the Agent Control panel of the robot app, at the machine.'
      );
    }

    if (state.moveInProgress || state.planningState === 'stage1_fast' || state.planningState === 'stage2_refine') {
      throw new BridgeCommandError(
        'BUSY',
        'The arm is already executing a move. Wait for it to finish, or call stop first.'
      );
    }

    if (!state.isCartesianReady()) {
      throw new BridgeCommandError('NOT_READY', this.describeReadiness(state));
    }

    if (!state.firmwareConfig?.capabilities?.trajectoryQueue) {
      throw new BridgeCommandError(
        'NO_QUEUE',
        'This firmware does not support the trajectory queue (TQ), which straight-line moves require.'
      );
    }
  }

  private async executeMove(
    targetM: Vector3,
    fromPose: { position: Vector3; rotation: Rotation3 },
    keepOrientation: boolean,
    wait: boolean,
    extra: Record<string, unknown>
  ): Promise<unknown> {
    const store = useRobotStore.getState();
    const startedAt = Date.now();

    // Subscribe before commanding, or the first transitions are missed.
    const watcher = wait ? this.watchMoveOutcome(MOVE_SETTLE_TIMEOUT_MS) : null;

    // Passing the current rotation forces pose_lock regardless of the app's
    // Cartesian mode: "move sideways without tilting the tool".
    const movePromise = store.moveToPosition(
      targetM,
      keepOrientation ? { ...fromPose.rotation } : undefined
    );

    const base = {
      ...extra,
      keepOrientation,
      cartesianMode: keepOrientation ? 'pose_lock' : useRobotStore.getState().cartesianMode,
      fromMm: {
        x: toMm(fromPose.position.x),
        y: toMm(fromPose.position.y),
        z: toMm(fromPose.position.z)
      },
      requestedToMm: { x: toMm(targetM.x), y: toMm(targetM.y), z: toMm(targetM.z) }
    };

    if (!wait) {
      await movePromise;
      const after = useRobotStore.getState();
      if (after.planningState === 'failed') {
        throw new BridgeCommandError(
          'PLANNING_FAILED',
          after.ikStatus?.error ?? 'Cartesian planning failed',
          after.planningNotes.slice(0, 8)
        );
      }
      return { ...base, outcome: 'started', note: 'Queue is running. Poll get_status for arrival.' };
    }

    const outcome = await watcher!;
    await movePromise.catch(() => undefined);

    const settledPose = this.computeCurrentPose();
    const actualToMm = settledPose
      ? {
          x: toMm(settledPose.position.x),
          y: toMm(settledPose.position.y),
          z: toMm(settledPose.position.z)
        }
      : null;

    const common = { ...base, actualToMm, durationMs: Date.now() - startedAt };

    switch (outcome.status) {
      case 'arrived':
        return { ...common, outcome: 'arrived' };
      case 'failed':
        // The planner's messages are written for humans and say what to do
        // next; passing them through beats inventing a code.
        throw new BridgeCommandError('PLANNING_FAILED', outcome.error, outcome.notes);
      case 'stopped':
        throw new BridgeCommandError(
          'STOPPED',
          'The move was interrupted by an emergency stop before it finished.'
        );
      default:
        throw new BridgeCommandError('UNKNOWN_OUTCOME', outcome.reason);
    }
  }

  /** Straight-line relative move, via the queue-based Cartesian path. */
  private async moveRelative(params: PreviewMoveParams): Promise<unknown> {
    this.assertCanMove();

    const pose = this.computeCurrentPose();
    if (!pose) {
      throw new BridgeCommandError(
        'NO_POSE',
        'The current tool position is not known yet. Wait for Cartesian frame sync.'
      );
    }

    const distanceMm = Math.min(params.distanceMm, MAX_DISPLACEMENT_MM);
    const { target, unitVector, deltaMm } = applyDirectionalOffset(
      pose.position,
      params.direction,
      distanceMm,
      params.frame,
      0
    );

    return this.executeMove(target, pose, params.keepOrientation !== false, params.wait !== false, {
      resolved: {
        direction: params.direction,
        frame: params.frame,
        viewYawDeg: 0,
        unitVector,
        worldDeltaMm: deltaMm
      },
      distanceMm,
      ...(params.clamped ? { clampedFrom: params.requestedDistanceMm } : {})
    });
  }

  /** Straight-line move to an absolute point, in millimetres from the base. */
  private async moveToAbsolute(params: {
    targetMm: { x: number; y: number; z: number };
    keepOrientation: boolean;
    wait: boolean;
  }): Promise<unknown> {
    this.assertCanMove();

    const pose = this.computeCurrentPose();
    if (!pose) {
      throw new BridgeCommandError(
        'NO_POSE',
        'The current tool position is not known yet. Wait for Cartesian frame sync.'
      );
    }

    const target: Vector3 = {
      x: params.targetMm.x / 1000,
      y: params.targetMm.y / 1000,
      z: params.targetMm.z / 1000
    };

    // The per-command ceiling is about displacement, so it has to be checked
    // here — the broker cannot know how far away an absolute point is.
    const distanceMm = Math.hypot(
      (target.x - pose.position.x) * 1000,
      (target.y - pose.position.y) * 1000,
      (target.z - pose.position.z) * 1000
    );

    if (distanceMm > MAX_DISPLACEMENT_MM) {
      throw new BridgeCommandError(
        'TOO_FAR',
        `That point is ${distanceMm.toFixed(0)} mm away, over the ${MAX_DISPLACEMENT_MM} mm per-command limit. Move there in steps.`
      );
    }

    return this.executeMove(target, pose, params.keepOrientation, params.wait, {
      distanceMm: Math.round(distanceMm * 10) / 10
    });
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
