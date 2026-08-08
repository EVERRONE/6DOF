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
import { BridgeTelemetry, MoveToParams, PreviewMoveParams } from './bridgeProtocol';

/**
 * Everything the bridge does to the running app.
 *
 * Split out from the WebSocket client so the decisions that matter — may this
 * move happen, and did it actually arrive — can be tested without a socket.
 * Doing so immediately surfaced a bug: an emergency stop leaves `robotState` at
 * ESTOPPED indefinitely, which made every later move report failure while the
 * arm was in fact moving.
 *
 * This class does no kinematics beyond FK for the current pose, speaks no
 * serial, and builds no trajectories. It calls the same store actions the UI
 * buttons call.
 */

/** Executor-side displacement ceiling, in mm. The copy closest to the hardware. */
export const MAX_DISPLACEMENT_MM = 150;

/**
 * How long to wait for a move before giving up on knowing the outcome.
 * Planning alone can take 20 s, then the arm has to travel. On timeout the
 * answer is "unknown" — a move still running must never be reported as arrived.
 */
export const MOVE_SETTLE_TIMEOUT_MS = 90_000;

export type MoveOutcome =
  | { status: 'arrived' }
  | { status: 'failed'; error: string; notes: string[] }
  | { status: 'stopped' }
  | { status: 'unknown'; reason: string };

export class BridgeCommandError extends Error {
  code: string;
  notes?: string[];

  constructor(code: string, message: string, notes?: string[]) {
    super(message);
    this.name = 'BridgeCommandError';
    this.code = code;
    this.notes = notes;
  }
}

const toMm = (metres: number): number => Math.round(metres * 10000) / 10;

type StoreState = ReturnType<typeof useRobotStore.getState>;

export class AgentCommandExecutor {
  /** Arming lives here because it is the human's decision, made at the machine. */
  private armedUntil: number | null = null;

  /** Heading of the operator's "forward", in base-frame degrees. */
  private viewYawDeg = 0;

  private settleTimeoutMs: number;

  constructor(options: { settleTimeoutMs?: number } = {}) {
    this.settleTimeoutMs = options.settleTimeoutMs ?? MOVE_SETTLE_TIMEOUT_MS;
  }

  isArmed(): boolean {
    return this.armedUntil !== null && this.armedUntil > Date.now();
  }

  getArmedUntil(): number | null {
    return this.isArmed() ? this.armedUntil : null;
  }

  arm(durationMinutes: number): void {
    this.armedUntil = Date.now() + durationMinutes * 60_000;
  }

  disarm(): void {
    this.armedUntil = null;
  }

  setViewYawDeg(yawDeg: number): void {
    this.viewYawDeg = Number.isFinite(yawDeg) ? yawDeg : 0;
  }

  getViewYawDeg(): number {
    return this.viewYawDeg;
  }

  buildTelemetry(): BridgeTelemetry {
    const state = useRobotStore.getState();
    return {
      armed: this.isArmed(),
      armedUntil: this.getArmedUntil(),
      connected: state.connectionStatus === ConnectionStatus.CONNECTED,
      motorsEnabled: state.motorsEnabled,
      cartesianReady: state.isCartesianReady(),
      robotState: state.robotState,
      planningState: state.planningState,
      busy: this.isBusy(state)
    };
  }

  async run(type: string, params: unknown): Promise<unknown> {
    switch (type) {
      case 'get_status':
        return this.buildStatus();
      case 'preview_move':
        return this.previewMove(params as PreviewMoveParams);
      case 'move_relative':
        return this.moveRelative(params as PreviewMoveParams);
      case 'move_to':
        return this.moveToAbsolute(params as MoveToParams);
      case 'stop':
        return this.stopMotion();
      default:
        throw new BridgeCommandError('UNKNOWN_COMMAND', `This app does not implement "${type}".`);
    }
  }

  private isBusy(state: StoreState): boolean {
    return (
      state.moveInProgress ||
      state.planningState === 'stage1_fast' ||
      state.planningState === 'stage2_refine'
    );
  }

  /**
   * Full tool pose from a single FK traversal. The store exposes position but
   * not rotation, and a pose-locked move needs both to agree.
   */
  computeCurrentPose(): { position: Vector3; rotation: Rotation3 } | null {
    const state = useRobotStore.getState();
    const config = state.firmwareConfig;
    if (!config || !state.kinematicsFrameReady) return null;

    const offsets = getEffectiveUrdfOffsets(config);
    const a = state.currentAngles;
    const fk = ForwardKinematics.solve(
      logicalToUrdfAngles([a.J1, a.J2, a.J3, a.J4, a.J5, a.J6], offsets)
    );
    if (!fk.success) return null;

    return { position: fk.endEffectorPose.position, rotation: fk.endEffectorPose.rotation };
  }

  /** Preconditions in plain words — an agent cannot recover from what it cannot see. */
  describeReadiness(state: StoreState = useRobotStore.getState()): string {
    if (state.connectionStatus !== ConnectionStatus.CONNECTED) return 'Not connected to the robot.';
    if (state.robotState === RobotState.ESTOPPED) {
      return 'Emergency stop is latched. Clear it by re-homing the arm from the control app.';
    }
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

  buildStatus(): unknown {
    const state = useRobotStore.getState();
    const pose = this.computeCurrentPose();

    return {
      armed: this.isArmed(),
      armedUntil: this.getArmedUntil() ? new Date(this.getArmedUntil()!).toISOString() : null,
      connected: state.connectionStatus === ConnectionStatus.CONNECTED,
      motorsEnabled: state.motorsEnabled,
      emergencyStopLatched: state.robotState === RobotState.ESTOPPED,
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
      busy: this.isBusy(state),
      viewYawDeg: this.viewYawDeg,
      tcpPositionMm: pose
        ? { x: toMm(pose.position.x), y: toMm(pose.position.y), z: toMm(pose.position.z) }
        : null,
      jointAnglesDeg: { ...state.currentAngles },
      lastError: state.ikStatus?.success === false ? state.ikStatus.error ?? null : null,
      planningNotes: state.planningNotes.slice(0, 6)
    };
  }

  /**
   * Dry run: resolve the direction and report where the tool would end up,
   * without touching the store. The cheapest way to catch a wrong frame.
   */
  previewMove(params: PreviewMoveParams): unknown {
    const state = useRobotStore.getState();
    const pose = this.computeCurrentPose();

    if (!pose) {
      throw new BridgeCommandError(
        'NO_POSE',
        'The current tool position is not known yet. Connect to the robot and wait for Cartesian frame sync.'
      );
    }

    const { target, unitVector, deltaMm } = applyDirectionalOffset(
      pose.position,
      params.direction,
      params.distanceMm,
      params.frame,
      this.viewYawDeg
    );

    const atlas = isReachabilityAtlasReady() ? getDefaultReachabilityAtlas() : null;
    const reachability = atlas ? classifyReachability(target, atlas) : null;

    return {
      resolved: this.describeResolution(params, unitVector, deltaMm),
      distanceMm: params.distanceMm,
      ...(params.clamped
        ? {
            clampedFrom: params.requestedDistanceMm,
            note: `Distance clamped to the ${params.distanceMm} mm per-command limit.`
          }
        : {}),
      fromMm: { x: toMm(pose.position.x), y: toMm(pose.position.y), z: toMm(pose.position.z) },
      toMm: { x: toMm(target.x), y: toMm(target.y), z: toMm(target.z) },
      reachability: reachability
        ? { likelyReachable: reachability.likelyReachable, reason: reachability.reason }
        : { likelyReachable: null, reason: 'atlas_unavailable' },
      readyToMove: this.describeReadiness(state),
      executed: false,
      note: 'Preview only — nothing was sent to the robot.'
    };
  }

  private describeResolution(
    params: PreviewMoveParams,
    unitVector: Vector3,
    deltaMm: Vector3
  ): Record<string, unknown> {
    return {
      direction: params.direction,
      frame: params.frame,
      viewYawDeg: params.frame === 'view' ? this.viewYawDeg : 0,
      unitVector,
      worldDeltaMm: deltaMm
    };
  }

  /**
   * Refuses for reasons the agent can act on, before anything is committed.
   *
   * The ESTOPPED check is not decorative: moveToPosition does not itself
   * refuse while an emergency stop is latched — it would set MOVING and drive
   * the arm — so this is the only thing standing in the way.
   */
  assertCanMove(): void {
    const state = useRobotStore.getState();

    if (!this.isArmed()) {
      throw new BridgeCommandError(
        'NOT_ARMED',
        'AI control is not armed. Someone has to arm it in the Agent Control panel of the robot app, at the machine.'
      );
    }

    if (state.robotState === RobotState.ESTOPPED) {
      throw new BridgeCommandError(
        'ESTOP_LATCHED',
        'An emergency stop is still latched. Clear it by re-homing the arm from the control app before moving again.'
      );
    }

    if (this.isBusy(state)) {
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

  /**
   * Watches the store for the real outcome of a move.
   *
   * `moveToPosition` cannot report either thing we need: its promise resolves
   * when `TQ RUN` is acknowledged rather than on arrival, and it never throws —
   * failures are reported by setting `planningState: 'failed'`.
   *
   * Must be started *before* `moveToPosition` is called. Note that ESTOPPED is
   * only treated as an interruption on the *transition* into it; the state is
   * latched, so testing the level would fire instantly on a stale stop.
   */
  watchMoveOutcome(timeoutMs: number = this.settleTimeoutMs): Promise<MoveOutcome> {
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

      unsubscribe = useRobotStore.subscribe((state: StoreState, prev: StoreState) => {
        if (state.robotState === RobotState.ESTOPPED && prev.robotState !== RobotState.ESTOPPED) {
          finish({ status: 'stopped' });
          return;
        }

        if (state.planningState === 'failed' && prev.planningState !== 'failed') {
          finish({
            status: 'failed',
            error: state.ikStatus?.error ?? 'Cartesian planning failed',
            notes: state.planningNotes.slice(0, 8)
          });
          return;
        }

        if (state.moveInProgress) {
          sawMotion = true;
          return;
        }

        // TQ_DONE clears moveInProgress and returns robotState to IDLE in one
        // update — that is the arrival signal.
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
    const watcher = wait ? this.watchMoveOutcome() : null;

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
    const common = {
      ...base,
      actualToMm: settledPose
        ? {
            x: toMm(settledPose.position.x),
            y: toMm(settledPose.position.y),
            z: toMm(settledPose.position.z)
          }
        : null,
      durationMs: Date.now() - startedAt
    };

    switch (outcome.status) {
      case 'arrived':
        return { ...common, outcome: 'arrived' };
      case 'failed':
        // Planner messages are written for humans and say what to do next;
        // passing them through beats inventing a code.
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
  async moveRelative(params: PreviewMoveParams): Promise<unknown> {
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
      this.viewYawDeg
    );

    return this.executeMove(target, pose, params.keepOrientation !== false, params.wait !== false, {
      resolved: this.describeResolution(params, unitVector, deltaMm),
      distanceMm,
      ...(params.clamped ? { clampedFrom: params.requestedDistanceMm } : {})
    });
  }

  /** Straight-line move to an absolute point, in mm from the base. */
  async moveToAbsolute(params: MoveToParams): Promise<unknown> {
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

    // The ceiling is about displacement, so it can only be checked here — the
    // broker cannot know how far away an absolute point is.
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
  async stopMotion(): Promise<unknown> {
    await useRobotStore.getState().emergencyStop();
    // Whatever the agent was doing, a human almost certainly wants to approve
    // again before it resumes.
    this.disarm();
    return {
      stopped: true,
      armed: false,
      note: 'Emergency stop sent. AI control has been disarmed, and the stop stays latched until the arm is re-homed.'
    };
  }
}
