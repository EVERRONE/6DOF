import { create } from 'zustand';
import { JointAngles, EndstopState, ConnectionStatus, RobotState } from '../types/robot';
import { SerialManager } from '../communication/SerialManager';
import { AxisLimits, FirmwareState, FirmwareStatus, IOState } from '../communication/types';
import { Vector3, Rotation3, IKResult } from '../kinematics/types';
import { ForwardKinematics } from '../kinematics/ForwardKinematics';
import { InverseKinematics } from '../kinematics/InverseKinematics';
import { checkSelfCollision, firstCollidingPoint } from '../kinematics/CollisionChecker';
import {
  getRotation,
  getTranslation,
  matrixToRpy,
  multiply3,
  rotationAboutAxis,
  rpyToMatrix
} from '../kinematics/linalg';
import {
  Waypoint,
  Trajectory,
  ExecutionState,
  ExecutionProgress,
  PathPlannerConfig,
  SavedPath
} from '../motion/types';
import { TrajectoryPlanner, DEFAULT_PLANNER_CONFIG } from '../motion/TrajectoryPlanner';
import { PathInterpolator } from '../motion/PathInterpolator';
import { makeCircleWaypoint, validateCircle } from '../motion/Shapes';
import { ShapePlane, WaypointShape } from '../motion/types';
import {
  HOME_POSE_DEG,
  JOINT_LIMITS_DEG,
  NUM_JOINTS,
  ToolFrame,
  clampToLimitsDeg,
  degToRad,
  getToolFrame,
  resetToolFrame as resetModelToolFrame,
  setToolFrame as setModelToolFrame
} from '../kinematics/robotModel';
import { loadToolFrame, saveToolFrame } from './toolFrameStorage';
import { ToolCalibration, ToolTouch, solveToolOffset } from '../kinematics/toolCalibration';
import { loadWorkObjects, saveWorkObjects } from './workObjectStorage';
import {
  BASE_FRAME,
  WorkObject,
  danglingFrames,
  frameById,
  pointFromBase,
  pointToBase,
  rotationFromBase,
  teachFromThreePoints
} from '../kinematics/workObject';

/** One line for the on-screen event log. */
export interface RobotEvent {
  id: number;
  at: number;
  kind: 'sent' | 'ok' | 'error' | 'info';
  text: string;
}

/** How many events to keep. Enough to scroll back through a homing run. */
const MAX_EVENTS = 200;
let nextEventId = 1;

interface RobotStore {
  // Connection
  connectionStatus: ConnectionStatus;
  /** Why the link is down, or what the manager is doing about it. */
  connectionDetail: string | null;
  serialManager: SerialManager | null;

  // Robot state
  robotState: RobotState;
  currentAngles: JointAngles;
  targetAngles: JointAngles;
  endstopState: EndstopState;
  motorsEnabled: boolean;

  /**
   * Everything the arm said, and everything we said to it. The firmware's replies
   * used to go only to console.error, which hid "Endstop triggered during move"
   * from the one person who needs to see it.
   */
  events: RobotEvent[];

  /** Last STATUS line from the firmware: queue space, homed flags, trust. */
  firmwareStatus: FirmwareStatus | null;
  /** When that STATUS arrived, used to tell a fresh report from a stale one. */
  firmwareStatusAt: number;

  // Kinematics state
  currentPosition: Vector3 | null;
  /** Current tool orientation, as rpy. Needed to hold it while moving. */
  currentRotation: Rotation3 | null;
  /**
   * Whether Cartesian moves must keep the tool pointing the way it is.
   *
   * Off, a Cartesian move solves for position only: three equations, six
   * unknowns, so the orientation is free and the tool tips as the arm reaches -
   * 8 degrees for a 20 mm move in Z from the parked pose, 21 degrees for 50 mm.
   * Fine for getting somewhere, useless for carrying a pen or a gripper.
   */
  toolLocked: boolean;
  /**
   * The orientation to hold, captured when the lock is switched on rather than
   * read at each move. Re-reading it every move would let small residuals
   * accumulate, and the tool would ratchet away from where it started.
   */
  lockedRotation: Rotation3 | null;
  targetPosition: Vector3 | null;
  ikStatus: IKResult | null;

  /**
   * How a Cartesian move gets from here to there.
   *
   * `linear` solves IK along the straight line and streams the samples, so the
   * tool travels the line it was shown and holds its orientation the whole way.
   * `joint` solves the destination once and lets every joint run proportionally
   * to it - faster, always reachable if the destination is, and curved in
   * Cartesian space with the tool tipping several degrees on the way.
   *
   * Linear by default: a target typed into a Cartesian panel is a statement
   * about where the tool should be, and a path that bows 7.7 mm off it is a
   * surprise. Joint is there for repositioning in free space, and as the escape
   * when the line itself is unreachable.
   */
  cartesianMode: 'linear' | 'joint';
  /** Tool speed along a linear move, mm/s. */
  cartesianSpeed: number;
  /** Tool acceleration along a linear move, mm/s^2. */
  cartesianAccel: number;

  /**
   * Where the tool is relative to the flange. Mirrors the kinematic model's own
   * copy, which is what FK and IK read; held here so the UI re-renders when it
   * changes.
   */
  toolFrame: ToolFrame;

  /** Digital I/O as the firmware last reported it, or null before it has. */
  ioState: IOState | null;
  /**
   * The names the firmware gave its I/O points.
   *
   * Asked for rather than hardcoded. The pin a gripper is on is a fact about
   * the machine, and a host that carried its own copy would be wrong the first
   * time somebody moved a wire and right about nothing afterwards.
   */
  ioNames: { outputs: string[]; inputs: string[] };

  // UI state
  manualSpeed: number;

  // Trajectory / waypoint state
  waypoints: Waypoint[];
  trajectory: Trajectory | null;
  trajectoryPositions: Vector3[];
  executionState: ExecutionState;
  executionProgress: ExecutionProgress;
  plannerConfig: PathPlannerConfig;
  /**
   * Where the planned path folds the arm into itself, or null when it is clear.
   *
   * Held rather than only logged, because execution has to refuse on it and the
   * panel has to say why. Recomputed on every plan.
   */
  trajectoryCollision: { atPercent: number; message: string } | null;

  /**
   * Motion limits as the firmware currently holds them, one entry per axis.
   *
   * Populated by asking, not assumed: these can be changed at runtime while
   * tuning, so the app has to read them back rather than mirror a constant.
   */
  axisLimits: AxisLimits[];

  // Joint space actions
  /**
   * Open the link. `manager` is a test seam: production calls this with no
   * argument and the store builds its own.
   */
  connect: (manager?: SerialManager) => Promise<void>;
  disconnect: () => Promise<void>;
  setTargetAngles: (angles: Partial<JointAngles>) => void;
  moveToTarget: () => Promise<void>;
  homeJoints: (joints: string) => Promise<void>;
  enableMotors: (enable: boolean) => Promise<void>;
  emergencyStop: () => Promise<void>;
  setManualSpeed: (speed: number) => void;

  /** Copy the arm's reported angles into the target fields. */
  syncTargetsToCurrent: () => void;
  /** Move one joint by a relative amount, from the current target. */
  jogJoint: (joint: keyof JointAngles, deltaDegrees: number) => Promise<void>;

  /**
   * Which axes a Cartesian jog moves along.
   *
   * `base`  the world. Z is up, and that is the only one of the three that
   *         means the same thing wherever the arm is standing.
   * `tool`  the tool's own axes, so "down" is into whatever it is pointing at
   *         rather than towards the floor. What you want when approaching a
   *         surface at an angle.
   * `work`  the active work object's axes, so a jog runs along the fixture's
   *         own edges. What you want when touching its three points.
   */
  jogFrame: 'base' | 'tool' | 'work';
  setJogFrame: (frame: 'base' | 'tool' | 'work') => void;
  /** Step size for a Cartesian jog, in millimetres. */
  jogStepMm: number;
  setJogStepMm: (mm: number) => void;
  /** Step size for a rotary jog, in degrees. */
  jogStepDeg: number;
  setJogStepDeg: (deg: number) => void;

  /**
   * Move the tool a fixed distance along one axis of the chosen frame, holding
   * its orientation.
   *
   * Holding the orientation is not optional here even though it costs reach. A
   * jog is a translation: "5 mm up" that also tips the tool 3 degrees is not a
   * jog, it is a surprise, and it is exactly what an unconstrained solve does -
   * measured at 8 degrees for a 20 mm move in Z from the parked pose.
   */
  jogCartesian: (axis: 'x' | 'y' | 'z', millimetres: number) => Promise<void>;
  /** The jog frame's axes as columns of a rotation matrix, in base coordinates. */
  jogAxes: () => number[][] | null;
  /** Turn the tool about one axis of the chosen frame, without moving the tip. */
  jogRotation: (axis: 'x' | 'y' | 'z', degrees: number) => Promise<void>;
  /**
   * Whether the arm may be commanded to move at all, logging why not.
   *
   * Shared rather than repeated per entry point. Jogging was added by calling
   * moveAlongLine directly and inherited none of moveToPosition's checks, which
   * is exactly how a guard that lives in one caller gets missed by the next.
   */
  canCommandMotion: (what: string) => boolean;
  /** Move to the post-homing rest pose. */
  goToHomePose: () => Promise<void>;
  /** Send a raw protocol line, for bring-up and diagnostics. */
  sendRawCommand: (line: string) => Promise<void>;
  clearEvents: () => void;
  logEvent: (kind: RobotEvent['kind'], text: string) => void;

  // Cartesian space actions
  setTargetPosition: (position: Vector3) => void;
  moveToPosition: (position: Vector3) => Promise<void>;
  /**
   * Stream the straight line to `position`, holding `orientation` at every
   * sample. Called by moveToPosition in linear mode; separate so the geometry
   * can be tested without the guards and the IK in front of it.
   */
  moveAlongLine: (position: Vector3, orientation?: Rotation3) => Promise<void>;
  /**
   * Wait for a STATUS reported after `since` that says the arm has stopped.
   *
   * Timestamped rather than "the first IDLE seen": a status can already be in
   * flight when a move is queued, so the first one back describes the arm before
   * the move started.
   */
  waitUntilIdle: (since: number, aborted: () => boolean) => Promise<void>;
  setCartesianMode: (mode: 'linear' | 'joint') => void;
  /** Drive one output. Zero-based. */
  setOutput: (index: number, high: boolean) => Promise<void>;
  /** Ask the firmware for its I/O names and current state. */
  refreshIO: () => Promise<void>;
  setCartesianSpeed: (mmPerSecond: number) => void;
  /**
   * Set the tool frame, in the model and here, and remember it.
   *
   * Changing it moves the TCP, so anything already captured against the old one
   * is stale: a held orientation was recorded for a different tool and the
   * reported position was measured to a different point. Both are cleared rather
   * than silently reinterpreted.
   */
  setToolFrame: (frame: Partial<ToolFrame>) => void;
  /** Back to the bare flange. */
  resetToolFrame: () => void;

  /**
   * Touches of one fixed point, for working out where the tool tip is.
   *
   * Flange poses - frame 6, before any tool is applied - so re-calibrating
   * cannot compound onto the previous answer. Recording the TCP instead would
   * make every calibration relative to the last one.
   */
  toolTouches: ToolTouch[];
  /** True while the four-touch procedure is running. */
  calibratingTool: boolean;
  beginToolCalibration: () => void;
  /** Record where the flange is now as another touch of the same point. */
  touchToolPoint: () => void;
  undoToolTouch: () => void;
  cancelToolCalibration: () => void;
  /** Solve for the offset and apply it. Returns what it found, or the reason not. */
  finishToolCalibration: () => ToolCalibration;
  /** The last calibration's result, kept so the panel can show its residual. */
  toolCalibration: ToolCalibration | null;
  /** Drop everything that was measured against the previous tool frame. */
  afterToolFrameChange: () => void;
  updateCurrentPosition: () => void;
  setToolLocked: (locked: boolean) => void;

  /** Ask the firmware what its motion limits currently are. */
  refreshAxisLimits: () => Promise<void>;
  /**
   * Set one axis's ceiling while the arm is running, for tuning by ear.
   *
   * Not persisted anywhere: the firmware forgets on reboot, deliberately, so
   * nobody leaves a tuning value in and stops config.h describing the machine.
   * Write the numbers down when they are right.
   */
  setAxisLimit: (axis: number, speed: number, accel: number) => Promise<void>;

  // Waypoint actions
  addWaypoint: (waypoint: Waypoint) => void;
  removeWaypoint: (id: string) => void;
  reorderWaypoints: (fromIndex: number, toIndex: number) => void;
  updateWaypoint: (id: string, updates: Partial<Waypoint>) => void;
  clearWaypoints: () => void;
  teachCurrentPosition: (label?: string) => void;

  // Work objects
  /** Named frames taught points can be measured in. */
  workObjects: WorkObject[];
  /** Which one new points are taught into, or null for the robot base. */
  activeWorkObject: string | null;
  /** The active frame itself, resolved. Base when nothing is selected. */
  activeFrame: () => WorkObject;
  setActiveWorkObject: (id: string | null) => void;
  /** Create one, sitting on the base frame until it is taught. Returns its id. */
  addWorkObject: (name: string) => string;
  renameWorkObject: (id: string, name: string) => void;
  removeWorkObject: (id: string) => void;
  /**
   * Define a frame from three touched points: origin, a point on +X, and a point
   * in the +Y half of the XY plane. Re-planning follows, because every waypoint
   * in that frame has just moved.
   */
  teachWorkObject: (id: string, points: [Vector3, Vector3, Vector3]) => boolean;

  /**
   * The three-point teach in progress, if any.
   *
   * Held here rather than in the panel because teaching a frame *requires*
   * leaving the panel: you touch a point, then move the arm, and moving the arm
   * lives on another tab. React unmounts the panel when the tab changes, which
   * threw away every point touched so far - so the one workflow this feature
   * exists for was the one that could not be completed.
   */
  teachingWorkObject: string | null;
  /** Points touched so far, in base coordinates. */
  touchedPoints: Vector3[];
  beginTeaching: (id: string) => void;
  /** Record where the tool is now as the next point. */
  touchPoint: () => void;
  undoTouch: () => void;
  cancelTeaching: () => void;
  /** Build the frame from the three touched points. */
  finishTeaching: () => boolean;
  /**
   * Forget a waypoint's recorded joint angles, so it is solved from its position.
   *
   * A waypoint that carries joint angles replays them exactly and does not follow
   * its work object. That is right for a pose taught to avoid an obstacle and
   * wrong for a point on a fixture, and only the operator knows which.
   */
  dropJointAngles: (id: string) => void;
  /**
   * Append a circle to the waypoint list, centred where the tool is now.
   *
   * Placing one by hand means teaching dozens of points and getting them all
   * slightly wrong; describing it is exact. Returns false and says why when the
   * arm cannot reach the whole figure - better found here than one point at a
   * time with the arm already moving.
   */
  addCircle: (options: { radius: number; plane: ShapePlane; clockwise?: boolean }) => boolean;

  // Trajectory actions
  planTrajectory: () => void;
  executeTrajectory: () => Promise<void>;
  pauseExecution: () => void;
  resumeExecution: () => void;
  cancelExecution: () => void;
  updatePlannerConfig: (config: Partial<PathPlannerConfig>) => void;

  // Path save/load
  exportPath: (name: string, description?: string) => SavedPath;
  importPath: (savedPath: SavedPath) => boolean;
}

// Create IK solver instance. Defaults come from DEFAULT_IK_OPTIONS; the
// damping is adaptive, so there is no fixed factor to tune here.
const ikSolver = new InverseKinematics();

// Trajectory planner instance
let trajectoryPlanner = new TrajectoryPlanner();

// Used for single Cartesian moves, which need the same straight-line sampling a
// planned path gets but none of the waypoint machinery around it.
const pathInterpolator = new PathInterpolator();

// Execution control
let executionAbortController: AbortController | null = null;
let executionPaused = false;

/**
 * Whether the target fields have adopted a real reported position yet. Reset on
 * every disconnect, so a reconnect re-seeds from the arm rather than from stale
 * numbers.
 */
let targetsInitialised = false;

const sleep = (ms: number) => new Promise<void>(resolve => setTimeout(resolve, ms));

/**
 * An id nothing else will have.
 *
 * Date.now() alone is not one. Two work objects added in the same millisecond -
 * which is what happens when anything creates them in a loop, or a test does -
 * came out with the same id, and a duplicate id is not a cosmetic problem here:
 * frameById returns the first match, so every waypoint naming that frame
 * silently resolves against whichever of the two happened to be added first.
 * The same held for waypoint ids, where a duplicate breaks deletion and
 * reordering.
 */
let idCounter = 0;
function uniqueId(prefix: string): string {
  idCounter += 1;
  return `${prefix}-${Date.now().toString(36)}-${idCounter.toString(36)}`;
}

/** Result of offering one trajectory point to the firmware. */
type SendOutcome = 'sent' | 'aborted' | 'paused';

/**
 * Offer one point to the firmware, waiting for queue space if it is full.
 *
 * `BUSY` is normal back-pressure while streaming, not a fault: the firmware
 * holds a bounded queue and answers `BUSY` rather than dropping the move, so the
 * caller resends. Keeping that queue full is the whole mechanism by which a
 * streamed path comes out smooth - the planner needs look-ahead to carry speed
 * from one point into the next instead of stopping at each one.
 *
 * Shared by both streamers because this retry is the part that is easy to get
 * subtly wrong. The loops around it are not shared: running a taught path has
 * pause and per-segment progress semantics that a single Cartesian move does not.
 *
 * @param guard checked before every attempt, so a cancel lands within one retry
 *        interval rather than after the queue drains - which can be seconds.
 */
async function offerPoint(
  serialManager: SerialManager,
  angles: JointAngles,
  speed: number,
  guard: () => SendOutcome | null
): Promise<SendOutcome> {
  for (let attempt = 0; attempt < 2000; attempt++) {
    const interrupted = guard();
    if (interrupted) return interrupted;

    const ack = await serialManager.moveToAngles(angles, speed);
    if (ack.accepted) return 'sent';

    await sleep(20);
  }
  throw new Error('Firmware motion queue never drained');
}

/** Joint angles array to the record the serial layer wants. */
function toJointAngles(q: number[]): JointAngles {
  return { J1: q[0], J2: q[1], J3: q[2], J4: q[3], J5: q[4], J6: q[5] };
}

/** Map the firmware's reported state onto the UI's state enum. */
function toRobotState(state: FirmwareState): RobotState {
  switch (state) {
    case 'MOVING': return RobotState.MOVING;
    case 'HOMING': return RobotState.HOMING;
    case 'ERROR': return RobotState.ERROR;
    case 'ESTOP': return RobotState.ESTOPPED;
    case 'IDLE':
    default: return RobotState.IDLE;
  }
}

/** Which trajectory segment a flattened point index falls in. */
function locateSegment(
  trajectory: Trajectory,
  globalIndex: number
): { segment: number; pointInSegment: number; pointsInSegment: number } {
  let remaining = globalIndex;
  for (let s = 0; s < trajectory.segments.length; s++) {
    const count = trajectory.segments[s].points.length;
    if (remaining < count) {
      return { segment: s, pointInSegment: remaining, pointsInSegment: count };
    }
    remaining -= count;
  }
  const last = Math.max(0, trajectory.segments.length - 1);
  return {
    segment: last,
    pointInSegment: 0,
    pointsInSegment: trajectory.segments[last]?.points.length ?? 0
  };
}

const initialProgress: ExecutionProgress = {
  state: ExecutionState.IDLE,
  currentSegment: 0,
  totalSegments: 0,
  currentPointInSegment: 0,
  totalPointsInSegment: 0,
  overallProgress: 0,
  elapsedTime: 0,
  estimatedTimeRemaining: 0
};

export const useRobotStore = create<RobotStore>((set, get) => ({
  // Initial state
  connectionStatus: ConnectionStatus.DISCONNECTED,
  connectionDetail: null,
  serialManager: null,
  robotState: RobotState.IDLE,
  currentAngles: { J1: 0, J2: 0, J3: 0, J4: 0, J5: 0, J6: 0 },
  targetAngles: { J1: 0, J2: 0, J3: 0, J4: 0, J5: 0, J6: 0 },
  endstopState: { J1: false, J2: false, J3: false, J4: false, J5: false, J6: false },
  motorsEnabled: false,
  events: [],
  firmwareStatus: null,
  firmwareStatusAt: 0,
  currentPosition: null,
  currentRotation: null,
  toolLocked: false,
  lockedRotation: null,
  targetPosition: null,
  ikStatus: null,
  cartesianMode: 'linear',
  // Slow enough to watch and to stop, and well inside what the joints can do:
  // J1 at its 120 deg/s limit sweeps about 420 mm/s at a 200 mm radius. The
  // firmware clamps per joint anyway, so asking for too much costs accuracy in
  // the duration estimate rather than control of the arm.
  cartesianSpeed: 50,
  cartesianAccel: 200,
  // Pushed into the kinematic model as well as held here, because FK and IK read
  // the model's copy and the store's is only for rendering. Done at module load
  // so a remembered tool is in force before the first solve, not after it.
  toolFrame: setModelToolFrame(loadToolFrame()),
  workObjects: loadWorkObjects(),
  activeWorkObject: null,
  ioState: null,
  ioNames: { outputs: [], inputs: [] },
  toolTouches: [],
  calibratingTool: false,
  toolCalibration: null,
  jogFrame: 'base',
  jogStepMm: 5,
  jogStepDeg: 5,
  teachingWorkObject: null,
  touchedPoints: [],
  manualSpeed: 30,

  // Trajectory initial state
  waypoints: [],
  trajectory: null,
  trajectoryPositions: [],
  executionState: ExecutionState.IDLE,
  executionProgress: { ...initialProgress },
  plannerConfig: { ...DEFAULT_PLANNER_CONFIG },
  trajectoryCollision: null,
  axisLimits: [],

  // Connect to robot
  connect: async (injected?: SerialManager) => {
    // Reuse the existing manager if there is one, so its listeners and
    // reconnect state are not duplicated.
    const manager = injected ?? get().serialManager ?? new SerialManager();
    const isNew = manager !== get().serialManager;

    set({ serialManager: manager, connectionDetail: null });

    if (isNew) {
      manager.onStateChange((state, detail) => {
        set({ connectionStatus: state, connectionDetail: detail ?? null });
        get().logEvent(
          state === ConnectionStatus.CONNECTED ? 'info' : 'error',
          detail ? `${state}: ${detail}` : state
        );

        if (state === ConnectionStatus.CONNECTED) return;

        // The reported position is no longer live, so stop treating it as the
        // target seed.
        targetsInitialised = false;

        // The link is down, so nothing about the arm is known any more. Abandon
        // any running path rather than let it resume against a controller that
        // has restarted in the meantime.
        executionAbortController?.abort();
        executionPaused = false;

        set({
          firmwareStatus: null,
          firmwareStatusAt: 0,
          motorsEnabled: false,
          robotState: RobotState.IDLE,
          executionState: ExecutionState.IDLE,
          executionProgress: { ...initialProgress }
        });
      });

      manager.onMessage((msg) => {
        switch (msg.type) {
          case 'POS':
            set({ currentAngles: msg.data });
            // Compute forward kinematics to update XYZ position
            get().updateCurrentPosition();

            // First position report after connecting: adopt it as the target, so
            // the target fields describe where the arm IS. They used to start at
            // all zeros, which made the very first "Move to Target" slam every
            // joint to zero - a 220 degree swing on J5 from the rest pose.
            if (!targetsInitialised) {
              targetsInitialised = true;
              set({ targetAngles: { ...msg.data } });
            }
            break;
          case 'ENDSTOP':
            set({ endstopState: msg.data });
            break;
          case 'STATUS':
            // The firmware is the authority on whether the arm is moving: it
            // still has queued moves to run after the last command was accepted.
            set({
              firmwareStatus: msg.data,
              firmwareStatusAt: msg.timestamp,
              robotState: toRobotState(msg.data.state),
              // Reported by the firmware rather than assumed from our own E
              // command, which could have been rejected or lost.
              motorsEnabled: msg.data.enabled
            });
            break;
          case 'HOMED':
            get().logEvent('ok', `J${msg.data} homed`);
            break;
          case 'LIMIT': {
            const limits = [...get().axisLimits];
            limits[msg.data.axis] = msg.data;
            set({ axisLimits: limits });
            break;
          }
          case 'IO':
            set({ ioState: msg.data });
            break;
          case 'IONAME': {
            const names = { ...get().ioNames };
            const list = [...(msg.data.direction === 'out' ? names.outputs : names.inputs)];
            list[msg.data.index] = msg.data.name;
            set({
              ioNames:
                msg.data.direction === 'out'
                  ? { ...names, outputs: list }
                  : { ...names, inputs: list }
            });
            break;
          }
          case 'OK':
            get().logEvent('ok', msg.data);
            break;
          case 'ERROR':
            get().logEvent('error', msg.data);
            break;
        }
      });
    }

    // The manager reports CONNECTING, CONNECTED and every later transition
    // through onStateChange, including a loss while this call is in flight.
    await manager.connect();

    // Ask what I/O this firmware has. The names decide how an IO report is split
    // between outputs and inputs, so without them the panel cannot tell which
    // digits are which - and hardcoding a count here would be a copy of config.h
    // that goes stale the first time a pin moves.
    await get().refreshIO().catch(() => undefined);
  },

  // Disconnect
  disconnect: async () => {
    const { serialManager } = get();

    executionAbortController?.abort();
    executionPaused = false;

    if (serialManager) {
      await serialManager.disconnect();
      serialManager.dispose();
    }

    set({
      serialManager: null,
      connectionStatus: ConnectionStatus.DISCONNECTED,
      connectionDetail: null,
      firmwareStatus: null,
      firmwareStatusAt: 0,
      motorsEnabled: false,
      robotState: RobotState.IDLE,
      executionState: ExecutionState.IDLE,
      executionProgress: { ...initialProgress },
      // Where the tool is stops being known the moment the link drops: the arm
      // can be moved by hand or power-cycled while we are not looking. Keeping
      // the last reading would leave the Cartesian panel quoting a position
      // nobody can vouch for, and the tool lock holding an orientation captured
      // from it.
      currentPosition: null,
      currentRotation: null,
      targetPosition: null,
      toolLocked: false,
      lockedRotation: null
    });
  },

  // Set target angles
  setTargetAngles: (angles) => {
    set((state) => ({
      targetAngles: { ...state.targetAngles, ...angles }
    }));
  },

  // Move to target
  moveToTarget: async () => {
    const { serialManager, targetAngles, manualSpeed } = get();
    if (!serialManager) return;

    get().logEvent('sent', 'move to target');
    const ack = await serialManager.moveToAngles(targetAngles, manualSpeed);
    if (!ack.accepted) {
      get().logEvent('error', 'Move refused: firmware queue full');
      return;
    }
    set({ robotState: RobotState.MOVING });
  },

  // Home joints
  homeJoints: async (joints) => {
    const { serialManager } = get();
    if (!serialManager) return;

    get().logEvent('sent', `H ${joints}`);
    set({ robotState: RobotState.HOMING });
    await serialManager.homeJoints(joints);
  },

  // Enable/disable motors
  enableMotors: async (enable) => {
    const { serialManager } = get();
    if (!serialManager) return;

    get().logEvent('sent', `E ${enable ? 1 : 0}`);
    await serialManager.enableMotors(enable);
    // motorsEnabled is not set here: the next STATUS report says what actually
    // happened, and guessing would show the drivers as live when the command was
    // refused.
  },

  // Emergency stop
  emergencyStop: async () => {
    const { serialManager } = get();

    // Stop the sender without going through cancelExecution, which would send a
    // graceful abort first - pointless noise ahead of a hard stop.
    executionAbortController?.abort();
    executionPaused = false;

    set({
      executionState: ExecutionState.IDLE,
      executionProgress: { ...initialProgress }
    });

    if (!serialManager) return;

    await serialManager.emergencyStop();
    set({ robotState: RobotState.ESTOPPED });
  },

  // Set manual speed
  setManualSpeed: (speed) => {
    set({ manualSpeed: speed });
  },

  logEvent: (kind, text) => {
    if (!text) return;
    set(state => ({
      events: [
        ...state.events.slice(-(MAX_EVENTS - 1)),
        { id: nextEventId++, at: Date.now(), kind, text }
      ]
    }));
  },

  clearEvents: () => set({ events: [] }),

  syncTargetsToCurrent: () => {
    set(state => ({ targetAngles: { ...state.currentAngles } }));
  },

  /**
   * Nudge one joint. Measured from the current target rather than the reported
   * position, so pressing +5 three times moves 15 degrees even while the arm is
   * still catching up.
   */
  setJogFrame: (frame) => set({ jogFrame: frame }),
  setJogStepMm: (mm) => set({ jogStepMm: Math.max(0.01, Math.min(200, mm)) }),
  setJogStepDeg: (deg) => set({ jogStepDeg: Math.max(0.1, Math.min(90, deg)) }),

  /**
   * The chosen frame's axes, expressed in base coordinates.
   *
   * Columns of a rotation matrix, so column k is that frame's k-th axis seen
   * from the robot. Which is why a jog "along X" is the same operation in all
   * three frames and only the matrix differs.
   */
  jogAxes: () => {
    const { jogFrame, currentRotation } = get();

    if (jogFrame === 'tool') {
      // Where the tool actually points, which after a tool frame is measured is
      // not where the flange points.
      return currentRotation ? rpyToMatrix(currentRotation) : null;
    }
    if (jogFrame === 'work') {
      return rpyToMatrix(get().activeFrame().rpy);
    }
    return [
      [1, 0, 0],
      [0, 1, 0],
      [0, 0, 1]
    ];
  },

  jogCartesian: async (axis, millimetres) => {
    const { currentPosition, currentRotation, jogFrame } = get();

    if (!currentPosition || !currentRotation) {
      get().logEvent('error', 'No tool position yet — connect and home first');
      return;
    }

    const R = get().jogAxes();
    if (!R) {
      get().logEvent('error', 'No tool orientation yet');
      return;
    }

    const k = { x: 0, y: 1, z: 2 }[axis];
    const step = millimetres / 1000;
    const target = {
      x: currentPosition.x + R[0][k] * step,
      y: currentPosition.y + R[1][k] * step,
      z: currentPosition.z + R[2][k] * step
    };

    set({ targetPosition: target });
    get().logEvent(
      'sent',
      `jog ${millimetres > 0 ? '+' : ''}${millimetres} mm along ${jogFrame} ${axis.toUpperCase()}`
    );

    // Straight line with the attitude held, which is what a jog means. Reuses
    // the move the Cartesian panel makes, so a jog gets the same treatment: the
    // line is sampled and solved rather than end-point solved, collisions are
    // checked at every sample, and a wrist that has to reconfigure is turned
    // first rather than flipping mid-move.
    await get().moveAlongLine(target, { ...currentRotation });
  },

  jogRotation: async (axis, degrees) => {
    const { currentPosition, currentRotation, jogFrame } = get();

    if (!get().canCommandMotion('the turn')) return;

    if (!currentPosition || !currentRotation) {
      get().logEvent('error', 'No tool position yet — connect and home first');
      return;
    }

    const R = get().jogAxes();
    if (!R) return;

    const k = { x: 0, y: 1, z: 2 }[axis];
    const about = { x: R[0][k], y: R[1][k], z: R[2][k] };
    const delta = rotationAboutAxis(about, (degrees * Math.PI) / 180);

    // Applied on the left: `about` is already expressed in base coordinates, so
    // this turns the tool about that world direction whichever frame the
    // direction came from. Composing on the right would apply it in the tool's
    // own frame a second time, and a jog in the base frame would then follow the
    // tool around.
    const next = matrixToRpy(multiply3(delta, rpyToMatrix(currentRotation)));

    get().logEvent(
      'sent',
      `turn ${degrees > 0 ? '+' : ''}${degrees}° about ${jogFrame} ${axis.toUpperCase()}`
    );

    // Same place, new attitude. moveAlongLine needs a line, and there is none -
    // so this is a pose solve and a single move, which is correct here: the tip
    // is meant to stay put, and nothing is being traced.
    const seed = [
      get().currentAngles.J1, get().currentAngles.J2, get().currentAngles.J3,
      get().currentAngles.J4, get().currentAngles.J5, get().currentAngles.J6
    ];
    const solved = ikSolver.solvePose({ position: currentPosition, rotation: next }, seed);
    set({ ikStatus: solved });

    if (!solved.success) {
      get().logEvent('error', `Cannot turn there: ${solved.error ?? 'no solution'}`);
      return;
    }

    const hit = checkSelfCollision(solved.jointAngles);
    if (hit.colliding) {
      get().logEvent('error', `Refusing the turn: it would put the ${hit.message}.`);
      return;
    }

    const { serialManager, manualSpeed } = get();
    if (!serialManager) return;
    const ack = await serialManager.moveToAngles(toJointAngles(solved.jointAngles), manualSpeed);
    if (!ack.accepted) get().logEvent('error', 'Move refused: firmware queue full');
  },

  jogJoint: async (joint, deltaDegrees) => {
    const { serialManager, targetAngles, manualSpeed } = get();
    if (!serialManager) return;

    const index = (['J1', 'J2', 'J3', 'J4', 'J5', 'J6'] as const).indexOf(joint);
    if (index < 0) return;

    const proposed = targetAngles[joint] + deltaDegrees;
    const clamped = Math.max(
      JOINT_LIMITS_DEG.min[index],
      Math.min(JOINT_LIMITS_DEG.max[index], proposed)
    );

    if (clamped !== proposed) {
      get().logEvent('info', `${joint} clamped to its limit at ${clamped.toFixed(1)} deg`);
    }

    const next: JointAngles = { ...targetAngles, [joint]: clamped };
    set({ targetAngles: next });

    get().logEvent('sent', `jog ${joint} ${deltaDegrees > 0 ? '+' : ''}${deltaDegrees}`);
    const ack = await serialManager.moveToAngles(next, manualSpeed);
    if (!ack.accepted) get().logEvent('error', 'Move refused: firmware queue full');
  },

  goToHomePose: async () => {
    const { serialManager, manualSpeed } = get();
    if (!serialManager) return;

    const pose = clampToLimitsDeg(HOME_POSE_DEG.slice(0, NUM_JOINTS));
    const angles: JointAngles = {
      J1: pose[0], J2: pose[1], J3: pose[2], J4: pose[3], J5: pose[4], J6: pose[5]
    };

    set({ targetAngles: angles });
    get().logEvent('sent', 'move to rest pose');

    const ack = await serialManager.moveToAngles(angles, manualSpeed);
    if (!ack.accepted) get().logEvent('error', 'Move refused: firmware queue full');
  },

  /**
   * Send a protocol line as typed. The bring-up checklist asks for exact commands
   * such as `J 0 0 10 0 0 0 5`, and without this the only way to send one was a
   * separate serial monitor, which cannot hold the port at the same time.
   */
  sendRawCommand: async (line) => {
    const { serialManager } = get();
    const trimmed = line.trim();
    if (!serialManager || trimmed.length === 0) return;

    get().logEvent('sent', trimmed);
    try {
      await serialManager.sendCommand(trimmed);
    } catch (error) {
      get().logEvent('error', error instanceof Error ? error.message : 'Send failed');
    }
  },

  // Update current Cartesian position using forward kinematics
  updateCurrentPosition: () => {
    const { currentAngles } = get();

    const anglesArray = [
      currentAngles.J1,
      currentAngles.J2,
      currentAngles.J3,
      currentAngles.J4,
      currentAngles.J5,
      currentAngles.J6
    ];

    const fkResult = ForwardKinematics.solve(anglesArray);

    if (fkResult.success) {
      set({
        currentPosition: fkResult.endEffectorPose.position,
        currentRotation: fkResult.endEffectorPose.rotation
      });
    }
  },

  refreshAxisLimits: async () => {
    await get().sendRawCommand('V');
  },

  setAxisLimit: async (axis, speed, accel) => {
    await get().sendRawCommand(`V ${axis + 1} ${speed.toFixed(2)} ${accel.toFixed(2)}`);
    // Read back rather than assume: the firmware clamps to its own ceiling, and
    // refuses outright while the arm is moving.
    await get().refreshAxisLimits();
  },

  setCartesianMode: (mode) => set({ cartesianMode: mode }),

  setOutput: async (index, high) => {
    const { serialManager } = get();
    if (!serialManager) return;
    await get().sendRawCommand(`O ${index + 1} ${high ? 1 : 0}`);
  },

  refreshIO: async () => {
    const { serialManager } = get();
    if (!serialManager) return;
    await get().sendRawCommand('O');
  },

  beginToolCalibration: () => {
    set({ calibratingTool: true, toolTouches: [], toolCalibration: null });
    get().logEvent(
      'info',
      'Tool calibration: touch one fixed point four times, turning the wrist ' +
        'well between touches'
    );
  },

  touchToolPoint: () => {
    const { calibratingTool, toolTouches, currentAngles, firmwareStatus } = get();
    if (!calibratingTool) return;

    if (firmwareStatus && !firmwareStatus.positionTrusted) {
      get().logEvent(
        'error',
        'Not recording that touch: the reported position is not trusted until ' +
          'the arm has been homed.'
      );
      return;
    }

    // Frame 6, not the TCP. The whole point is to find what sits between them,
    // so reading the TCP would fold the answer being looked for into the
    // measurement of it.
    const fk = ForwardKinematics.solveRad(
      degToRad([
        currentAngles.J1, currentAngles.J2, currentAngles.J3,
        currentAngles.J4, currentAngles.J5, currentAngles.J6
      ])
    );
    const flange = fk.frames[NUM_JOINTS - 1];

    set({
      toolTouches: [
        ...toolTouches,
        { position: getTranslation(flange), rotation: getRotation(flange) }
      ]
    });
    get().logEvent('ok', `Touch ${toolTouches.length + 1} of 4 recorded`);
  },

  undoToolTouch: () => set(state => ({ toolTouches: state.toolTouches.slice(0, -1) })),

  cancelToolCalibration: () =>
    set({ calibratingTool: false, toolTouches: [], toolCalibration: null }),

  finishToolCalibration: () => {
    const result = solveToolOffset(get().toolTouches);
    set({ toolCalibration: result });

    if (!result.ok) {
      get().logEvent('error', `Tool calibration: ${result.reason}`);
      // Touches are kept, so a rejected set can be added to rather than redone.
      return result;
    }

    // Only the offset. The rotation is a different measurement and needs a
    // gauge; leaving it alone means a calibration cannot silently undo one.
    get().setToolFrame({ xyz: result.offset });
    set({ calibratingTool: false, toolTouches: [] });

    get().logEvent(
      'ok',
      `Tool tip is ${(Math.hypot(result.offset.x, result.offset.y, result.offset.z) * 1000).toFixed(1)} mm ` +
        `from the flange, touches agreeing to ${result.residualMm.toFixed(2)} mm`
    );
    return result;
  },

  setToolFrame: (frame) => {
    const applied = setModelToolFrame(frame);
    saveToolFrame(applied);
    set({ toolFrame: applied });
    get().afterToolFrameChange();
  },

  resetToolFrame: () => {
    const applied = resetModelToolFrame();
    saveToolFrame(applied);
    set({ toolFrame: applied });
    get().afterToolFrameChange();
  },

  /**
   * Everything that was measured against the old tool is now wrong.
   *
   * The held orientation was captured for a tool pointing a different way, and
   * the reported position was measured to a different point - moving the TCP by
   * 100 mm does not move the arm, it changes what the number means. Reinterpreting
   * them silently would leave a tool lock holding an attitude nobody chose. The
   * position is recomputed from the joint angles, which have not changed and are
   * still true.
   */
  afterToolFrameChange: () => {
    if (get().toolLocked) {
      set({ toolLocked: false, lockedRotation: null });
      get().logEvent(
        'info',
        'Tool lock released: the orientation it was holding was captured for the previous tool frame'
      );
    }
    set({ ikStatus: null, targetPosition: null });
    get().updateCurrentPosition();
  },

  setCartesianSpeed: (mmPerSecond) => {
    const speed = Math.max(1, Math.min(500, mmPerSecond));
    // Acceleration tracks speed rather than being set separately. Reaching the
    // requested speed inside a quarter of a second is what keeps the ramp short
    // relative to the move; a fixed acceleration would turn a slow move into a
    // long ramp with no cruise, which is where the tool tips.
    set({ cartesianSpeed: speed, cartesianAccel: speed * 4 });
  },

  setToolLocked: (locked) => {
    if (!locked) {
      set({ toolLocked: false, lockedRotation: null });
      return;
    }
    // Capture where the tool points now, and hold that.
    get().updateCurrentPosition();
    const rotation = get().currentRotation;
    if (!rotation) {
      get().logEvent('error', 'Cannot lock the tool: no position reported yet');
      return;
    }
    set({ toolLocked: true, lockedRotation: { ...rotation } });
    get().logEvent('info', 'Tool orientation locked');
  },

  // Set target Cartesian position
  setTargetPosition: (position) => {
    set({ targetPosition: position });
  },

  // Move to Cartesian position using inverse kinematics
  moveToPosition: async (position) => {
    const {
      serialManager,
      currentAngles,
      manualSpeed,
      firmwareStatus,
      toolLocked,
      lockedRotation
    } = get();
    if (!serialManager) return;

    if (!get().canCommandMotion('the Cartesian move')) return;

    // Use current joint angles as initial guess for IK
    const initialGuess = [
      currentAngles.J1,
      currentAngles.J2,
      currentAngles.J3,
      currentAngles.J4,
      currentAngles.J5,
      currentAngles.J6
    ];

    // Locked, the solver has to satisfy orientation as well as position: six
    // constraints against six joints, which is exactly determined and costs
    // reach - from the parked pose, roughly 40 mm in +X against 85 mm free.
    // Unlocked, orientation is left to fall where it may.
    const orientation = toolLocked && lockedRotation ? lockedRotation : undefined;

    // Solved here as well as inside the line interpolator, to answer two
    // questions the interpolator does not: whether the destination is reachable
    // at all, which is the cheap check to fail on first, and what residual to
    // show in the panel.
    const ikResult = orientation
      ? ikSolver.solvePose({ position, rotation: orientation }, initialGuess)
      : ikSolver.solvePosition(position, initialGuess);

    // Store IK status for UI feedback
    set({ ikStatus: ikResult });

    if (!ikResult.success) {
      // The Cartesian panel renders ikStatus, including the residual, so there is
      // no need to interrupt with a modal dialog.
      get().logEvent('error', `IK: ${ikResult.error ?? 'no solution'}`);
      return;
    }

    // The solver satisfies position and orientation, and knows nothing about the
    // arm's own shape. A solution can put the tool exactly where it was asked
    // while folding the forearm through the shoulder to get there.
    const hit = checkSelfCollision(ikResult.jointAngles);
    if (hit.colliding) {
      get().logEvent(
        'error',
        `Refusing the move: it would put the ${hit.message}. The target is ` +
          'reachable, so approaching it from a different pose may work.'
      );
      return;
    }

    const targetAngles = toJointAngles(ikResult.jointAngles);
    set({ targetAngles });

    if (get().cartesianMode === 'joint') {
      get().logEvent('sent', 'move to XYZ target, joint path');
      const ack = await serialManager.moveToAngles(targetAngles, manualSpeed);
      if (!ack.accepted) {
        get().logEvent('error', 'Move refused: firmware queue full');
        return;
      }
      set({ robotState: RobotState.MOVING });
      return;
    }

    await get().moveAlongLine(position, orientation);
  },

  /**
   * Drive the tool along the straight line to `position`, solving IK at every
   * sample rather than only at the two ends.
   *
   * Solving only the ends and sending one joint move constrains nothing in
   * between. The firmware, given a start pose and an end pose, moves every joint
   * proportionally so they arrive together - a perfectly valid path between the
   * two, and an arbitrary one. Measured on a 100 mm move in +Y from the parked
   * pose, that path bows 7.7 mm off the straight line and tips the tool 11.13
   * degrees at the midpoint, with both errors falling back to zero at the far
   * end where the solve was done.
   *
   * The give-away is J4, the wrist joint that holds the tool level. Holding the
   * tool still needs almost its whole 88 degree move inside the first few
   * millimetres and then nothing - 165 -> 84 -> 77 -> 75 -> 75. Spread evenly
   * across the move instead - 165, 154, 143, 132, 121 - the tool tips with it.
   *
   * Sampling every 2 mm with the orientation constraint applied at each one
   * leaves the firmware only a 2 mm gap to fill, where the same proportional
   * interpolation cannot go far wrong. The same move then holds 0.29 degrees.
   */
  canCommandMotion: (what) => {
    const { firmwareStatus } = get();

    // A Cartesian target becomes absolute joint angles, so it only means
    // anything if the arm and the firmware agree on where the arm is. Untrusted
    // - not homed since power-up, or steps possibly lost to a stop - those
    // angles are measured from a datum that does not exist.
    if (firmwareStatus && !firmwareStatus.positionTrusted) {
      get().logEvent(
        'error',
        `Refusing ${what}: the arm has not been homed since power-up, or lost ` +
          'its datum to a stop.'
      );
      return false;
    }

    if (firmwareStatus && !firmwareStatus.enabled) {
      get().logEvent('error', `Refusing ${what}: the motors are off (E 1)`);
      return false;
    }

    return true;
  },

  waitUntilIdle: async (since, aborted) => {
    for (let attempt = 0; attempt < 3000; attempt++) {
      if (aborted()) return;
      const { firmwareStatus, firmwareStatusAt } = get();
      if (firmwareStatus && firmwareStatusAt > since && firmwareStatus.state === 'IDLE') return;
      await sleep(20);
    }
  },

  moveAlongLine: async (position, orientation) => {
    const { serialManager, currentAngles, cartesianSpeed, cartesianAccel, manualSpeed } = get();
    if (!serialManager) return;

    // Guarded here and not only in moveToPosition. This is a public action that
    // commands the arm, and jogging calls it directly - so relying on the
    // caller's checks meant a jog moved an arm whose datum nobody trusted,
    // which is the one thing every other entry point refuses to do.
    if (!get().canCommandMotion('the move')) return;

    const startAngles = [
      currentAngles.J1, currentAngles.J2, currentAngles.J3,
      currentAngles.J4, currentAngles.J5, currentAngles.J6
    ];

    const segment = pathInterpolator.interpolateCartesianSpace(
      startAngles,
      position,
      cartesianSpeed,
      cartesianAccel,
      20,
      orientation
    );

    // The endpoint is reachable - it was solved above - but the straight line to
    // it need not be. Refuse rather than run the part that works: a move that
    // stops partway leaves the tool somewhere nobody asked for, and the arm is
    // then not where the panel says it is going.
    if ((segment.unreachableSamples ?? 0) > 0 || segment.points.length < 2) {
      get().logEvent(
        'error',
        'The target is reachable but the straight line to it is not — ' +
          `${segment.unreachableSamples ?? 0} of ${segment.points.length} points ` +
          'have no solution. Move somewhere in between first, or switch to a joint path.'
      );
      return;
    }

    // A jump between two samples that are 2 mm apart in tool space. The tool pose
    // is right at both of them and unconstrained between, and the firmware fills
    // that gap by moving every joint proportionally - which is precisely the
    // problem this whole path was built to avoid, reappearing inside one step.
    //
    // The interpolator has already tried solving the line from its far end, so
    // reaching here means that did not help either.
    const jump = segment.discontinuity;
    if (jump) {
      get().logEvent(
        'error',
        `Refusing the move: J${jump.axis + 1} jumps ${jump.degrees.toFixed(0)}° ` +
          `between two samples 2 mm apart, ${jump.atPercent}% along, and solving ` +
          'the line from the far end does not avoid it. Move J5 away from 131° ' +
          'first, or use a joint path.'
      );
      return;
    }

    // Every sample, not just the destination. A line can pass through a pose that
    // folds the arm into itself while both of its ends are clear. The
    // reconfiguration is checked with them: it is a real move the arm makes.
    const reconfiguration = segment.reconfiguration ?? null;
    const toCheck = segment.points.map(p => p.jointAngles);
    if (reconfiguration) {
      for (let k = 1; k < 20; k++) {
        const t = k / 20;
        toCheck.push(startAngles.map((v, i) => v + t * (reconfiguration[i] - v)));
      }
    }
    const collision = firstCollidingPoint(toCheck);
    if (collision) {
      const inPath = collision.index < segment.points.length;
      const atPercent = Math.round((collision.index / (segment.points.length - 1)) * 100);
      get().logEvent(
        'error',
        inPath
          ? `Refusing the move: the straight line folds the arm into itself ` +
              `${atPercent}% of the way along (${collision.result.message}).`
          : `Refusing the move: turning the wrist into position for it would put ` +
              `the ${collision.result.message}.`
      );
      return;
    }

    executionAbortController = new AbortController();
    const aborted = () => executionAbortController?.signal.aborted ?? true;

    // Turn the wrist into the configuration the line needs before starting it.
    //
    // The tool does not move while this happens: at the singularity J4 and J6
    // turn about the same axis, so counter-rotating them is exactly null-space
    // motion. Measured over the whole 90 degree reconfiguration the tool moves
    // 0.0 mm and tilts 0 degrees - it is a wrist turning in place, not a detour.
    if (reconfiguration) {
      const swing = Math.max(...reconfiguration.map((v, i) => Math.abs(v - startAngles[i])));
      get().logEvent(
        'info',
        `Turning the wrist ${swing.toFixed(0)}° into position first — the tool stays where it is`
      );

      const ack = await serialManager.moveToAngles(toJointAngles(reconfiguration), manualSpeed);
      if (!ack.accepted) {
        get().logEvent('error', 'Wrist reconfiguration refused: firmware queue full');
        return;
      }
      await get().waitUntilIdle(Date.now(), aborted);
      if (aborted()) return;
    }

    get().logEvent(
      'sent',
      `move to XYZ target, straight line in ${segment.points.length} points ` +
        `(${(segment.distance * 1000).toFixed(0)} mm)`
    );
    set({ robotState: RobotState.MOVING });

    try {
      for (let i = 1; i < segment.points.length; i++) {
        // Per-point speed, so the tool actually travels at the requested mm/s.
        // A single figure for the whole move cannot do that: each 2 mm step
        // needs a different joint speed depending on how the arm is folded, and
        // near a singularity the same 2 mm costs far more joint travel than it
        // does in the middle of the workspace.
        const dt = segment.points[i].time - segment.points[i - 1].time;
        const travel = Math.max(
          ...segment.points[i].jointAngles.map((v, j) =>
            Math.abs(v - segment.points[i - 1].jointAngles[j])
          )
        );
        const speed = dt > 1e-6 ? travel / dt : cartesianSpeed;

        const outcome = await offerPoint(
          serialManager,
          toJointAngles(segment.points[i].jointAngles),
          speed,
          () => (aborted() ? 'aborted' : null)
        );

        if (outcome !== 'sent') {
          get().logEvent('info', 'Cartesian move cancelled');
          return;
        }
      }
    } catch (error) {
      if (!aborted()) {
        get().logEvent('error', `Cartesian move failed: ${(error as Error).message}`);
      }
    } finally {
      executionAbortController = null;
    }
  },

  // ===== WAYPOINT ACTIONS =====

  addWaypoint: (waypoint) => {
    set((state) => ({
      waypoints: [...state.waypoints, waypoint],
      trajectory: null,
      trajectoryPositions: []
    }));
  },

  removeWaypoint: (id) => {
    set((state) => ({
      waypoints: state.waypoints.filter(wp => wp.id !== id),
      trajectory: null,
      trajectoryPositions: []
    }));
  },

  reorderWaypoints: (fromIndex, toIndex) => {
    set((state) => {
      const newWaypoints = [...state.waypoints];
      const [moved] = newWaypoints.splice(fromIndex, 1);
      newWaypoints.splice(toIndex, 0, moved);
      return {
        waypoints: newWaypoints,
        trajectory: null,
        trajectoryPositions: []
      };
    });
  },

  updateWaypoint: (id, updates) => {
    set((state) => ({
      waypoints: state.waypoints.map(wp =>
        wp.id === id ? { ...wp, ...updates } : wp
      ),
      trajectory: null,
      trajectoryPositions: []
    }));
  },

  clearWaypoints: () => {
    set({ trajectoryCollision: null });
    set({
      waypoints: [],
      trajectory: null,
      trajectoryPositions: [],
      executionState: ExecutionState.IDLE,
      executionProgress: { ...initialProgress }
    });
  },

  teachCurrentPosition: (label?) => {
    const { currentAngles, currentPosition, waypoints, firmwareStatus } = get();

    // Teaching records the angles the arm is reporting. Untrusted, those are a
    // guess, and a waypoint taught from a guess is a wrong position that only
    // shows itself later, when the path is run.
    if (firmwareStatus && !firmwareStatus.positionTrusted) {
      get().logEvent(
        'error',
        'Not teaching this point: the reported position is not trusted until ' +
          'the arm has been homed.'
      );
      return;
    }

    if (!currentPosition) {
      get().updateCurrentPosition();
    }

    const position = get().currentPosition;
    if (!position) return;

    // Recorded in the active work object's frame, which is the whole point of
    // having one: the number stored describes the fixture, not where the fixture
    // happened to be standing today.
    //
    // The joint angles are recorded too and are NOT in any frame - they are the
    // arm's own. That is deliberate and it is a trap worth knowing about: a
    // waypoint that carries joint angles is replayed from them exactly, so it
    // will not follow its work object when the frame is re-taught. Points meant
    // to move with a fixture have to be taught without them, which is what
    // dropJointAngles does.
    const frame = get().activeFrame();
    const newWaypoint: Waypoint = {
      id: uniqueId('wp'),
      position: pointFromBase(frame, position),
      frame: frame.id === BASE_FRAME.id ? undefined : frame.id,
      jointAngles: [
        currentAngles.J1,
        currentAngles.J2,
        currentAngles.J3,
        currentAngles.J4,
        currentAngles.J5,
        currentAngles.J6
      ],
      speed: get().plannerConfig.defaultSpeed,
      label: label || `WP ${waypoints.length + 1}`
    };

    get().addWaypoint(newWaypoint);
  },

  activeFrame: () => frameById(get().workObjects, get().activeWorkObject),

  setActiveWorkObject: (id) => {
    set({ activeWorkObject: id });
    const frame = frameById(get().workObjects, id);
    get().logEvent('info', `Teaching into ${frame.name}`);
  },

  addWorkObject: (name) => {
    // Created at the base frame rather than nowhere, so it is usable - and
    // visibly untaught - before the three points are touched.
    const object: WorkObject = {
      id: uniqueId('wobj'),
      name: name.trim() || `Work object ${get().workObjects.length + 1}`,
      origin: { x: 0, y: 0, z: 0 },
      rpy: { roll: 0, pitch: 0, yaw: 0 }
    };
    set(state => ({
      workObjects: [...state.workObjects, object],
      activeWorkObject: object.id
    }));
    saveWorkObjects(get().workObjects);
    get().logEvent('info', `Added ${object.name} — sitting on the base frame until it is taught`);
    return object.id;
  },

  renameWorkObject: (id, name) => {
    set(state => ({
      workObjects: state.workObjects.map(o =>
        o.id === id ? { ...o, name: name.trim() || o.name } : o
      )
    }));
    saveWorkObjects(get().workObjects);
  },

  removeWorkObject: (id) => {
    // Waypoints keep naming it. Resolving a missing frame falls back to base,
    // which puts them somewhere real and wrong rather than nowhere - so say so
    // loudly instead of deleting quietly.
    const orphaned = get().waypoints.filter(w => w.frame === id).length;

    set(state => ({
      workObjects: state.workObjects.filter(o => o.id !== id),
      activeWorkObject: state.activeWorkObject === id ? null : state.activeWorkObject
    }));
    saveWorkObjects(get().workObjects);

    if (orphaned > 0) {
      get().logEvent(
        'error',
        `${orphaned} waypoint${orphaned === 1 ? '' : 's'} still name that work object. ` +
          'They now resolve against the robot base, which is not where they were taught.'
      );
    }
  },

  teachWorkObject: (id, points) => {
    const result = teachFromThreePoints(points[0], points[1], points[2]);
    if (!result.ok) {
      get().logEvent('error', `Cannot build that frame: ${result.reason}`);
      return false;
    }

    set(state => ({
      workObjects: state.workObjects.map(o =>
        o.id === id ? { ...o, origin: result.origin, rpy: result.rpy } : o
      )
    }));
    saveWorkObjects(get().workObjects);

    const moved = get().waypoints.filter(w => w.frame === id).length;
    get().logEvent(
      'info',
      `Taught ${frameById(get().workObjects, id).name}` +
        (moved > 0 ? ` — ${moved} waypoint${moved === 1 ? '' : 's'} moved with it` : '')
    );

    // The path was planned against the old frame.
    if (get().trajectory) get().planTrajectory();
    return true;
  },

  beginTeaching: (id) => {
    set({ teachingWorkObject: id, touchedPoints: [] });
    // Jogging along the fixture's own edges is what the next three touches want,
    // and it is the frame being taught - so until it is, this is the previous
    // guess. Selecting it anyway means the axes drawn in the 3D view are the
    // ones the jog buttons move along, which is the point.
    set({ activeWorkObject: id });
    get().logEvent(
      'info',
      `Teaching ${frameById(get().workObjects, id).name}: touch the origin, ` +
        'then a point along +X, then one on the +Y side'
    );
  },

  touchPoint: () => {
    const { teachingWorkObject, touchedPoints, firmwareStatus } = get();
    if (!teachingWorkObject || touchedPoints.length >= 3) return;

    // A touched point is a measurement of where the tool is. Untrusted, it is a
    // guess, and a frame built from guesses puts every point taught in it
    // somewhere nobody chose.
    if (firmwareStatus && !firmwareStatus.positionTrusted) {
      get().logEvent(
        'error',
        'Not recording that point: the reported position is not trusted until ' +
          'the arm has been homed.'
      );
      return;
    }

    get().updateCurrentPosition();
    const position = get().currentPosition;
    if (!position) {
      get().logEvent('error', 'No tool position yet');
      return;
    }

    set({ touchedPoints: [...touchedPoints, { ...position }] });
    get().logEvent(
      'ok',
      `Point ${touchedPoints.length + 1} of 3 recorded at ` +
        `${(position.x * 1000).toFixed(1)}, ${(position.y * 1000).toFixed(1)}, ` +
        `${(position.z * 1000).toFixed(1)} mm`
    );
  },

  undoTouch: () => set(state => ({ touchedPoints: state.touchedPoints.slice(0, -1) })),

  cancelTeaching: () => set({ teachingWorkObject: null, touchedPoints: [] }),

  finishTeaching: () => {
    const { teachingWorkObject, touchedPoints } = get();
    if (!teachingWorkObject || touchedPoints.length < 3) return false;

    const ok = get().teachWorkObject(teachingWorkObject, [
      touchedPoints[0],
      touchedPoints[1],
      touchedPoints[2]
    ]);
    // Kept on a failure, so a rejected third point can be re-touched without
    // starting the first two again.
    if (ok) set({ teachingWorkObject: null, touchedPoints: [] });
    return ok;
  },

  dropJointAngles: (id) => {
    set(state => ({
      waypoints: state.waypoints.map(w =>
        w.id === id ? { ...w, jointAngles: undefined } : w
      )
    }));
    get().logEvent(
      'info',
      'Waypoint will be solved from its position now, so it follows its work object'
    );
  },

  addCircle: ({ radius, plane, clockwise }) => {
    const { currentPosition, currentAngles, toolLocked, lockedRotation, plannerConfig } = get();

    if (!currentPosition) {
      get().logEvent('error', 'No tool position yet — connect and home first');
      return false;
    }
    if (!(radius > 0)) {
      get().logEvent('error', 'Circle radius must be greater than zero');
      return false;
    }

    const seed = [
      currentAngles.J1, currentAngles.J2, currentAngles.J3,
      currentAngles.J4, currentAngles.J5, currentAngles.J6
    ];

    const shape: WaypointShape = { kind: 'circle', radius, plane, clockwise };

    // Hold the tool if either the Cartesian panel's lock or the path setting
    // asks for it, so the figure is checked against the way it will be run.
    const orientation =
      plannerConfig.holdToolOrientation || toolLocked ? lockedRotation ?? undefined : undefined;

    const check = validateCircle(shape, currentPosition, orientation, seed);
    if (!check.ok) {
      get().logEvent('error', check.message);
      return false;
    }

    // One waypoint for the whole figure: deleting it removes the circle, not
    // forty points of it, and the planner samples the arc itself.
    //
    // Recorded in the active work object like any other taught point, so a
    // circle drawn on a fixture moves with the fixture. The validation above
    // used base coordinates deliberately - reachability is a fact about the arm,
    // not about the frame the figure is described in.
    const frame = get().activeFrame();
    const waypoint = makeCircleWaypoint(
      shape,
      pointFromBase(frame, currentPosition),
      orientation ? rotationFromBase(frame, orientation) : undefined,
      plannerConfig.defaultSpeed
    );
    if (frame.id !== BASE_FRAME.id) waypoint.frame = frame.id;

    set(state => ({ waypoints: [...state.waypoints, waypoint] }));
    get().logEvent('info', `Added ${check.message}`);
    return true;
  },

  // ===== TRAJECTORY ACTIONS =====

  planTrajectory: () => {
    const { waypoints, currentAngles, plannerConfig, workObjects } = get();

    if (waypoints.length === 0) {
      set({ trajectory: null, trajectoryPositions: [] });
      return;
    }

    // A waypoint naming a work object that no longer exists resolves against the
    // base frame, which puts it somewhere real and wrong. Said once, here, rather
    // than left for the operator to notice when the arm goes to the wrong place.
    const dangling = danglingFrames(workObjects, waypoints.map(w => w.frame));
    if (dangling.length > 0) {
      get().logEvent(
        'error',
        `${dangling.length} work object${dangling.length === 1 ? '' : 's'} named by ` +
          'waypoints no longer exist. Those points are being planned against the ' +
          'robot base, which is not where they were taught.'
      );
    }

    set({
      executionState: ExecutionState.PLANNING,
      executionProgress: { ...initialProgress, state: ExecutionState.PLANNING }
    });

    trajectoryPlanner.updateConfig(plannerConfig);

    const startAngles = [
      currentAngles.J1,
      currentAngles.J2,
      currentAngles.J3,
      currentAngles.J4,
      currentAngles.J5,
      currentAngles.J6
    ];

    const trajectory = trajectoryPlanner.planTrajectory(waypoints, startAngles, workObjects);
    const positions = TrajectoryPlanner.getTrajectoryPositions(trajectory);

    // Checking the waypoints is not enough: the arm can pass through itself
    // between two poses that are each perfectly safe, and a path is where that
    // happens - it is the only part of the app that commits the arm to a route
    // rather than a destination. Reported at planning time, with a position, so
    // it can be seen in the 3D preview before anything moves.
    const points = TrajectoryPlanner.flattenTrajectory(trajectory);
    const hit = firstCollidingPoint(points.map(p => p.jointAngles));
    const collision = hit
      ? {
          atPercent:
            points.length > 1 ? Math.round((100 * hit.index) / (points.length - 1)) : 0,
          message: hit.result.message
        }
      : null;

    if (collision) {
      get().logEvent(
        'error',
        `This path folds the arm into itself ${collision.atPercent}% of the way ` +
          `through (${collision.message}). It will not run until the waypoints change.`
      );
    }
    set({ trajectoryCollision: collision });

    set({
      trajectory,
      trajectoryPositions: positions,
      executionState: ExecutionState.IDLE,
      executionProgress: {
        ...initialProgress,
        totalSegments: trajectory.segments.length
      }
    });
  },

  executeTrajectory: async () => {
    const { trajectory, serialManager, firmwareStatus } = get();

    if (!trajectory || trajectory.segments.length === 0) {
      get().logEvent('error', 'Nothing to execute: plan a path first');
      return;
    }

    if (!serialManager) {
      get().logEvent('error', 'Not connected to the arm');
      return;
    }

    // A path is a list of absolute joint angles, so it only means anything if
    // the arm agrees with the firmware about where it is. When the position is
    // untrusted - not homed since power-up, or steps possibly lost to an
    // emergency stop - those angles are measured from a datum that does not
    // exist, and running them drives the arm somewhere nobody chose.
    //
    // The firmware will not catch this. It accepts J commands whenever the
    // motors are on and no stop is latched; it reports the doubt in STATUS and
    // leaves the decision to the host. Loading a saved path onto an un-homed arm
    // and pressing Execute is the way this bites.
    if (firmwareStatus && !firmwareStatus.positionTrusted) {
      get().logEvent(
        'error',
        'Refusing to run the path: the arm has not been homed since power-up, ' +
          'or lost its datum to a stop. Home it first (H ALL).'
      );
      return;
    }

    if (firmwareStatus && !firmwareStatus.enabled) {
      get().logEvent('error', 'Refusing to run the path: the motors are off (E 1)');
      return;
    }

    const collision = get().trajectoryCollision;
    if (collision) {
      get().logEvent(
        'error',
        `Refusing to run the path: it folds the arm into itself ` +
          `${collision.atPercent}% of the way through (${collision.message}).`
      );
      return;
    }

    // Same refusal a single Cartesian move makes, for the same reason: two
    // samples 2 mm apart with the arm somewhere else entirely between them. The
    // tool pose is right at both and unconstrained through the jump.
    const jump = trajectory.discontinuity;
    if (jump) {
      const segment = trajectory.segments[jump.segment];
      const unreachable = segment?.unreachableSamples ?? 0;

      // Where the jump falls says what caused it, and the two want different
      // things done about them. At the start it is the wrist being in the wrong
      // configuration for the line - the pose the arm is parked in. Part way
      // along, the start pose is not the suspect: either the line has run out of
      // reach and the solver has restarted from a distant seed, or the wrist
      // crosses the singular configuration mid-leg. Telling an operator to
      // change the start pose for a fault at 72% sends them somewhere useless.
      const cause =
        unreachable > 0
          ? `${unreachable} samples of that segment have no solution at all, so the ` +
            'line runs out of reach part way and the solver restarts somewhere else. ' +
            'Holding the tool costs a lot of reach — shorten that leg, or free the tool for it.'
          : jump.atPercent <= 15
            ? 'The wrist is in the wrong configuration for this line. Turn J5 away ' +
              'from 131° before starting, or run this leg in joint mode.'
            : 'The wrist crosses its singular configuration part way along this leg. ' +
              'Put a waypoint either side of where it happens, or run this leg in joint mode.';

      get().logEvent(
        'error',
        `Refusing to run the path: J${jump.axis + 1} jumps ${jump.degrees.toFixed(0)}° ` +
          `between two samples in segment ${jump.segment + 1}, ${jump.atPercent}% along it — ` +
          `about ${(jump.degrees / 180).toFixed(1)} half-turns while the tool moves 2 mm. ` +
          cause
      );
      return;
    }

    // The path may need the wrist in a configuration the arm is not in: its
    // first segment can have been solved from its far end, which is what keeps
    // the line smooth but leaves the near end wanting a different wrist. Without
    // this the first point of the path simply *is* that jump, unannounced and
    // unchecked.
    //
    // Free at the tool - J4 and J6 are the same axis there, so counter-rotating
    // them is null-space motion - but a real move, so it is checked for
    // collisions and waited on like any other.
    const entry = trajectory.reconfiguration;
    if (entry) {
      const here = [
        get().currentAngles.J1, get().currentAngles.J2, get().currentAngles.J3,
        get().currentAngles.J4, get().currentAngles.J5, get().currentAngles.J6
      ];
      const swing = Math.max(...entry.map((v, i) => Math.abs(v - here[i])));

      const sweep: number[][] = [];
      for (let k = 1; k <= 20; k++) {
        const t = k / 20;
        sweep.push(here.map((v, i) => v + t * (entry[i] - v)));
      }
      const hit = firstCollidingPoint(sweep);
      if (hit) {
        get().logEvent(
          'error',
          `Refusing to run the path: turning the wrist into position for it would ` +
            `put the ${hit.result.message}.`
        );
        return;
      }

      get().logEvent(
        'info',
        `Turning the wrist ${swing.toFixed(0)}° into position first — the tool stays where it is`
      );
      const ack = await serialManager.moveToAngles(toJointAngles(entry), get().manualSpeed);
      if (!ack.accepted) {
        get().logEvent('error', 'Wrist reconfiguration refused: firmware queue full');
        return;
      }
      await get().waitUntilIdle(Date.now(), () => false);
    }

    // Set up abort controller
    executionAbortController = new AbortController();
    executionPaused = false;

    const allPoints = TrajectoryPlanner.flattenTrajectory(trajectory);
    // Built by the same walk as the points themselves, so the two agree on which
    // index is which. Reconstructing it from segment lengths does not: the seam
    // between segments is dropped as a duplicate timestamp.
    const arrivals = TrajectoryPlanner.waypointArrivals(trajectory);

    set({
      executionState: ExecutionState.EXECUTING,
      executionProgress: {
        state: ExecutionState.EXECUTING,
        currentSegment: 0,
        totalSegments: trajectory.segments.length,
        currentPointInSegment: 0,
        totalPointsInSegment: 0,
        overallProgress: 0,
        elapsedTime: 0,
        estimatedTimeRemaining: trajectory.totalDuration
      }
    });

    const startTime = Date.now();
    const loopCount = get().plannerConfig.loopCount;
    const iterations = loopCount > 0 ? loopCount : 1;

    const aborted = () => executionAbortController?.signal.aborted ?? true;

    /**
     * Speed to send with one point: the joint travel it costs, over the time the
     * planner allotted it.
     *
     * One figure for the whole path cannot deliver the requested tool speed. On a
     * linear path the samples are evenly spaced in Cartesian space and wildly
     * uneven in joint space - measured on a 150 mm line, between 0.538 and 1.872
     * degrees of joint travel per 2 mm step - so a constant joint speed makes the
     * tool travel 3.5 times faster at one end of the move than the other. The
     * arm's own limits still cap it; this only stops the host asking for the
     * wrong thing.
     */
    const speedFor = (index: number): number => {
      if (index <= 0) return DEFAULT_PLANNER_CONFIG.defaultSpeed;
      const dt = allPoints[index].time - allPoints[index - 1].time;
      if (!(dt > 1e-6)) return DEFAULT_PLANNER_CONFIG.defaultSpeed;

      let travel = 0;
      for (let j = 0; j < NUM_JOINTS; j++) {
        travel = Math.max(
          travel,
          Math.abs(allPoints[index].jointAngles[j] - allPoints[index - 1].jointAngles[j])
        );
      }
      return travel / dt;
    };

    // Deliberately does not touch robotState. The firmware's STATUS line is the
    // authority on that, and setting it here raced an emergency stop: the sender
    // unwinding would overwrite ESTOPPED with IDLE, so the UI stopped showing a
    // stop the firmware still had latched.
    const stopAndReset = () => {
      set({
        executionState: ExecutionState.IDLE,
        executionProgress: { ...initialProgress }
      });
    };

    /**
     * Push one point into the firmware queue, waiting for space if the queue is
     * full.
     *
     * The firmware owns the timing now: it plans a continuous velocity profile
     * across everything it holds, so the host's job is to keep the queue fed, not
     * to pace points off its own clock. The previous version slept
     * min(dt, 200) ms between points, which both drifted from the planned
     * trajectory and kept the queue shallow, so the arm restarted from a standstill
     * at every point.
     */
    const sendPoint = (angles: JointAngles, speed: number): Promise<SendOutcome> =>
      // A full queue can hold the sender for seconds, so both the abort and the
      // pause have to be observed inside the retry loop. Checking only at the top
      // of the outer loop left the pause button doing nothing until the queue
      // drained.
      offerPoint(serialManager, angles, speed, () =>
        aborted() ? 'aborted' : executionPaused ? 'paused' : null
      );

    /** Wait for a status report, issued after `since`, that says the arm is idle. */
    const waitUntilIdle = async (since: number): Promise<void> => {
      for (let attempt = 0; attempt < 3000; attempt++) {
        if (aborted()) return;

        const { firmwareStatus, firmwareStatusAt } = get();
        if (firmwareStatus && firmwareStatusAt > since && firmwareStatus.state === 'IDLE') {
          return;
        }
        await sleep(20);
      }
    };

    try {
      for (let loop = 0; loop < iterations; loop++) {
        // The index advances in the body, so a point interrupted by a pause is
        // retried rather than skipped.
        for (let i = 0; i < allPoints.length; ) {
          if (aborted()) {
            stopAndReset();
            return;
          }

          // Pausing has to stop the arm as well as the sender: the firmware is
          // holding queued moves that would otherwise keep running. The abort is
          // issued from here rather than from pauseExecution() so that it can
          // never land while a move acknowledgement is in flight.
          if (executionPaused) {
            await serialManager.abort().catch(() => undefined);

            while (executionPaused) {
              if (aborted()) {
                stopAndReset();
                return;
              }
              await sleep(100);
            }

            if (aborted()) {
              stopAndReset();
              return;
            }
            // Nothing is skipped: this point has not been sent yet.
          }

          const outcome = await sendPoint(
            toJointAngles(allPoints[i].jointAngles),
            speedFor(i)
          );

          if (outcome === 'aborted') {
            stopAndReset();
            return;
          }
          if (outcome === 'paused') {
            // Do not advance: this point was not accepted, and the pause handler
            // at the top of the loop is what stops the arm.
            continue;
          }

          // Outputs attached to a waypoint fire once the arm has actually
          // arrived, not when its last point was accepted. The firmware holds a
          // deep queue on purpose - that is what keeps motion continuous - so an
          // acknowledgement means "queued", and a gripper commanded on it opens
          // seconds early, somewhere over the bench.
          const arrived = arrivals[i] ?? null;
          if (arrived?.setOutputs?.length) {
            await get().waitUntilIdle(Date.now(), aborted);
            if (aborted()) {
              stopAndReset();
              return;
            }
            for (const action of arrived.setOutputs) {
              await get().setOutput(action.index, action.high);
            }
          }

          const where = locateSegment(trajectory, i);
          const elapsedTime = (Date.now() - startTime) / 1000;
          const sent = loop * allPoints.length + i + 1;
          const overallProgress = (sent / (iterations * allPoints.length)) * 100;

          set({
            executionProgress: {
              state: ExecutionState.EXECUTING,
              currentSegment: where.segment,
              totalSegments: trajectory.segments.length,
              currentPointInSegment: where.pointInSegment,
              totalPointsInSegment: where.pointsInSegment,
              overallProgress,
              elapsedTime,
              estimatedTimeRemaining: Math.max(
                0,
                trajectory.totalDuration * iterations - elapsedTime
              )
            }
          });

          i++;
        }
      }

      // Every point has been accepted, but the arm is still working through the
      // queue. Wait for it to actually finish before reporting completion.
      await waitUntilIdle(Date.now());

      if (aborted()) {
        stopAndReset();
        return;
      }

      // Execution complete
      set({
        executionState: ExecutionState.COMPLETED,
        executionProgress: {
          ...get().executionProgress,
          state: ExecutionState.COMPLETED,
          overallProgress: 100
        }
      });

    } catch (error) {
      // Cancelling mid-flight rejects the acknowledgement the sender was waiting
      // on, which lands here. That is a cancel, not a fault.
      if (aborted()) {
        stopAndReset();
        return;
      }

      console.error('Trajectory execution error:', error);
      set({
        executionState: ExecutionState.ERROR,
        executionProgress: {
          ...get().executionProgress,
          state: ExecutionState.ERROR
        }
      });
    } finally {
      executionAbortController = null;
    }
  },

  pauseExecution: () => {
    executionPaused = true;
    set({
      executionState: ExecutionState.PAUSED,
      executionProgress: {
        ...get().executionProgress,
        state: ExecutionState.PAUSED
      }
    });
  },

  resumeExecution: () => {
    executionPaused = false;
    set({
      executionState: ExecutionState.EXECUTING,
      executionProgress: {
        ...get().executionProgress,
        state: ExecutionState.EXECUTING
      }
    });
  },

  cancelExecution: () => {
    if (executionAbortController) {
      executionAbortController.abort();
    }
    executionPaused = false;

    // Stop the arm too, not just the sender: the firmware is holding queued moves
    // that would otherwise keep running after the host stopped streaming. A
    // graceful abort decelerates within the acceleration limit and keeps the
    // position, unlike an emergency stop.
    const { serialManager } = get();
    if (serialManager) {
      serialManager.abort().catch(error => console.error('Abort failed:', error));
    }

    set({
      executionState: ExecutionState.IDLE,
      executionProgress: { ...initialProgress }
    });
  },

  updatePlannerConfig: (config) => {
    const newConfig = { ...get().plannerConfig, ...config };
    set({ plannerConfig: newConfig });
    trajectoryPlanner.updateConfig(newConfig);
  },

  // ===== PATH SAVE/LOAD =====

  exportPath: (name, description?) => {
    const { waypoints, plannerConfig, workObjects } = get();
    // The frames go with the path. Without them a saved file is a list of
    // numbers in coordinate systems the receiving machine has never heard of,
    // and it would run them against the robot base without complaint.
    const referenced = new Set(waypoints.map(w => w.frame).filter(Boolean));
    return {
      ...TrajectoryPlanner.exportPath(name, waypoints, plannerConfig, description),
      workObjects: workObjects.filter(o => referenced.has(o.id))
    };
  },

  importPath: (savedPath) => {
    if (!TrajectoryPlanner.validateSavedPath(savedPath)) {
      console.error('Invalid path file');
      return false;
    }

    // Frames from the file are merged in rather than replacing what is here: two
    // paths can share a fixture, and importing the second must not delete the
    // first one's frames. Same id wins from the file, because it was saved
    // alongside the points that use it.
    const incoming = savedPath.workObjects ?? [];
    const merged = [
      ...get().workObjects.filter(o => !incoming.some(i => i.id === o.id)),
      ...incoming
    ];

    set({
      waypoints: savedPath.waypoints,
      workObjects: merged,
      plannerConfig: { ...DEFAULT_PLANNER_CONFIG, ...savedPath.config },
      trajectory: null,
      trajectoryPositions: [],
      executionState: ExecutionState.IDLE,
      executionProgress: { ...initialProgress }
    });

    saveWorkObjects(merged);
    trajectoryPlanner.updateConfig(savedPath.config);
    return true;
  }
}));
