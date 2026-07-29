import { create } from 'zustand';
import { JointAngles, EndstopState, ConnectionStatus, RobotState } from '../types/robot';
import { SerialManager } from '../communication/SerialManager';
import { FirmwareState, FirmwareStatus } from '../communication/types';
import { Vector3, IKResult } from '../kinematics/types';
import { ForwardKinematics } from '../kinematics/ForwardKinematics';
import { InverseKinematics } from '../kinematics/InverseKinematics';
import {
  Waypoint,
  Trajectory,
  ExecutionState,
  ExecutionProgress,
  PathPlannerConfig,
  SavedPath
} from '../motion/types';
import { TrajectoryPlanner, DEFAULT_PLANNER_CONFIG } from '../motion/TrajectoryPlanner';

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

  /** Last STATUS line from the firmware: queue space, homed flags, trust. */
  firmwareStatus: FirmwareStatus | null;
  /** When that STATUS arrived, used to tell a fresh report from a stale one. */
  firmwareStatusAt: number;

  // Kinematics state
  currentPosition: Vector3 | null;
  targetPosition: Vector3 | null;
  ikStatus: IKResult | null;

  // UI state
  manualSpeed: number;

  // Trajectory / waypoint state
  waypoints: Waypoint[];
  trajectory: Trajectory | null;
  trajectoryPositions: Vector3[];
  executionState: ExecutionState;
  executionProgress: ExecutionProgress;
  plannerConfig: PathPlannerConfig;

  // Joint space actions
  connect: () => Promise<void>;
  disconnect: () => Promise<void>;
  setTargetAngles: (angles: Partial<JointAngles>) => void;
  moveToTarget: () => Promise<void>;
  homeJoints: (joints: string) => Promise<void>;
  enableMotors: (enable: boolean) => Promise<void>;
  emergencyStop: () => Promise<void>;
  setManualSpeed: (speed: number) => void;

  // Cartesian space actions
  setTargetPosition: (position: Vector3) => void;
  moveToPosition: (position: Vector3) => Promise<void>;
  updateCurrentPosition: () => void;

  // Waypoint actions
  addWaypoint: (waypoint: Waypoint) => void;
  removeWaypoint: (id: string) => void;
  reorderWaypoints: (fromIndex: number, toIndex: number) => void;
  updateWaypoint: (id: string, updates: Partial<Waypoint>) => void;
  clearWaypoints: () => void;
  teachCurrentPosition: (label?: string) => void;

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

// Execution control
let executionAbortController: AbortController | null = null;
let executionPaused = false;

const sleep = (ms: number) => new Promise<void>(resolve => setTimeout(resolve, ms));

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
  firmwareStatus: null,
  firmwareStatusAt: 0,
  currentPosition: null,
  targetPosition: null,
  ikStatus: null,
  manualSpeed: 30,

  // Trajectory initial state
  waypoints: [],
  trajectory: null,
  trajectoryPositions: [],
  executionState: ExecutionState.IDLE,
  executionProgress: { ...initialProgress },
  plannerConfig: { ...DEFAULT_PLANNER_CONFIG },

  // Connect to robot
  connect: async () => {
    // Reuse the existing manager if there is one, so its listeners and
    // reconnect state are not duplicated.
    const manager = get().serialManager ?? new SerialManager();
    const isNew = manager !== get().serialManager;

    set({ serialManager: manager, connectionDetail: null });

    if (isNew) {
      manager.onStateChange((state, detail) => {
        set({ connectionStatus: state, connectionDetail: detail ?? null });

        if (state === ConnectionStatus.CONNECTED) return;

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
              robotState: toRobotState(msg.data.state)
            });
            break;
          case 'ERROR':
            console.error('Robot error:', msg.data);
            break;
        }
      });
    }

    // The manager reports CONNECTING, CONNECTED and every later transition
    // through onStateChange, including a loss while this call is in flight.
    await manager.connect();
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
      executionProgress: { ...initialProgress }
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

    const ack = await serialManager.moveToAngles(targetAngles, manualSpeed);
    if (!ack.accepted) {
      console.warn('Move rejected: firmware queue full');
      return;
    }
    set({ robotState: RobotState.MOVING });
  },

  // Home joints
  homeJoints: async (joints) => {
    const { serialManager } = get();
    if (!serialManager) return;

    set({ robotState: RobotState.HOMING });
    await serialManager.homeJoints(joints);
  },

  // Enable/disable motors
  enableMotors: async (enable) => {
    const { serialManager } = get();
    if (!serialManager) return;

    await serialManager.enableMotors(enable);
    set({ motorsEnabled: enable });
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
      set({ currentPosition: fkResult.endEffectorPose.position });
    }
  },

  // Set target Cartesian position
  setTargetPosition: (position) => {
    set({ targetPosition: position });
  },

  // Move to Cartesian position using inverse kinematics
  moveToPosition: async (position) => {
    const { serialManager, currentAngles, manualSpeed } = get();
    if (!serialManager) return;

    // Use current joint angles as initial guess for IK
    const initialGuess = [
      currentAngles.J1,
      currentAngles.J2,
      currentAngles.J3,
      currentAngles.J4,
      currentAngles.J5,
      currentAngles.J6
    ];

    // Solve inverse kinematics
    const ikResult = ikSolver.solvePosition(position, initialGuess);

    // Store IK status for UI feedback
    set({ ikStatus: ikResult });

    if (!ikResult.success) {
      // The Cartesian panel renders ikStatus, including the residual, so there is
      // no need to interrupt with a modal dialog.
      console.warn('IK failed:', ikResult.error);
      return;
    }

    const targetAngles: JointAngles = {
      J1: ikResult.jointAngles[0],
      J2: ikResult.jointAngles[1],
      J3: ikResult.jointAngles[2],
      J4: ikResult.jointAngles[3],
      J5: ikResult.jointAngles[4],
      J6: ikResult.jointAngles[5]
    };

    set({ targetAngles });

    const ack = await serialManager.moveToAngles(targetAngles, manualSpeed);
    if (!ack.accepted) {
      console.warn('Move rejected: firmware queue full');
      return;
    }

    set({ robotState: RobotState.MOVING });
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
    set({
      waypoints: [],
      trajectory: null,
      trajectoryPositions: [],
      executionState: ExecutionState.IDLE,
      executionProgress: { ...initialProgress }
    });
  },

  teachCurrentPosition: (label?) => {
    const { currentAngles, currentPosition, waypoints } = get();

    if (!currentPosition) {
      get().updateCurrentPosition();
    }

    const position = get().currentPosition;
    if (!position) return;

    const newWaypoint: Waypoint = {
      id: Date.now().toString(),
      position: { ...position },
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

  // ===== TRAJECTORY ACTIONS =====

  planTrajectory: () => {
    const { waypoints, currentAngles, plannerConfig } = get();

    if (waypoints.length === 0) {
      set({ trajectory: null, trajectoryPositions: [] });
      return;
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

    const trajectory = trajectoryPlanner.planTrajectory(waypoints, startAngles);
    const positions = TrajectoryPlanner.getTrajectoryPositions(trajectory);

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
    const { trajectory, serialManager } = get();

    if (!trajectory || trajectory.segments.length === 0) {
      console.warn('No trajectory to execute');
      return;
    }

    if (!serialManager) {
      console.warn('Not connected to robot');
      return;
    }

    // Set up abort controller
    executionAbortController = new AbortController();
    executionPaused = false;

    const allPoints = TrajectoryPlanner.flattenTrajectory(trajectory);

    set({
      executionState: ExecutionState.EXECUTING,
      robotState: RobotState.MOVING,
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
    const speed = get().plannerConfig.maxJointSpeed;

    const aborted = () => executionAbortController?.signal.aborted ?? true;

    const stopAndReset = () => {
      set({
        executionState: ExecutionState.IDLE,
        robotState: RobotState.IDLE,
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
    const sendPoint = async (angles: JointAngles): Promise<boolean> => {
      for (let attempt = 0; attempt < 2000; attempt++) {
        if (aborted()) return false;

        const ack = await serialManager.moveToAngles(angles, speed);
        if (ack.accepted) return true;

        // Queue full: normal back-pressure. Wait for the arm to consume a move.
        await sleep(20);
      }
      throw new Error('Firmware motion queue never drained');
    };

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
        for (let i = 0; i < allPoints.length; i++) {
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

          const point = allPoints[i];
          const angles: JointAngles = {
            J1: point.jointAngles[0],
            J2: point.jointAngles[1],
            J3: point.jointAngles[2],
            J4: point.jointAngles[3],
            J5: point.jointAngles[4],
            J6: point.jointAngles[5]
          };

          if (!(await sendPoint(angles))) {
            stopAndReset();
            return;
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
        robotState: RobotState.IDLE,
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
        robotState: RobotState.ERROR,
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
      robotState: RobotState.IDLE,
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
    const { waypoints, plannerConfig } = get();
    return TrajectoryPlanner.exportPath(name, waypoints, plannerConfig, description);
  },

  importPath: (savedPath) => {
    if (!TrajectoryPlanner.validateSavedPath(savedPath)) {
      console.error('Invalid path file');
      return false;
    }

    set({
      waypoints: savedPath.waypoints,
      plannerConfig: { ...DEFAULT_PLANNER_CONFIG, ...savedPath.config },
      trajectory: null,
      trajectoryPositions: [],
      executionState: ExecutionState.IDLE,
      executionProgress: { ...initialProgress }
    });

    trajectoryPlanner.updateConfig(savedPath.config);
    return true;
  }
}));
