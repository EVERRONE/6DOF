import { create } from 'zustand';
import { JointAngles, EndstopState, ConnectionStatus, RobotState } from '../types/robot';
import { SerialManager } from '../communication/SerialManager';
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
  serialManager: SerialManager | null;

  // Robot state
  robotState: RobotState;
  currentAngles: JointAngles;
  targetAngles: JointAngles;
  endstopState: EndstopState;
  motorsEnabled: boolean;

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
  serialManager: null,
  robotState: RobotState.IDLE,
  currentAngles: { J1: 0, J2: 0, J3: 0, J4: 0, J5: 0, J6: 0 },
  targetAngles: { J1: 0, J2: 0, J3: 0, J4: 0, J5: 0, J6: 0 },
  endstopState: { J1: false, J2: false, J3: false, J4: false, J5: false, J6: false },
  motorsEnabled: false,
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
    const manager = new SerialManager();

    set({ connectionStatus: ConnectionStatus.CONNECTING });

    try {
      await manager.connect();

      // Subscribe to messages
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
          case 'HOMED':
            set({ robotState: RobotState.IDLE });
            break;
          case 'ERROR':
            set({ robotState: RobotState.ERROR });
            console.error('Robot error:', msg.data);
            break;
        }
      });

      set({
        serialManager: manager,
        connectionStatus: ConnectionStatus.CONNECTED
      });

      // Query initial position
      await manager.queryPosition();

    } catch (error) {
      set({ connectionStatus: ConnectionStatus.ERROR });
      throw error;
    }
  },

  // Disconnect
  disconnect: async () => {
    const { serialManager } = get();
    if (serialManager) {
      await serialManager.disconnect();
    }
    set({
      serialManager: null,
      connectionStatus: ConnectionStatus.DISCONNECTED
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

    set({ robotState: RobotState.MOVING });
    await serialManager.moveToAngles(targetAngles, manualSpeed);
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
    const { serialManager, executionState } = get();

    // Cancel any running trajectory
    if (executionState === ExecutionState.EXECUTING || executionState === ExecutionState.PAUSED) {
      get().cancelExecution();
    }

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

    if (ikResult.success) {
      // Convert solution to JointAngles format
      const targetAngles: JointAngles = {
        J1: ikResult.jointAngles[0],
        J2: ikResult.jointAngles[1],
        J3: ikResult.jointAngles[2],
        J4: ikResult.jointAngles[3],
        J5: ikResult.jointAngles[4],
        J6: ikResult.jointAngles[5]
      };

      // Send move command to robot
      set({
        robotState: RobotState.MOVING,
        targetAngles: targetAngles
      });

      await serialManager.moveToAngles(targetAngles, manualSpeed);
    } else {
      console.error('IK failed:', ikResult.error);
      alert(`Inverse kinematics failed: ${ikResult.error}`);
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

    try {
      for (let loop = 0; loop < iterations; loop++) {
        for (let i = 0; i < allPoints.length; i++) {
          // Check for abort
          if (executionAbortController.signal.aborted) {
            set({
              executionState: ExecutionState.IDLE,
              robotState: RobotState.IDLE,
              executionProgress: { ...initialProgress }
            });
            return;
          }

          // Wait while paused
          while (executionPaused) {
            if (executionAbortController.signal.aborted) {
              set({
                executionState: ExecutionState.IDLE,
                robotState: RobotState.IDLE,
                executionProgress: { ...initialProgress }
              });
              return;
            }
            await new Promise(resolve => setTimeout(resolve, 100));
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

          // Send to robot
          await serialManager.moveToAngles(angles, get().plannerConfig.maxJointSpeed);

          // Determine which segment we're in
          let segIdx = 0;
          let pointInSeg = i;
          let pointsInSeg = allPoints.length;
          for (let s = 0; s < trajectory.segments.length; s++) {
            if (pointInSeg < trajectory.segments[s].points.length) {
              segIdx = s;
              pointsInSeg = trajectory.segments[s].points.length;
              break;
            }
            pointInSeg -= trajectory.segments[s].points.length;
          }

          const elapsedTime = (Date.now() - startTime) / 1000;
          const overallProgress = ((loop * allPoints.length + i + 1) / (iterations * allPoints.length)) * 100;

          set({
            executionProgress: {
              state: ExecutionState.EXECUTING,
              currentSegment: segIdx,
              totalSegments: trajectory.segments.length,
              currentPointInSegment: pointInSeg,
              totalPointsInSegment: pointsInSeg,
              overallProgress,
              elapsedTime,
              estimatedTimeRemaining: Math.max(0, trajectory.totalDuration * iterations - elapsedTime)
            }
          });

          // Wait for next point timing
          if (i < allPoints.length - 1) {
            const nextTime = allPoints[i + 1].time;
            const dt = (nextTime - point.time) * 1000; // ms
            if (dt > 0) {
              await new Promise(resolve => setTimeout(resolve, Math.min(dt, 200)));
            }
          }
        }
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
