import { create } from 'zustand';
import {
  JointAngles,
  EndstopState,
  HomedState,
  ConnectionStatus,
  RobotState,
  FirmwareConfig
} from '../types/robot';
import { SerialManager } from '../communication/SerialManager';
import {
  PlannerWorkerRequest,
  PlannerWorkerResponse
} from '../communication/types';
import {
  CartesianMode,
  IKAngleFrame,
  IKEngineMode,
  IKLimitsSource,
  IKPlanningBudget,
  IKResult,
  IKSolveContext,
  IKSolveOptions,
  IKSolveProfile,
  IKSolveStage,
  IKTrackingMode,
  IKSampleDiagnostic,
  Rotation3,
  Vector3
} from '../kinematics/types';
import { ForwardKinematics } from '../kinematics/ForwardKinematics';
import {
  getEffectiveUrdfOffsets,
  logicalToUrdfAngles,
  urdfToLogicalAngles
} from '../kinematics/angleMapping';
import { HybridIKSolver } from '../kinematics/HybridIKSolver';
import { InverseKinematics } from '../kinematics/InverseKinematics';
import { QuaternionMath } from '../kinematics/QuaternionMath';
import {
  classifyReachability,
  getDefaultReachabilityAtlas,
  isReachabilityAtlasReady
} from '../kinematics/reachabilityAtlas';
import { logJointSensitivity } from '../kinematics/diagnostics';
import { summarizeIKResult } from '../kinematics/qualityMetrics';
import {
  CartesianInterpolationResult,
  Waypoint,
  Trajectory,
  ExecutionState,
  ExecutionProgress,
  PathPlannerConfig,
  SavedPath
} from '../motion/types';
import { TrajectoryPlanner, DEFAULT_PLANNER_CONFIG } from '../motion/TrajectoryPlanner';
import { IKRuntimeService } from '../services/cartesian/IKRuntimeService';
import { TrajectoryExecutionService } from '../services/cartesian/TrajectoryExecutionService';
import { CartesianPlanningService } from '../services/cartesian/CartesianPlanningService';
import {
  CartesianPlannerDiagnosticEvent,
  NormalizedConstraintContract,
  Stage2CandidateDiagnostic,
  Stage2SelectionPolicy
} from '../services/cartesian/types';
import { ConstraintAdapterService } from '../services/cartesian/ConstraintAdapterService';
import {
  computeAdaptiveStage2TimeoutMs,
  isStage2TimeoutMessage
} from '../services/cartesian/Stage2RefinementPolicy';

interface RobotStore {
  // Connection
  connectionStatus: ConnectionStatus;
  serialManager: SerialManager | null;

  // Robot state
  robotState: RobotState;
  currentAngles: JointAngles;
  targetAngles: JointAngles;
  endstopState: EndstopState;
  homedJoints: HomedState;
  motorsEnabled: boolean;
  moveInProgress: boolean;
  moveTargetSnapshot: JointAngles | null;
  moveStableCount: number;
  firmwareConfig: FirmwareConfig | null;

  // Kinematics state
  kinematicsFrameReady: boolean;
  currentPosition: Vector3 | null;
  targetPosition: Vector3 | null;
  ikStatus: IKResult | null;
  ikEngineMode: IKEngineMode;
  ikSolveProfile: IKSolveProfile;
  ikPrimaryMode: 'analytic_first' | 'numeric_fallback';
  branchLockEnabled: boolean;
  resolvedRateEnabled: boolean;
  collisionCheckEnabled: boolean;
  reachabilityAtlasReady: boolean;
  motionKernelStatus: {
    tickJitterUs: number;
    queueUnderrun: number;
    stepOverrun: number;
    lastUpdated: number | null;
  };
  ikDiagnostics: {
    attempts: number;
    bestStage: string;
    bestResidualMm: number;
    branchId: string;
    seedIndex: number;
  } | null;
  cartesianMode: CartesianMode;
  queueProgress: { pointIndex: number; elapsedMs: number; count: number };
  planningState: 'idle' | 'stage1_fast' | 'stage2_refine' | 'ready' | 'failed';
  planningLatencyMs: number;
  planningNotes: string[];
  planningDiagnostics: CartesianPlannerDiagnosticEvent[];

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
  isCartesianReady: () => boolean;
  requestConfig: () => Promise<void>;
  requestHomePose: () => Promise<void>;
  setCalibration: (joint: number, scale: number, offset: number) => Promise<void>;
  zeroCalibration: (joint: number, logicalDeg: number) => Promise<void>;
  saveCalibration: () => Promise<void>;
  loadCalibration: () => Promise<void>;
  resetCalibration: () => Promise<void>;
  jogRelative: (joint: number, deltaDeg: number, speed: number) => Promise<void>;
  setHomePoseEnabled: (enabled: boolean) => Promise<void>;
  setHomePoseJoint: (joint: number, angleDeg: number) => Promise<void>;
  setHomePoseAll: (j2: number, j3: number, j4: number, j5: number) => Promise<void>;
  setHomePoseSpeed: (speedDegS: number) => Promise<void>;
  saveHomePose: () => Promise<void>;
  loadHomePose: () => Promise<void>;
  resetHomePose: () => Promise<void>;

  // Cartesian space actions
  setTargetPosition: (position: Vector3 | null) => void;
  cancelCartesianPlanning: (reason?: string) => void;
  moveToPosition: (position: Vector3, targetOrientation?: Rotation3) => Promise<void>;
  jogCartesian: (delta: { dx?: number; dy?: number; dz?: number; rx?: number; ry?: number; rz?: number; frame?: 'world' | 'tool' }) => Promise<void>;
  updateCurrentPosition: () => void;
  setCartesianMode: (mode: CartesianMode) => void;
  setIKEngineMode: (mode: IKEngineMode) => void;
  setIKSolveProfile: (profile: IKSolveProfile) => void;
  setIKPrimaryMode: (mode: 'analytic_first' | 'numeric_fallback') => void;
  setBranchLockEnabled: (enabled: boolean) => void;
  setResolvedRateEnabled: (enabled: boolean) => void;
  setCollisionCheckEnabled: (enabled: boolean) => void;
  requestMotionKernelStatus: () => Promise<void>;

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

const CARTESIAN_DIRECT_SPEED_MM_S = 20;
const CARTESIAN_DIRECT_MAX_SPEED_MM_S = 80;
const CARTESIAN_DIRECT_ACCEL_MM_S2 = 60;
const CARTESIAN_DIRECT_POINTS_PER_SEC = 100;
const CARTESIAN_TARGET_MIN_POINTS_PER_SEC = 25;
const CARTESIAN_ABSOLUTE_MIN_POINTS_PER_SEC = 8;
const CARTESIAN_INVALID_TARGET_RESIDUAL_M = 0.003;
const CARTESIAN_SEED_DELTA_DEG = 2.5;
const CARTESIAN_STAGE2_MIN_POINTS_PER_SEC = 25;
const FEATURE_IK_ENGINE_V2 = process.env.REACT_APP_IK_ENGINE_V2 !== '0';
const FEATURE_REACHABILITY_ATLAS_V1 = process.env.REACT_APP_REACHABILITY_ATLAS_V1 !== '0';
const FEATURE_MOTION_KERNEL_V2 = process.env.REACT_APP_MOTION_KERNEL_V2 !== '0';
const FEATURE_IK_ANALYTIC_PRIMARY_V1 = process.env.REACT_APP_IK_ANALYTIC_PRIMARY_V1 !== '0';
const FEATURE_IK_RESOLVED_RATE_V1 = process.env.REACT_APP_IK_RESOLVED_RATE_V1 !== '0';
const FEATURE_IK_COLLISION_CHECK_V1 = process.env.REACT_APP_IK_COLLISION_CHECK_V1 !== '0';
const FEATURE_IK_DIAGNOSTICS_LOG = process.env.REACT_APP_IK_DIAGNOSTICS_LOG === '1';
const FEATURE_INDUSTRIAL_PROFILE_V1 = process.env.REACT_APP_INDUSTRIAL_PROFILE_V1 === '1';

const DEFAULT_IK_PLANNING_BUDGET: IKPlanningBudget = {
  stage1BudgetMs: 500,
  stage2MaxMs: 6000,
  // 30 iterations gives the DLS solver enough budget to escape the near-tolerance
  // damping death spiral that occurs when the arm approaches the workspace boundary.
  trackingMaxIterations: 30
};
const STAGE2_TIMEOUT_MIN_MS = 6000;
const STAGE2_TIMEOUT_MAX_MS = 20000;
const STAGE2_SELECTION_POLICY: Stage2SelectionPolicy = 'first_success_plus_one_probe';
const STAGE2_MAX_CANDIDATE_ATTEMPTS = 4;
const STAGE2_MAX_ADDITIONAL_PROBES_AFTER_SUCCESS = 1;

const clampToRange = (value: number, min: number, max: number): number => (
  Math.min(max, Math.max(min, value))
);

const estimateMinimumJerkDuration = (
  distanceM: number,
  speedMmS: number,
  accelMmS2: number
): number => {
  const speedMs = Math.max(0.001, speedMmS / 1000);
  const accelMs = Math.max(0.001, accelMmS2 / 1000);
  const tVel = (1.875 * distanceM) / speedMs;
  const tAcc = Math.sqrt((5.8 * distanceM) / accelMs);
  return Math.max(0.25, tVel, tAcc);
};

// Trajectory planner instance
let trajectoryPlanner = new TrajectoryPlanner();
const hybridEndpointSolver = new HybridIKSolver();
const legacyEndpointSolver = new InverseKinematics();
let activePlannerWorker: Worker | null = null;
let activePlannerRequestId = 0;
type PlannerWorkerFactory = () => Worker;
let plannerWorkerFactoryPromise: Promise<PlannerWorkerFactory | null> | null = null;

const toUrdfLimitVectors = (
  constraints: NormalizedConstraintContract
): { min: number[]; max: number[] } => ({
  min: constraints.joints.map((joint) => joint.urdfMinDeg),
  max: constraints.joints.map((joint) => joint.urdfMaxDeg)
});

const applyConstraintContractToSolvers = (
  constraints: NormalizedConstraintContract | null
): void => {
  if (!constraints) return;
  const limits = toUrdfLimitVectors(constraints);
  legacyEndpointSolver.setJointLimitsDeg(limits.min, limits.max);
  hybridEndpointSolver.setJointLimitsDeg(limits.min, limits.max);
  trajectoryPlanner.setJointLimitsDeg(limits.min, limits.max);
};

const stopPlannerWorker = (): void => {
  if (activePlannerWorker) {
    activePlannerWorker.terminate();
    activePlannerWorker = null;
  }
};

const getPlannerWorkerFactory = async (): Promise<PlannerWorkerFactory | null> => {
  if (typeof Worker === 'undefined' || process.env.NODE_ENV === 'test') {
    return null;
  }

  if (!plannerWorkerFactoryPromise) {
    plannerWorkerFactoryPromise = import('../workers/createPlannerWorker')
      .then((mod) => mod.createCartesianPlannerWorker)
      .catch((error) => {
        console.warn('Planner worker unavailable, falling back to main-thread refinement:', error);
        return null;
      });
  }

  return plannerWorkerFactoryPromise;
};

interface Stage2WorkerRequestData {
  requestId: number;
  startAngles: number[];
  targetPosition: Vector3;
  targetOrientation?: Rotation3;
  urdfOffsetsDeg: number[];
  jointLimitsUrdfDeg: {
    min: number[];
    max: number[];
  };
  cartesianMode: CartesianMode;
  ikEngineMode: IKEngineMode;
  speedMmS: number;
  accelerationMmS2: number;
  minPointsPerSecond: number;
  maxPointsPerSecond: number;
  queuePointBudget: number;
  strictIkOptions: Partial<IKSolveOptions>;
  timeoutMs: number;
  deadlineMs: number;
  selectionPolicy: Stage2SelectionPolicy;
  maxCandidateAttempts: number;
  maxAdditionalProbesAfterSuccess: number;
  onProgress: (msg: string, elapsedMs: number) => void;
}

interface Stage2PlannerSuccess {
  interpolation: CartesianInterpolationResult;
  chosenPps: number;
  notes: string[];
  elapsedMs: number;
  diagnostics: Stage2CandidateDiagnostic[];
}

class Stage2PlannerFailure extends Error {
  readonly failureCategory?: IKResult['failureCategory'];
  readonly notes: string[];
  readonly diagnostics: Stage2CandidateDiagnostic[];
  readonly timedOut: boolean;

  constructor(
    message: string,
    options?: {
      failureCategory?: IKResult['failureCategory'];
      notes?: string[];
      diagnostics?: Stage2CandidateDiagnostic[];
      timedOut?: boolean;
    }
  ) {
    super(message);
    this.name = 'Stage2PlannerFailure';
    this.failureCategory = options?.failureCategory;
    this.notes = options?.notes || [];
    this.diagnostics = options?.diagnostics || [];
    this.timedOut = Boolean(options?.timedOut);
  }
}

const buildPlannerPpsCandidates = (minPps: number, maxPps: number): number[] => {
  const clampedMin = Math.max(1, Math.floor(minPps));
  const clampedMax = Math.max(clampedMin, Math.floor(maxPps));
  if (clampedMin >= clampedMax) {
    return [clampedMin];
  }

  const candidates = new Set<number>();
  candidates.add(clampedMin);
  candidates.add(clampedMax);

  let probe = clampedMin;
  while (probe < clampedMax) {
    probe = Math.min(clampedMax, Math.round(probe * 1.35));
    candidates.add(probe);
    if (probe === clampedMax) break;
  }

  return Array.from(candidates).sort((a, b) => a - b);
};

const isStrictInterpolationConverged = (
  interpolation: CartesianInterpolationResult,
  tolerancePositionM: number,
  toleranceOrientationRad: number,
  toleranceWeighted: number
): boolean => {
  if (!interpolation.success || !interpolation.lastIK?.quality) {
    return false;
  }

  const quality = interpolation.lastIK.quality;
  return (
    quality.positionResidualM <= tolerancePositionM &&
    quality.orientationResidualRad <= toleranceOrientationRad &&
    quality.weightedResidual <= toleranceWeighted
  );
};

const runStage2PlannerInline = async (
  data: Stage2WorkerRequestData
): Promise<Stage2PlannerSuccess> => {
  const startedAt = Date.now();
  const planner = new TrajectoryPlanner();
  planner.setUrdfOffsets(data.urdfOffsetsDeg);
  planner.setJointLimitsDeg(data.jointLimitsUrdfDeg.min, data.jointLimitsUrdfDeg.max);

  const ppsCandidates = buildPlannerPpsCandidates(data.minPointsPerSecond, data.maxPointsPerSecond);
  const notes: string[] = ['Worker unavailable: using main-thread strict refinement.'];
  const candidateDiagnostics: Stage2CandidateDiagnostic[] = [];
  let bestFailureMessage = 'Stage2 refinement failed';
  let bestFailureCategory: IKResult['failureCategory'] | undefined;
  const strictSuccesses: Array<{
    interpolation: CartesianInterpolationResult;
    chosenPps: number;
    weightedResidual: number;
    candidateIndex: number;
  }> = [];
  let attemptsWithoutStrictSuccess = 0;
  let firstStrictSuccessSeen = false;
  let additionalProbeCount = 0;

  for (let candidateIndex = 0; candidateIndex < ppsCandidates.length; candidateIndex++) {
    const pps = ppsCandidates[candidateIndex];
    if (data.requestId !== activePlannerRequestId) {
      throw new Error('Planning cancelled');
    }

    const now = Date.now();
    if (now > data.deadlineMs) {
      throw new Stage2PlannerFailure(
        `Stage2 refinement timeout after ${data.timeoutMs}ms`,
        {
          failureCategory: 'stage2_timeout',
          notes,
          diagnostics: candidateDiagnostics,
          timedOut: true
        }
      );
    }

    if (!firstStrictSuccessSeen && attemptsWithoutStrictSuccess >= data.maxCandidateAttempts) {
      notes.push(`Stopped after ${data.maxCandidateAttempts} attempts without strict success`);
      break;
    }
    if (firstStrictSuccessSeen && additionalProbeCount >= data.maxAdditionalProbesAfterSuccess) {
      notes.push('Stopped after one extra probe following first strict success');
      break;
    }

    const elapsedMs = Date.now() - startedAt;
    data.onProgress(`Refining strict path at ${pps}Hz`, elapsedMs);
    const solveStartedAt = Date.now();
    const interpolation = planner.planPoseLockedCartesianMove(
      data.startAngles,
      data.targetPosition,
      data.targetOrientation,
      {
        speedMmS: data.speedMmS,
        accelerationMmS2: data.accelerationMmS2,
        pointsPerSecond: pps,
        ikOptions: data.strictIkOptions,
        ikEngineMode: data.ikEngineMode,
        deadlineMs: data.deadlineMs
      }
    );
    const solveDurationMs = Date.now() - solveStartedAt;

    const pointCount = interpolation.segment.points.length;
    const remainingBudgetMs = Math.max(0, data.deadlineMs - Date.now());
    const strictOk = isStrictInterpolationConverged(
      interpolation,
      data.strictIkOptions.tolerancePositionM ?? 0.0015,
      data.strictIkOptions.toleranceOrientationRad ?? Number.POSITIVE_INFINITY,
      data.strictIkOptions.toleranceWeighted ?? 0.01
    );
    const timedOut = Boolean(
      interpolation.timedOut ||
      interpolation.lastIK?.failureCategory === 'stage2_timeout'
    );
    const diagnostic: Stage2CandidateDiagnostic = {
      candidateIndex,
      pointsPerSecond: pps,
      sampleCount: pointCount,
      solveDurationMs,
      strictConverged: interpolation.success && strictOk,
      remainingBudgetMs
    };

    if (pointCount > data.queuePointBudget) {
      const reason = `${pps}Hz skipped: ${pointCount} points exceeds budget ${data.queuePointBudget}`;
      diagnostic.failureReason = reason;
      candidateDiagnostics.push(diagnostic);
      notes.push(reason);
      bestFailureMessage = 'Stage2 refinement exceeded queue point budget';
      bestFailureCategory = 'queue_upload_failed';
      break;
    }

    if (timedOut) {
      const reason = interpolation.error || `Stage2 refinement timeout at ${pps}Hz`;
      diagnostic.failureReason = reason;
      candidateDiagnostics.push(diagnostic);
      notes.push(`${pps}Hz timeout: ${reason}`);
      throw new Stage2PlannerFailure(reason, {
        failureCategory: 'stage2_timeout',
        notes,
        diagnostics: candidateDiagnostics,
        timedOut: true
      });
    }

    if (interpolation.success && strictOk) {
      candidateDiagnostics.push(diagnostic);
      strictSuccesses.push({
        interpolation,
        chosenPps: pps,
        weightedResidual: interpolation.lastIK?.quality?.weightedResidual ?? Number.POSITIVE_INFINITY,
        candidateIndex
      });
      notes.push(`Strict pass at ${pps}Hz`);
      if (!firstStrictSuccessSeen) {
        firstStrictSuccessSeen = true;
      } else {
        additionalProbeCount += 1;
      }
      continue;
    }

    const reason = interpolation.error || interpolation.lastIK?.error || 'No strict solution found';
    diagnostic.failureReason = reason;
    candidateDiagnostics.push(diagnostic);
    bestFailureMessage = reason;
    bestFailureCategory = interpolation.lastIK?.failureCategory;
    notes.push(`${pps}Hz failed: ${reason}`);
    if (!firstStrictSuccessSeen) {
      attemptsWithoutStrictSuccess += 1;
    } else {
      additionalProbeCount += 1;
    }
  }

  if (strictSuccesses.length > 0) {
    let selected = strictSuccesses[0];
    for (let i = 1; i < strictSuccesses.length; i++) {
      const current = strictSuccesses[i];
      const currentResidual = Number.isFinite(current.weightedResidual)
        ? current.weightedResidual
        : Number.POSITIVE_INFINITY;
      const selectedResidual = Number.isFinite(selected.weightedResidual)
        ? selected.weightedResidual
        : Number.POSITIVE_INFINITY;
      if (
        currentResidual < selectedResidual ||
        (Math.abs(currentResidual - selectedResidual) < 1e-12 && current.chosenPps > selected.chosenPps)
      ) {
        selected = current;
      }
    }

    for (let i = 0; i < candidateDiagnostics.length; i++) {
      if (candidateDiagnostics[i].candidateIndex === selected.candidateIndex) {
        candidateDiagnostics[i].selected = true;
        candidateDiagnostics[i].selectionReason = data.selectionPolicy;
      }
    }

    notes.push(`Stage2 strict refinement complete at ${selected.chosenPps}Hz`);
    return {
      interpolation: selected.interpolation,
      chosenPps: selected.chosenPps,
      notes,
      elapsedMs: Date.now() - startedAt,
      diagnostics: candidateDiagnostics
    };
  }

  throw new Stage2PlannerFailure(`${bestFailureMessage} (${notes.join(' | ')})`, {
    failureCategory: bestFailureCategory,
    notes,
    diagnostics: candidateDiagnostics,
    timedOut: bestFailureCategory === 'stage2_timeout' || isStage2TimeoutMessage(bestFailureMessage)
  });
};

const runStage2PlannerWorker = async (
  data: Stage2WorkerRequestData
): Promise<Stage2PlannerSuccess> => {
  const workerFactory = await getPlannerWorkerFactory();
  if (!workerFactory) {
    return runStage2PlannerInline(data);
  }

  stopPlannerWorker();
  const worker = workerFactory();
  activePlannerWorker = worker;
  const startedAt = Date.now();

  return new Promise((resolve, reject) => {
    const timer = setTimeout(() => {
      stopPlannerWorker();
      reject(new Stage2PlannerFailure(`Stage2 refinement timeout after ${data.timeoutMs}ms`, {
        failureCategory: 'stage2_timeout',
        notes: [`Stage2 refinement timeout after ${data.timeoutMs}ms`],
        timedOut: true
      }));
    }, data.timeoutMs + 300);

    const cleanup = (): void => {
      clearTimeout(timer);
      if (worker === activePlannerWorker) {
        activePlannerWorker = null;
      }
      worker.onmessage = null;
      worker.onerror = null;
    };

    worker.onerror = (event) => {
      cleanup();
      reject(new Error(event.message || 'Planner worker failed'));
    };

    worker.onmessage = (event: MessageEvent<PlannerWorkerResponse>) => {
      const message = event.data;
      if (!message || message.requestId !== data.requestId) return;

      if (message.type === 'progress') {
        data.onProgress(message.message, message.elapsedMs);
        return;
      }

      cleanup();

      if (message.type === 'result') {
        resolve({
          interpolation: message.interpolation,
          chosenPps: message.chosenPps,
          notes: message.notes,
          elapsedMs: Date.now() - startedAt,
          diagnostics: message.diagnostics || []
        });
        return;
      }

      if (message.type === 'error') {
        reject(new Stage2PlannerFailure(message.error, {
          failureCategory: message.failureCategory as IKResult['failureCategory'] | undefined,
          notes: message.notes,
          diagnostics: message.diagnostics || [],
          timedOut: message.failureCategory === 'stage2_timeout' || isStage2TimeoutMessage(message.error)
        }));
        return;
      }

      reject(new Error('Unexpected planner worker response'));
    };

    const request: PlannerWorkerRequest = {
      type: 'plan_stage2',
      requestId: data.requestId,
      payload: {
        startAngles: data.startAngles,
        targetPosition: data.targetPosition,
        targetOrientation: data.targetOrientation,
        urdfOffsetsDeg: data.urdfOffsetsDeg,
        jointLimitsUrdfDeg: data.jointLimitsUrdfDeg,
        cartesianMode: data.cartesianMode,
        ikEngineMode: data.ikEngineMode,
        speedMmS: data.speedMmS,
        accelerationMmS2: data.accelerationMmS2,
        minPointsPerSecond: data.minPointsPerSecond,
        maxPointsPerSecond: data.maxPointsPerSecond,
        queuePointBudget: data.queuePointBudget,
        strictIkOptions: data.strictIkOptions,
        deadlineMs: data.deadlineMs,
        selectionPolicy: data.selectionPolicy,
        maxCandidateAttempts: data.maxCandidateAttempts,
        maxAdditionalProbesAfterSuccess: data.maxAdditionalProbesAfterSuccess
      }
    };
    worker.postMessage(request);
  });
};

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

type JointKey = keyof JointAngles;
const JOINT_KEYS: JointKey[] = ['J1', 'J2', 'J3', 'J4', 'J5', 'J6'];

const createZeroAngles = (): JointAngles => ({
  J1: 0,
  J2: 0,
  J3: 0,
  J4: 0,
  J5: 0,
  J6: 0
});

const createFalseHomedState = (): HomedState => ({
  J1: false,
  J2: false,
  J3: false,
  J4: false,
  J5: false,
  J6: false
});

const parseHomingSelection = (joints: string): JointKey[] => {
  const normalized = joints.trim().toUpperCase();
  if (normalized === 'ALL') {
    return ['J2', 'J3', 'J4', 'J5'];
  }

  return normalized
    .split('')
    .map((char) => Number.parseInt(char, 10))
    .filter((jointNum) => Number.isInteger(jointNum) && jointNum >= 1 && jointNum <= 6)
    .map((jointNum) => JOINT_KEYS[jointNum - 1]);
};

const getHomePoseJoints = (config: FirmwareConfig | null): number[] => {
  const fallback = [0, 0, 0, 0, 0, 0];
  const joints = config?.homePose?.jointsDeg;
  if (!Array.isArray(joints) || joints.length !== 6) return fallback;
  return joints.map((value) => (Number.isFinite(value) ? value : 0));
};


export const useRobotStore = create<RobotStore>((set, get) => ({
  // Initial state
  connectionStatus: ConnectionStatus.DISCONNECTED,
  serialManager: null,
  robotState: RobotState.IDLE,
  currentAngles: createZeroAngles(),
  targetAngles: createZeroAngles(),
  endstopState: { J1: false, J2: false, J3: false, J4: false, J5: false, J6: false },
  homedJoints: createFalseHomedState(),
  motorsEnabled: false,
  moveInProgress: false,
  moveTargetSnapshot: null,
  moveStableCount: 0,
  firmwareConfig: null,
  kinematicsFrameReady: false,
  currentPosition: null,
  targetPosition: null,
  ikStatus: null,
  ikEngineMode: FEATURE_IK_ENGINE_V2 ? 'hybrid_constrained_v2' : 'legacy_dls_v1',
  ikSolveProfile: FEATURE_INDUSTRIAL_PROFILE_V1 ? 'balanced' : 'smooth',
  ikPrimaryMode: FEATURE_IK_ANALYTIC_PRIMARY_V1 ? 'analytic_first' : 'numeric_fallback',
  branchLockEnabled: true,
  resolvedRateEnabled: FEATURE_IK_RESOLVED_RATE_V1 && !FEATURE_INDUSTRIAL_PROFILE_V1,
  collisionCheckEnabled: FEATURE_IK_COLLISION_CHECK_V1,
  reachabilityAtlasReady: FEATURE_REACHABILITY_ATLAS_V1 && isReachabilityAtlasReady(),
  motionKernelStatus: {
    tickJitterUs: 0,
    queueUnderrun: 0,
    stepOverrun: 0,
    lastUpdated: null
  },
  ikDiagnostics: null,
  cartesianMode: 'pose_lock',
  queueProgress: { pointIndex: 0, elapsedMs: 0, count: 0 },
  planningState: 'idle',
  planningLatencyMs: 0,
  planningNotes: [],
  planningDiagnostics: [],
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
            {
              let moveJustCompleted = false;

              set((state) => {
                let nextState: Partial<RobotStore> = { currentAngles: msg.data };

                if (state.moveInProgress && state.moveTargetSnapshot && state.robotState === RobotState.MOVING) {
                  const tolerance = 0.2;
                  let withinTolerance = true;

                  for (const joint of JOINT_KEYS) {
                    const target = state.moveTargetSnapshot[joint];
                    const actual = msg.data[joint];
                    if (Math.abs(target - actual) > tolerance) {
                      withinTolerance = false;
                      break;
                    }
                  }

                  const stableCount = withinTolerance ? state.moveStableCount + 1 : 0;

                  if (stableCount >= 3) {
                    moveJustCompleted = true;
                    nextState = {
                      ...nextState,
                      robotState: RobotState.IDLE,
                      moveInProgress: false,
                      moveTargetSnapshot: null,
                      moveStableCount: 0
                    };
                  } else {
                    nextState = {
                      ...nextState,
                      moveStableCount: stableCount
                    };
                  }
                }

                return nextState;
              });

              // Compute forward kinematics to update XYZ position
              get().updateCurrentPosition();

              // Snap target marker to current position once motion has settled.
              if (moveJustCompleted) {
                const settledPose = get().currentPosition;
                if (settledPose) {
                  set({ targetPosition: { ...settledPose } });
                }
              }
            }
            break;
          case 'ENDSTOP':
            set({ endstopState: msg.data });
            break;
          case 'HOMED':
            {
              set((state) => {
                const jointIndex = typeof msg.data === 'number' ? msg.data - 1 : null;
                if (jointIndex !== null && jointIndex >= 0 && jointIndex < JOINT_KEYS.length) {
                  const jointKey = JOINT_KEYS[jointIndex];
                  return {
                    robotState: RobotState.IDLE,
                    targetAngles: { ...state.targetAngles, [jointKey]: 0 },
                    homedJoints: { ...state.homedJoints, [jointKey]: true }
                  };
                }
                return { robotState: RobotState.IDLE };
              });

              const { serialManager } = get();
              if (serialManager) {
                serialManager.queryPosition().catch((error) => {
                  console.warn('Failed to query position after homing:', error);
                });
              }
            }
            break;
          case 'CFG':
            {
              const cfg = msg.data as FirmwareConfig;
              const constraints = ConstraintAdapterService.fromFirmwareConfig(cfg);
              set({ firmwareConfig: cfg });
              trajectoryPlanner.setUrdfOffsets(constraints?.urdfOffsetsDeg ?? getEffectiveUrdfOffsets(cfg));
              applyConstraintContractToSolvers(constraints);
              get().updateCurrentPosition();
              if (FEATURE_MOTION_KERNEL_V2 && cfg?.capabilities?.motionKernelDiag) {
                manager.queryMotionKernel().catch((error) => {
                  console.warn('Failed to query motion kernel after CFG:', error);
                });
              }
            }
            break;
          case 'HP':
            {
              set((state) => {
                if (!state.firmwareConfig) return {};
                return {
                  firmwareConfig: {
                    ...state.firmwareConfig,
                    homePose: msg.data
                  }
                };
              });

              const { firmwareConfig } = get();
              const constraints = ConstraintAdapterService.fromFirmwareConfig(firmwareConfig);
              trajectoryPlanner.setUrdfOffsets(
                constraints?.urdfOffsetsDeg ?? getEffectiveUrdfOffsets(firmwareConfig)
              );
              applyConstraintContractToSolvers(constraints);
              get().updateCurrentPosition();
            }
            break;
          case 'HOMEPOSE_REACHED':
            {
              set((state) => {
                const homePose = getHomePoseJoints(state.firmwareConfig);
                return {
                  robotState: RobotState.IDLE,
                  targetAngles: {
                    ...state.targetAngles,
                    J2: homePose[1],
                    J3: homePose[2],
                    J4: homePose[3],
                    J5: homePose[4]
                  }
                };
              });

              const { serialManager } = get();
              if (serialManager) {
                serialManager.queryPosition().catch((error) => {
                  console.warn('Failed to query position after home pose:', error);
                });
              }
            }
            break;
          case 'TQ_READY':
            set((state) => ({
              queueProgress: {
                ...state.queueProgress,
                count: msg.data?.count ?? state.queueProgress.count
              }
            }));
            break;
          case 'TQ_STAT':
            set((state) => ({
              queueProgress: {
                pointIndex: msg.data?.pointIndex ?? state.queueProgress.pointIndex,
                elapsedMs: msg.data?.elapsedMs ?? state.queueProgress.elapsedMs,
                count: msg.data?.count ?? state.queueProgress.count
              }
            }));
            break;
          case 'TQ_PROG':
            set((state) => ({
              robotState: RobotState.MOVING,
              queueProgress: {
                pointIndex: msg.data?.pointIndex ?? state.queueProgress.pointIndex,
                elapsedMs: msg.data?.elapsedMs ?? state.queueProgress.elapsedMs,
                count: state.queueProgress.count
              }
            }));
            break;
          case 'TQ_DONE':
            {
              set({
                robotState: RobotState.IDLE,
                moveInProgress: false,
                moveTargetSnapshot: null,
                moveStableCount: 0,
                planningState: 'idle'
              });
              get().updateCurrentPosition();
              const settledPose = get().currentPosition;
              if (settledPose) {
                set({ targetPosition: { ...settledPose } });
              }
              const { serialManager } = get();
              if (serialManager) {
                serialManager.queryPosition().catch((error) => {
                  console.warn('Failed to query position after queued move:', error);
                });
              }
            }
            break;
          case 'MQ_STAT':
            set({
              motionKernelStatus: {
                tickJitterUs: msg.data?.tickJitterUs ?? 0,
                queueUnderrun: msg.data?.queueUnderrun ?? 0,
                stepOverrun: msg.data?.stepOverrun ?? 0,
                lastUpdated: Date.now()
              }
            });
            break;
          case 'TQ_ERR':
            set((state) => ({
              robotState: RobotState.IDLE,
              moveInProgress: false,
              moveTargetSnapshot: null,
              moveStableCount: 0,
              planningState: 'failed',
              planningNotes: [typeof msg.data === 'string' ? msg.data : JSON.stringify(msg.data)],
              ikStatus: {
                jointAngles: [
                  state.currentAngles.J1,
                  state.currentAngles.J2,
                  state.currentAngles.J3,
                  state.currentAngles.J4,
                  state.currentAngles.J5,
                  state.currentAngles.J6
                ],
                success: false,
                error: typeof msg.data === 'string' ? msg.data : `TQ ${msg.data?.code || 'error'}`,
                errorCode: typeof msg.data === 'string' ? 'QUEUE_RUNTIME_ERROR' : msg.data?.code,
                stage: 'execution',
                limitsSource: 'firmware',
                angleFrame: 'logical',
                failureCategory: 'queue_upload_failed'
              }
            }));
            console.error('Trajectory queue error:', msg.data);
            break;
          case 'ERROR':
            if (typeof msg.data === 'string' && msg.data.startsWith('TQ ')) {
              set((state) => ({
                robotState: RobotState.IDLE,
                moveInProgress: false,
                moveTargetSnapshot: null,
                moveStableCount: 0,
                planningState: 'failed',
                planningNotes: [msg.data],
                ikStatus: {
                  jointAngles: [
                    state.currentAngles.J1,
                    state.currentAngles.J2,
                    state.currentAngles.J3,
                    state.currentAngles.J4,
                    state.currentAngles.J5,
                    state.currentAngles.J6
                  ],
                  success: false,
                  error: msg.data,
                  failureCategory: 'queue_upload_failed'
                }
              }));
            } else {
              set({ robotState: RobotState.ERROR });
            }
            console.error('Robot error:', msg.data);
            break;
        }
      });

      set({
        serialManager: manager,
        connectionStatus: ConnectionStatus.CONNECTED,
        motorsEnabled: false,
        homedJoints: createFalseHomedState(),
        moveInProgress: false,
        moveTargetSnapshot: null,
        moveStableCount: 0,
        firmwareConfig: null,
        reachabilityAtlasReady: FEATURE_REACHABILITY_ATLAS_V1 && isReachabilityAtlasReady(),
        motionKernelStatus: {
          tickJitterUs: 0,
          queueUnderrun: 0,
          stepOverrun: 0,
          lastUpdated: null
        },
        ikDiagnostics: null,
        kinematicsFrameReady: false,
        currentPosition: null,
        targetPosition: null,
        queueProgress: { pointIndex: 0, elapsedMs: 0, count: 0 },
        planningState: 'idle',
        planningLatencyMs: 0,
        planningNotes: []
      });

      // Query initial position
      await manager.queryConfig();
      await manager.queryTrajectoryQueue();
      await manager.queryPosition();

    } catch (error) {
      set({ connectionStatus: ConnectionStatus.ERROR });
      throw error;
    }
  },

  // Disconnect
  disconnect: async () => {
    stopPlannerWorker();
    const { serialManager } = get();
    if (serialManager) {
      await serialManager.disconnect();
    }
    set({
      serialManager: null,
      connectionStatus: ConnectionStatus.DISCONNECTED,
      motorsEnabled: false,
      homedJoints: createFalseHomedState(),
      moveInProgress: false,
      moveTargetSnapshot: null,
      moveStableCount: 0,
      firmwareConfig: null,
      reachabilityAtlasReady: FEATURE_REACHABILITY_ATLAS_V1 && isReachabilityAtlasReady(),
      motionKernelStatus: {
        tickJitterUs: 0,
        queueUnderrun: 0,
        stepOverrun: 0,
        lastUpdated: null
      },
      ikDiagnostics: null,
      kinematicsFrameReady: false,
      currentPosition: null,
      targetPosition: null,
      queueProgress: { pointIndex: 0, elapsedMs: 0, count: 0 },
      planningState: 'idle',
      planningLatencyMs: 0,
      planningNotes: []
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
    const { serialManager, targetAngles, manualSpeed, motorsEnabled, connectionStatus } = get();
    if (!serialManager) return;
    if (!motorsEnabled || connectionStatus !== ConnectionStatus.CONNECTED) {
      console.warn('Move to target ignored: motors disabled or not connected');
      return;
    }
    stopPlannerWorker();

    set({
      robotState: RobotState.MOVING,
      moveInProgress: true,
      moveTargetSnapshot: { ...targetAngles },
      moveStableCount: 0,
      planningState: 'idle',
      planningLatencyMs: 0,
      planningNotes: []
    });
    try {
      await serialManager.stopTrajectoryQueue();
    } catch (error) {
      console.warn('Failed to stop trajectory queue before manual move:', error);
    }
    await serialManager.moveToAngles(targetAngles, manualSpeed);
  },

  // Home joints
  homeJoints: async (joints) => {
    const { serialManager } = get();
    if (!serialManager) return;
    stopPlannerWorker();
    const selectedJoints = parseHomingSelection(joints);

    set((state) => {
      if (selectedJoints.length === 0) {
        return {
          robotState: RobotState.HOMING,
          moveInProgress: false,
          moveTargetSnapshot: null,
          moveStableCount: 0,
          targetPosition: null,
          queueProgress: { pointIndex: 0, elapsedMs: 0, count: 0 },
          planningState: 'idle',
          planningLatencyMs: 0,
          planningNotes: []
        };
      }

      const nextHomed = { ...state.homedJoints };
      for (const joint of selectedJoints) {
        nextHomed[joint] = false;
      }

      return {
        robotState: RobotState.HOMING,
        moveInProgress: false,
        moveTargetSnapshot: null,
        moveStableCount: 0,
        homedJoints: nextHomed,
        targetPosition: null,
        queueProgress: { pointIndex: 0, elapsedMs: 0, count: 0 },
        planningState: 'idle',
        planningLatencyMs: 0,
        planningNotes: []
      };
    });
    await serialManager.homeJoints(joints);
  },

  // Enable/disable motors
  enableMotors: async (enable) => {
    const { serialManager } = get();
    if (!serialManager) return;

    await serialManager.enableMotors(enable);
    set((state) => ({
      motorsEnabled: enable,
      homedJoints: enable ? state.homedJoints : createFalseHomedState()
    }));
  },

  // Emergency stop
  emergencyStop: async () => {
    const { serialManager, executionState } = get();
    stopPlannerWorker();

    // Cancel any running trajectory
    if (executionState === ExecutionState.EXECUTING || executionState === ExecutionState.PAUSED) {
      get().cancelExecution();
    }

    if (!serialManager) return;

    try {
      await serialManager.stopTrajectoryQueue();
    } catch (error) {
      console.warn('Failed to stop trajectory queue before E-stop:', error);
    }

    await serialManager.emergencyStop();
    set({
      robotState: RobotState.ESTOPPED,
      moveInProgress: false,
      moveTargetSnapshot: null,
      moveStableCount: 0,
      queueProgress: { pointIndex: 0, elapsedMs: 0, count: 0 },
      planningState: 'failed',
      planningNotes: ['Planning cancelled by emergency stop']
    });
  },

  // Set manual speed
  setManualSpeed: (speed) => {
    set({ manualSpeed: speed });
  },

  setCartesianMode: (mode) => {
    set({ cartesianMode: mode });
  },

  setIKEngineMode: (mode) => {
    if (!FEATURE_IK_ENGINE_V2) {
      set({ ikEngineMode: 'legacy_dls_v1' });
      return;
    }
    set({ ikEngineMode: mode });
  },

  setIKSolveProfile: (profile) => {
    set({ ikSolveProfile: profile });
  },

  setIKPrimaryMode: (mode) => {
    if (!FEATURE_IK_ANALYTIC_PRIMARY_V1) {
      set({ ikPrimaryMode: 'numeric_fallback' });
      return;
    }
    set({ ikPrimaryMode: mode });
  },

  setBranchLockEnabled: (enabled) => {
    set({ branchLockEnabled: enabled });
  },

  setResolvedRateEnabled: (enabled) => {
    if (!FEATURE_IK_RESOLVED_RATE_V1) {
      set({ resolvedRateEnabled: false });
      return;
    }
    set({ resolvedRateEnabled: enabled });
  },

  setCollisionCheckEnabled: (enabled) => {
    if (!FEATURE_IK_COLLISION_CHECK_V1) {
      set({ collisionCheckEnabled: false });
      return;
    }
    set({ collisionCheckEnabled: enabled });
  },

  isCartesianReady: () => {
    const { connectionStatus, motorsEnabled, homedJoints, kinematicsFrameReady } = get();
    return (
      connectionStatus === ConnectionStatus.CONNECTED &&
      kinematicsFrameReady &&
      motorsEnabled &&
      homedJoints.J2 &&
      homedJoints.J3 &&
      homedJoints.J4 &&
      homedJoints.J5
    );
  },

  requestConfig: async () => {
    const { serialManager } = get();
    if (!serialManager) return;
    await serialManager.queryConfig();
  },

  requestHomePose: async () => {
    const { serialManager } = get();
    if (!serialManager) return;
    await serialManager.queryHomePose();
  },

  requestMotionKernelStatus: async () => {
    const { serialManager, firmwareConfig } = get();
    if (!serialManager) return;
    if (!FEATURE_MOTION_KERNEL_V2) return;
    if (!firmwareConfig?.capabilities?.motionKernelDiag) return;
    await serialManager.queryMotionKernel();
  },

  setCalibration: async (joint, scale, offset) => {
    const { serialManager } = get();
    if (!serialManager) return;
    await serialManager.setCalibration(joint, scale, offset);
    await serialManager.queryConfig();
  },

  zeroCalibration: async (joint, logicalDeg) => {
    const { serialManager } = get();
    if (!serialManager) return;
    await serialManager.zeroCalibration(joint, logicalDeg);
    await serialManager.queryConfig();
  },

  saveCalibration: async () => {
    const { serialManager } = get();
    if (!serialManager) return;
    await serialManager.saveCalibration();
  },

  loadCalibration: async () => {
    const { serialManager } = get();
    if (!serialManager) return;
    await serialManager.loadCalibration();
    await serialManager.queryConfig();
  },

  resetCalibration: async () => {
    const { serialManager } = get();
    if (!serialManager) return;
    await serialManager.resetCalibration();
    await serialManager.queryConfig();
  },

  jogRelative: async (joint, deltaDeg, speed) => {
    const { serialManager } = get();
    if (!serialManager) return;
    await serialManager.jogRelative(joint, deltaDeg, speed);
  },

  setHomePoseEnabled: async (enabled) => {
    const { serialManager } = get();
    if (!serialManager) return;
    await serialManager.setHomePoseEnabled(enabled);
    await serialManager.queryConfig();
  },

  setHomePoseJoint: async (joint, angleDeg) => {
    const { serialManager } = get();
    if (!serialManager) return;
    await serialManager.setHomePoseJoint(joint, angleDeg);
    await serialManager.queryConfig();
  },

  setHomePoseAll: async (j2, j3, j4, j5) => {
    const { serialManager } = get();
    if (!serialManager) return;
    await serialManager.setHomePoseAll(j2, j3, j4, j5);
    await serialManager.queryConfig();
  },

  setHomePoseSpeed: async (speedDegS) => {
    const { serialManager } = get();
    if (!serialManager) return;
    await serialManager.setHomePoseSpeed(speedDegS);
    await serialManager.queryConfig();
  },

  saveHomePose: async () => {
    const { serialManager } = get();
    if (!serialManager) return;
    await serialManager.saveHomePose();
  },

  loadHomePose: async () => {
    const { serialManager } = get();
    if (!serialManager) return;
    await serialManager.loadHomePose();
    await serialManager.queryConfig();
  },

  resetHomePose: async () => {
    const { serialManager } = get();
    if (!serialManager) return;
    await serialManager.resetHomePose();
    await serialManager.queryConfig();
  },

  // Update current Cartesian position using forward kinematics
  updateCurrentPosition: () => {
    const { currentAngles, firmwareConfig } = get();

    if (!firmwareConfig || !Array.isArray(firmwareConfig.joints) || firmwareConfig.joints.length !== 6) {
      set({
        kinematicsFrameReady: false,
        currentPosition: null
      });
      return;
    }

    const offsets = getEffectiveUrdfOffsets(firmwareConfig);

    const anglesArray = [
      currentAngles.J1,
      currentAngles.J2,
      currentAngles.J3,
      currentAngles.J4,
      currentAngles.J5,
      currentAngles.J6
    ];

    const fkResult = ForwardKinematics.solve(logicalToUrdfAngles(anglesArray, offsets));

    if (fkResult.success) {
      set({
        kinematicsFrameReady: true,
        currentPosition: fkResult.endEffectorPose.position
      });
    } else {
      set({
        kinematicsFrameReady: false
      });
    }
  },

  // Set target Cartesian position
  setTargetPosition: (position) => {
    set({ targetPosition: position });
  },

  cancelCartesianPlanning: (reason = 'Planning cancelled') => {
    const state = get();
    if (state.planningState === 'idle') {
      return;
    }
    activePlannerRequestId += 1;
    stopPlannerWorker();
    set({
      planningState: 'idle',
      planningLatencyMs: 0,
      planningNotes: [reason],
      planningDiagnostics: [
        CartesianPlanningService.diagnosticEvent('unknown', reason)
      ]
    });
  },

  // Move to Cartesian position using inverse kinematics
  moveToPosition: async (position, targetOrientation) => {
    const {
      serialManager,
      currentAngles,
      firmwareConfig,
      connectionStatus,
      motorsEnabled,
      kinematicsFrameReady,
      cartesianMode: storedCartesianMode,
      ikEngineMode,
      ikPrimaryMode,
      branchLockEnabled,
      resolvedRateEnabled,
      collisionCheckEnabled
    } = get();
    const cartesianMode: CartesianMode = targetOrientation !== undefined ? 'pose_lock' : storedCartesianMode;

    const requestId = ++activePlannerRequestId;
    stopPlannerWorker();
    // Reset branch lock so this move starts fresh, not contaminated by the
    // previous motion's tracking_local interpolation phase.
    hybridEndpointSolver.resetBranchState();
    const planningStartedAt = performance.now();
    const plannerDiagnostics: CartesianPlannerDiagnosticEvent[] = [];
    plannerDiagnostics.push(
      CartesianPlanningService.diagnosticEvent('stage1_fast', 'Cartesian planning request accepted')
    );

    const setPlanningFailure = (
      error: string,
      baseResult?: Partial<IKResult>,
      notes: string[] = [],
      failureCategory?: IKResult['failureCategory']
    ): void => {
      const finalFailureCategory = failureCategory ?? baseResult?.failureCategory ?? 'max_iterations';
      const sampleDiagnostic = (baseResult?.quality
        ? ({
            stage: (baseResult?.stage || 'unknown') as IKSolveStage,
            sampleIndex: baseResult?.sampleIndex ?? -1,
            tSec: 0,
            residualPosM: baseResult.quality.positionResidualM,
            residualOriRad: baseResult.quality.orientationResidualRad,
            weightedResidual: baseResult.quality.weightedResidual,
            branchId: baseResult?.branchId || baseResult.quality.branchId,
            flags: baseResult?.singularityFlags || []
          } satisfies IKSampleDiagnostic)
        : undefined);
      plannerDiagnostics.push(
        CartesianPlanningService.diagnosticEvent(
          (baseResult?.stage || 'unknown') as IKSolveStage,
          error,
          {
            failureCategory: finalFailureCategory,
            errorCode: baseResult?.errorCode,
            branchId: baseResult?.branchId
          },
          sampleDiagnostic
        )
      );
      set((state) => ({
        planningState: 'failed',
        moveInProgress: false,
        moveTargetSnapshot: null,
        moveStableCount: 0,
        robotState: state.moveInProgress ? RobotState.IDLE : state.robotState,
        planningLatencyMs: performance.now() - planningStartedAt,
        planningNotes: notes,
        planningDiagnostics: [...plannerDiagnostics],
        ikStatus: {
          jointAngles: baseResult?.jointAngles || [
            state.currentAngles.J1,
            state.currentAngles.J2,
            state.currentAngles.J3,
            state.currentAngles.J4,
            state.currentAngles.J5,
            state.currentAngles.J6
          ],
          success: false,
          error,
          errorCode: baseResult?.errorCode || 'CARTESIAN_PLAN_FAILED',
          stage: (baseResult?.stage || 'unknown') as IKSolveStage,
          sampleIndex: baseResult?.sampleIndex,
          limitsSource: (baseResult?.limitsSource || 'firmware') as IKLimitsSource,
          angleFrame: (baseResult?.angleFrame || 'urdf') as IKAngleFrame,
          iterations: baseResult?.iterations,
          residualError: baseResult?.residualError,
          quality: baseResult?.quality,
          failureCategory: finalFailureCategory,
          notes
        }
      }));
    };

    if (
      !Number.isFinite(position.x) ||
      !Number.isFinite(position.y) ||
      !Number.isFinite(position.z)
    ) {
      setPlanningFailure('Invalid Cartesian target', undefined, ['Target must contain numeric XYZ values.'], 'invalid_target');
      return;
    }

    if (!serialManager || connectionStatus !== ConnectionStatus.CONNECTED) {
      setPlanningFailure('Connect to robot before Cartesian move');
      return;
    }

    if (!motorsEnabled) {
      setPlanningFailure('Enable motors before Cartesian move');
      return;
    }

    if (!kinematicsFrameReady) {
      setPlanningFailure('Wait for Cartesian frame sync before Cartesian move');
      return;
    }

    if (!get().isCartesianReady()) {
      setPlanningFailure('Home J2..J5 before Cartesian move');
      return;
    }

    const queueCapable = Boolean(firmwareConfig?.capabilities?.trajectoryQueue);
    if (!queueCapable) {
      setPlanningFailure(
        'Firmware does not support trajectory queue (TQ). Flash the latest firmware.',
        undefined,
        [],
        'queue_upload_failed'
      );
      return;
    }

    if (!firmwareConfig || firmwareConfig.joints.length !== 6) {
      setPlanningFailure('Firmware config missing or incomplete');
      return;
    }
    const constraintContract = ConstraintAdapterService.fromFirmwareConfig(firmwareConfig);
    if (!constraintContract) {
      setPlanningFailure('Unable to normalize runtime joint constraints from firmware config');
      return;
    }
    applyConstraintContractToSolvers(constraintContract);

    set({
      planningState: 'stage1_fast',
      planningLatencyMs: 0,
      planningNotes: ['Running fast endpoint feasibility solve...'],
      planningDiagnostics: [
        CartesianPlanningService.diagnosticEvent('stage1_fast', 'Running fast endpoint feasibility solve...')
      ],
      ikDiagnostics: null
    });

    const offsets = constraintContract.urdfOffsetsDeg;
    const urdfJointLimitsDeg = toUrdfLimitVectors(constraintContract);
    const currentLogicalAngles = [
      currentAngles.J1,
      currentAngles.J2,
      currentAngles.J3,
      currentAngles.J4,
      currentAngles.J5,
      currentAngles.J6
    ];
    const currentUrdfAngles = logicalToUrdfAngles(currentLogicalAngles, offsets);
    const currentPoseFk = ForwardKinematics.solve(currentUrdfAngles);

    if (!currentPoseFk.success) {
      setPlanningFailure('Failed to read current pose for Cartesian planning', {
        jointAngles: currentUrdfAngles,
        failureCategory: 'fk_failure'
      });
      return;
    }

    const lockedOrientation: Rotation3 = targetOrientation ?? {
      ...currentPoseFk.endEffectorPose.rotation
    };

    const atlasEnabled = FEATURE_REACHABILITY_ATLAS_V1 && get().reachabilityAtlasReady;
    const atlas = atlasEnabled ? getDefaultReachabilityAtlas() : null;
    const reachabilityProbe = atlas
      ? classifyReachability(position, atlas)
      : {
          likelyReachable: true,
          inAtlasBounds: true,
          reason: 'atlas_disabled',
          boundaryDistanceM: Number.POSITIVE_INFINITY
        };

    if (atlasEnabled && !reachabilityProbe.likelyReachable) {
      setPlanningFailure(
        'Target outside validated reachability atlas bounds.',
        {
          jointAngles: currentUrdfAngles,
          failureCategory: 'invalid_target'
        },
        [
          `Atlas reason=${reachabilityProbe.reason}`,
          'Move target inside validated workspace envelope.'
        ],
        'invalid_target'
      );
      return;
    }

    if (FEATURE_IK_DIAGNOSTICS_LOG) {
      logJointSensitivity(
        currentUrdfAngles,
        cartesianMode === 'pose_lock' ? 'Cartesian pose-lock IK' : 'Cartesian position-only IK'
      );
    }

    const queueLimitRaw = firmwareConfig.capabilities?.trajectoryMaxPoints ?? 0;
    const queuePointBudget = Math.max(16, (queueLimitRaw > 0 ? queueLimitRaw : 256) - 4);
    const distanceToTarget = Math.sqrt(
      Math.pow(position.x - currentPoseFk.endEffectorPose.position.x, 2) +
      Math.pow(position.y - currentPoseFk.endEffectorPose.position.y, 2) +
      Math.pow(position.z - currentPoseFk.endEffectorPose.position.z, 2)
    );

    let plannedSpeedMmS = CARTESIAN_DIRECT_SPEED_MM_S;
    let estimatedDuration = estimateMinimumJerkDuration(
      distanceToTarget,
      plannedSpeedMmS,
      CARTESIAN_DIRECT_ACCEL_MM_S2
    );
    const maxDurationAtTargetFloor = (queuePointBudget - 1) / CARTESIAN_TARGET_MIN_POINTS_PER_SEC;
    if (estimatedDuration > maxDurationAtTargetFloor) {
      let speedCandidate = plannedSpeedMmS;
      while (speedCandidate < CARTESIAN_DIRECT_MAX_SPEED_MM_S) {
        speedCandidate = Math.min(CARTESIAN_DIRECT_MAX_SPEED_MM_S, speedCandidate + 2);
        const candidateDuration = estimateMinimumJerkDuration(
          distanceToTarget,
          speedCandidate,
          CARTESIAN_DIRECT_ACCEL_MM_S2
        );
        plannedSpeedMmS = speedCandidate;
        estimatedDuration = candidateDuration;
        if (candidateDuration <= maxDurationAtTargetFloor) {
          break;
        }
      }
    }

    const maxPpsByQueue = Math.max(
      1,
      Math.floor((queuePointBudget - 1) / Math.max(estimatedDuration, 0.001))
    );
    const effectivePointsPerSecond = Math.min(CARTESIAN_DIRECT_POINTS_PER_SEC, maxPpsByQueue);
    if (effectivePointsPerSecond < CARTESIAN_ABSOLUTE_MIN_POINTS_PER_SEC) {
      setPlanningFailure(
        `Cartesian move exceeds trajectory queue budget (${queuePointBudget} points) for minimum sampling floor (${CARTESIAN_ABSOLUTE_MIN_POINTS_PER_SEC} Hz).`,
        { jointAngles: currentUrdfAngles, failureCategory: 'queue_upload_failed' },
        [
          `Planned speed ${plannedSpeedMmS.toFixed(1)} mm/s (max ${CARTESIAN_DIRECT_MAX_SPEED_MM_S} mm/s)`,
          `Estimated duration ${(estimatedDuration * 1000).toFixed(0)} ms`,
          `Queue-safe sampling ${maxPpsByQueue} Hz`
        ],
        'queue_upload_failed'
      );
      return;
    }

    const statefulPreferredBranch = get().ikStatus?.branchId;
    const ikSolveContext: IKSolveContext = IKRuntimeService.createSolveContext(
      currentLogicalAngles,
      offsets,
      statefulPreferredBranch,
      'firmware'
    );

    const createIkOptions = (
      mode: CartesianMode,
      variant: 'base' | 'relaxed',
      intent: 'endpoint_global' | 'tracking_local',
      overrides: Partial<IKSolveOptions> = {}
    ): Partial<IKSolveOptions> => {
      const isPoseLock = mode === 'pose_lock';
      const useLegacy =
        ikPrimaryMode === 'numeric_fallback' ||
        !FEATURE_IK_ENGINE_V2 ||
        ikEngineMode === 'legacy_dls_v1';
      const trackingMode: IKTrackingMode = resolvedRateEnabled ? 'resolved_rate' : 'iterative_pose';
      const base: Partial<IKSolveOptions> = {
        positionWeight: 1.0,
        // For path-sample solves (tracking_local) use a lower orientation weight so the solver
        // prioritises closing the position gap rather than getting stuck in an orientation-correct /
        // position-wrong saddle point.  Endpoint solves keep the full weight.
        orientationWeight: isPoseLock
          ? (intent === 'tracking_local' ? 0.20 : (useLegacy ? 0.25 : 0.35))
          : 0.0,
        maxStepDeg: variant === 'base' ? 4.0 : 5.5,
        maxJointVelocityDegS: useLegacy ? 100 : 115,
        maxJointAccelerationDegS2: useLegacy ? 300 : 340,
        minDamping: variant === 'base' ? 0.0005 : 0.00025,
        maxDamping: useLegacy ? 0.25 : 0.3,
        dampingGrowth: 2.0,
        dampingShrink: useLegacy ? 0.75 : 0.7,
        singularityThreshold: useLegacy ? 0.000001 : 0.0000005,
        postureWeight: variant === 'base' ? 0.0000002 : 0.00000005,
        // tracking_local is a path-feasibility solve; 2.5 mm balances convergence reliability
        // (avoids saddle at ~2.3 mm with orientationWeight=0.20) and trajectory smoothness
        // (tighter than 3 mm reduces IK oscillation between consecutive path samples).
        // Stage-2 and endpoint_global apply the strict 1.5 mm precision gate after the
        // feasibility path is established.
        tolerancePositionM: intent === 'tracking_local' ? 0.0025 : 0.0015,
        toleranceOrientationRad: isPoseLock ? 0.026 : Number.POSITIVE_INFINITY,
        toleranceWeighted: 0.01,
        activeConstraints: isPoseLock ? ['position', 'orientation_hold'] : ['position'],
        intent,
        mode,
        computeDiagnostics: intent === 'endpoint_global',
        diagnosticStride: intent === 'tracking_local' ? 6 : 2,
        trackingMaxIterations: DEFAULT_IK_PLANNING_BUDGET.trackingMaxIterations,
        maxSeeds: intent === 'tracking_local' ? 1 : 4,
        maxStages: intent === 'tracking_local' ? 1 : 2,
        preferredBranch: ikSolveContext.preferredBranch,
        previousSolutionDeg: ikSolveContext.previousSolutionUrdfDeg,
        branchLockEnabled,
        // tracking_local path-interpolation samples must not use collision steering:
        // the conservative capsule model (30 mm base, 22 mm links) rejects near-target
        // steps at valid wrist poses, stalling the solver exactly at the tolerance
        // boundary.  Endpoint validation (endpoint_global) keeps the full check.
        collisionCheckEnabled: intent === 'tracking_local' ? false : collisionCheckEnabled,
        boundaryDistanceM: reachabilityProbe.boundaryDistanceM,
        trackingMode
      };
      return {
        ...base,
        ...overrides
      };
    };

    const clampSeed = (seed: number[]): number[] => (
      seed.map((value, index) => clampToRange(
        value,
        constraintContract.joints[index].logicalMinDeg,
        constraintContract.joints[index].logicalMaxDeg
      ))
    );

    const seedCandidates: number[][] = [currentLogicalAngles];
    if (Array.isArray(reachabilityProbe.suggestedSeed) && reachabilityProbe.suggestedSeed.length === 6) {
      seedCandidates.push(clampSeed(reachabilityProbe.suggestedSeed));
    }
    seedCandidates.push(clampSeed([
      currentLogicalAngles[0],
      currentLogicalAngles[1] + CARTESIAN_SEED_DELTA_DEG,
      currentLogicalAngles[2] - CARTESIAN_SEED_DELTA_DEG,
      currentLogicalAngles[3],
      currentLogicalAngles[4],
      currentLogicalAngles[5]
    ]));

    const endpointTargetPose = {
      position,
      rotation: cartesianMode === 'pose_lock' ? lockedOrientation : currentPoseFk.endEffectorPose.rotation
    };

    const endpointSolver = (
      ikPrimaryMode === 'analytic_first' &&
      FEATURE_IK_ENGINE_V2 &&
      ikEngineMode === 'hybrid_constrained_v2'
    )
      ? hybridEndpointSolver
      : legacyEndpointSolver;

    type EndpointAttempt = {
      stage: string;
      seedLogical: number[];
      mode: CartesianMode;
      result: IKResult;
      logicalSolution: number[];
    };

    const runEndpointGroup = (
      mode: CartesianMode
    ): { success: EndpointAttempt | null; bestFailure: EndpointAttempt | null; attempts: EndpointAttempt[] } => {
      const attempts: EndpointAttempt[] = [];
      let success: EndpointAttempt | null = null;
      let bestFailure: EndpointAttempt | null = null;

      const attemptSpecs: Array<{ stage: string; seedLogical: number[]; ikOptions: Partial<IKSolveOptions> }> = [
        {
          stage: 'A/base',
          seedLogical: seedCandidates[0],
          ikOptions: createIkOptions(mode, 'base', 'endpoint_global')
        },
        {
          stage: 'B/relaxed',
          seedLogical: seedCandidates[0],
          ikOptions: createIkOptions(mode, 'relaxed', 'endpoint_global')
        }
      ];

      if (seedCandidates.length > 1) {
        attemptSpecs.push({
          stage: 'C/atlas',
          seedLogical: seedCandidates[1],
          ikOptions: createIkOptions(mode, 'relaxed', 'endpoint_global')
        });
      }

      for (const spec of attemptSpecs) {
        const seedUrdf = logicalToUrdfAngles(spec.seedLogical, offsets);
        const resultUrdf = endpointSolver.solvePoseWeighted(endpointTargetPose, seedUrdf, spec.ikOptions);
        const logicalSolution = urdfToLogicalAngles(resultUrdf.jointAngles, offsets);
        const attempt: EndpointAttempt = {
          stage: spec.stage,
          seedLogical: spec.seedLogical,
          mode,
          result: resultUrdf,
          logicalSolution
        };
        attempts.push(attempt);

        if (resultUrdf.success) {
          success = attempt;
          break;
        }

        const residual = resultUrdf.quality?.positionResidualM ?? Number.POSITIVE_INFINITY;
        const bestResidual = bestFailure?.result.quality?.positionResidualM ?? Number.POSITIVE_INFINITY;
        if (residual < bestResidual) {
          bestFailure = attempt;
        }
      }

      return { success, bestFailure, attempts };
    };

    const primaryEndpoint = runEndpointGroup(cartesianMode);
    let endpointSolved = primaryEndpoint.success;

    // Endpoint solve is a global feasibility heuristic. When it fails (e.g.
    // starting from joint-limit configurations such as immediately after homing),
    // we proceed to path interpolation anyway: tracking_local uses warm
    // incremental steps and can reach targets the global endpoint solve cannot.
    // The stage1 position-only probe and stage1 failure handlers below
    // correctly classify orientation-infeasible and truly-unreachable targets.

    if (requestId !== activePlannerRequestId) {
      return;
    }

    const endpointPreferredBranch = endpointSolved?.result.branchId || statefulPreferredBranch;
    let stage1PointsPerSecond = clampToRange(
      Math.min(22, effectivePointsPerSecond),
      CARTESIAN_ABSOLUTE_MIN_POINTS_PER_SEC,
      Math.max(CARTESIAN_ABSOLUTE_MIN_POINTS_PER_SEC, effectivePointsPerSecond)
    );
    let stage1FallbackUsed = false;
    const stage1AttemptNotes: string[] = [];
    const stage1FailureReason = (interpolation: CartesianInterpolationResult): string => (
      interpolation.error || interpolation.lastIK?.error || 'unknown interpolation failure'
    );
    const hasStage1Failure = (interpolation: CartesianInterpolationResult): boolean => (
      !interpolation.success || interpolation.segment.points.length === 0
    );

    const planStage1 = (
      mode: CartesianMode,
      variant: 'base' | 'relaxed',
      pps: number,
      optionsOverride?: Partial<IKSolveOptions>
    ): CartesianInterpolationResult => trajectoryPlanner.planPoseLockedCartesianMove(
      currentLogicalAngles,
      position,
      mode === 'pose_lock' ? lockedOrientation : undefined,
      {
        speedMmS: plannedSpeedMmS,
        accelerationMmS2: CARTESIAN_DIRECT_ACCEL_MM_S2,
        pointsPerSecond: pps,
        ikOptions: createIkOptions(mode, variant, 'tracking_local', {
          preferredBranch: mode === 'pose_lock' ? endpointPreferredBranch : undefined,
          ...optionsOverride
        }),
        ikEngineMode: FEATURE_IK_ENGINE_V2 ? ikEngineMode : 'legacy_dls_v1',
        strictFinalGate: false
      }
    );

    let stage1Interpolation = planStage1(cartesianMode, 'base', stage1PointsPerSecond);
    if (stage1Interpolation.lastIK) {
      stage1Interpolation.lastIK = IKRuntimeService.withContext(
        stage1Interpolation.lastIK,
        ikSolveContext,
        'stage1_fast',
        undefined,
        stage1Interpolation.lastIK.errorCode || 'IK_STAGE1_INTERPOLATION'
      );
    }
    if (hasStage1Failure(stage1Interpolation)) {
      stage1AttemptNotes.push(`Stage1 ${stage1PointsPerSecond}Hz failed: ${stage1FailureReason(stage1Interpolation)}`);
      const retryPointsPerSecond = clampToRange(
        Math.floor(stage1PointsPerSecond * 0.75),
        CARTESIAN_ABSOLUTE_MIN_POINTS_PER_SEC,
        stage1PointsPerSecond
      );
      const stage1Retry = planStage1(cartesianMode, 'relaxed', retryPointsPerSecond, {
        trackingMaxIterations: DEFAULT_IK_PLANNING_BUDGET.trackingMaxIterations + 8,
        maxSeeds: 2,
        maxStages: 2,
        // Looser positional and orientation gates for the fallback pass.
        // orientationWeight stays at 0.30 (original value): for near-limit poses the
        // orientation gradient actively guides the solver into the correct basin of
        // attraction.  Lowering it to 0.20 (same as base) was tested and made both
        // position AND orientation residuals worse in practice.
        // 3.5 mm (up from 3.0 mm): paths that converge to 3.0–3.5 mm are still useful
        // as Stage2 seeds; tightening this gate caused false joint_limit rejections on
        // otherwise reachable targets.
        tolerancePositionM: 0.0035,
        toleranceOrientationRad: cartesianMode === 'pose_lock' ? 0.04 : Number.POSITIVE_INFINITY,
        toleranceWeighted: 0.014,
        orientationWeight: cartesianMode === 'pose_lock' ? 0.30 : 0.0
      });
      if (stage1Retry.lastIK) {
        stage1Retry.lastIK = IKRuntimeService.withContext(
          stage1Retry.lastIK,
          ikSolveContext,
          'stage1_fast',
          undefined,
          stage1Retry.lastIK.errorCode || 'IK_STAGE1_INTERPOLATION_RETRY'
        );
      }
      if (hasStage1Failure(stage1Retry)) {
        stage1AttemptNotes.push(
          `Stage1 fallback ${retryPointsPerSecond}Hz failed: ${stage1FailureReason(stage1Retry)}`
        );
        stage1Interpolation = stage1Retry;
      } else {
        stage1FallbackUsed = true;
        stage1PointsPerSecond = retryPointsPerSecond;
        stage1Interpolation = stage1Retry;
        stage1AttemptNotes.push(
          `Stage1 fallback succeeded at ${retryPointsPerSecond}Hz with relaxed tracking options`
        );
      }
    }

    if (hasStage1Failure(stage1Interpolation) && cartesianMode === 'pose_lock') {
      const positionOnlyProbe = planStage1('position_only', 'relaxed', stage1PointsPerSecond, {
        // Use the same branch as the pose-lock solve so the probe tests reachability
        // on the same arm configuration.  preferredBranch: undefined would cause the
        // solver to explore a different branch, yielding worse position residuals than
        // the pose-lock solve itself and producing false "joint_limit" classifications.
        preferredBranch: endpointPreferredBranch,
        trackingMaxIterations: DEFAULT_IK_PLANNING_BUDGET.trackingMaxIterations + 8,
        maxSeeds: 2,
        maxStages: 2,
        // Loose 6 mm gate: the probe is a reachability indicator, not a precision solve.
        // The target position is typically reachable if the position-only solve gets
        // within 6 mm; precision is enforced by stage-2.
        tolerancePositionM: 0.006,
        toleranceWeighted: 0.020,
        orientationWeight: 0.0,
        toleranceOrientationRad: Number.POSITIVE_INFINITY
      });
      if (positionOnlyProbe.lastIK) {
        positionOnlyProbe.lastIK = IKRuntimeService.withContext(
          positionOnlyProbe.lastIK,
          ikSolveContext,
          'stage1_fast',
          undefined,
          positionOnlyProbe.lastIK.errorCode || 'IK_STAGE1_POSITION_ONLY_PROBE'
        );
      }
      if (!hasStage1Failure(positionOnlyProbe)) {
        // The position is geometrically reachable (probe passed) but the pose-locked solve failed.
        // Only classify as orientation_infeasible when orientation residual actually exceeds tolerance.
        // If orientation residual is well within tolerance, the solver just failed to converge on
        // position — labelling that as orientation_infeasible gives the user wrong advice.
        const POSE_LOCK_ORIENTATION_TOL_RAD = 0.026;
        const lastOriResidual = stage1Interpolation.lastIK?.quality?.orientationResidualRad;
        const orientationIsActuallyInfeasible =
          lastOriResidual === undefined || lastOriResidual >= POSE_LOCK_ORIENTATION_TOL_RAD;
        if (orientationIsActuallyInfeasible) {
          setPlanningFailure(
            'Pose lock infeasible along Cartesian path at current limits. Switch to Position Only and retry.',
            stage1Interpolation.lastIK || endpointSolved?.result || primaryEndpoint.bestFailure?.result,
            [
              `Mode=${cartesianMode}`,
              ...stage1AttemptNotes,
              `Stage1 position-only probe succeeded at ${stage1PointsPerSecond}Hz`
            ],
            'orientation_infeasible'
          );
          return;
        }

        // Orientation is within tolerance but position is stuck (orientationWeight=0.20 creates a
        // saddle at ~9mm for some path regions).  Try a soft pose-lock with near-zero orientation
        // weight so the position gradient dominates.  If it produces a feasible path, Stage2 will
        // attempt to tighten orientation back to the strict 0.026 rad limit.
        const softLockAttempt = planStage1('pose_lock', 'relaxed', stage1PointsPerSecond, {
          orientationWeight: 0.05,
          tolerancePositionM: 0.003,
          toleranceOrientationRad: 0.10,
          toleranceWeighted: 0.025,
          trackingMaxIterations: DEFAULT_IK_PLANNING_BUDGET.trackingMaxIterations + 12,
          maxSeeds: 3,
          maxStages: 3
        });
        if (!hasStage1Failure(softLockAttempt)) {
          // Soft lock found a feasible path — promote it and fall through to Stage2.
          stage1Interpolation = softLockAttempt;
          stage1FallbackUsed = true;
          stage1AttemptNotes.push(
            `Stage1 soft-lock (weight=0.05) succeeded; Stage2 will enforce full pose-lock`
          );
          // No return — fall through to Stage2.
        } else {
          stage1AttemptNotes.push(`Stage1 soft-lock failed: ${stage1FailureReason(softLockAttempt)}`);
          setPlanningFailure(
            'Failed to converge along Cartesian path. Try a shorter move, lower speed, or switch to Position Only.',
            stage1Interpolation.lastIK || endpointSolved?.result || primaryEndpoint.bestFailure?.result,
            [
              `Mode=${cartesianMode}`,
              ...stage1AttemptNotes,
              `Stage1 position-only probe succeeded at ${stage1PointsPerSecond}Hz`,
              `ori residual ${lastOriResidual?.toFixed(4)} rad (within tolerance)`
            ],
            'max_iterations'
          );
          return;
        }
      } else {
        stage1AttemptNotes.push(
          `Stage1 position-only probe failed: ${stage1FailureReason(positionOnlyProbe)}`
        );
      }
    }

    if (hasStage1Failure(stage1Interpolation)) {
      const stage1FailureCategory = stage1Interpolation.lastIK?.failureCategory || 'max_iterations';
      // If the endpoint IK succeeded the target position IS reachable.  Path-tracking can still
      // fail with failureCategory='joint_limit' because an intermediate waypoint near a joint
      // limit resists convergence — that is a path-quality issue, not true unreachability.
      // Override to 'max_iterations' so the user gets actionable advice ("try a different
      // starting pose") rather than the misleading "outside reachable workspace" message.
      const effectiveCategory: IKResult['failureCategory'] = (endpointSolved && stage1FailureCategory === 'joint_limit')
        ? 'max_iterations'
        : (stage1FailureCategory || endpointSolved?.result.failureCategory || 'max_iterations');
      const stage1Message = effectiveCategory === 'joint_limit'
        ? 'Target is outside the robot\'s reachable workspace (joint limit). Try a closer position or move the arm to a different starting pose.'
        : endpointSolved
          ? 'Path planning failed: the trajectory passes near a joint limit. Try a different starting pose or switch to Position Only mode.'
          : stage1Interpolation.error || 'Stage1 fast feasibility path failed';
      setPlanningFailure(
        stage1Message,
        stage1Interpolation.lastIK || {
          jointAngles: endpointSolved?.result.jointAngles ?? currentLogicalAngles,
          failureCategory: endpointSolved?.result.failureCategory || 'max_iterations'
        },
        [
          `Mode=${cartesianMode}`,
          ...stage1AttemptNotes,
          `Stage1 ${stage1PointsPerSecond}Hz feasibility path failed`
        ],
        effectiveCategory
      );
      return;
    }

    if (stage1Interpolation.segment.points.length > queuePointBudget) {
      setPlanningFailure(
        `Stage1 feasibility path exceeded queue budget (${stage1Interpolation.segment.points.length} > ${queuePointBudget}).`,
        stage1Interpolation.lastIK || {
          jointAngles: endpointSolved?.result.jointAngles ?? currentLogicalAngles,
          failureCategory: 'queue_upload_failed'
        },
        [
          `Mode=${cartesianMode}`,
          `Stage1 ${stage1PointsPerSecond}Hz produced ${stage1Interpolation.segment.points.length} points`
        ],
        'queue_upload_failed'
      );
      return;
    }

    const stage1LatencyMs = performance.now() - planningStartedAt;
    const stage1FeasibleAngles =
      stage1Interpolation.segment.points[stage1Interpolation.segment.points.length - 1]?.jointAngles ||
      endpointSolved?.logicalSolution ||
      currentLogicalAngles;
    const stage2MinPointsPerSecond = Math.min(CARTESIAN_STAGE2_MIN_POINTS_PER_SEC, effectivePointsPerSecond);
    const stage2StrictIkOptions = createIkOptions(cartesianMode, 'base', 'tracking_local', {
      preferredBranch: endpointPreferredBranch,
      trackingMaxIterations: Math.max(DEFAULT_IK_PLANNING_BUDGET.trackingMaxIterations + 10, 30),
      maxSeeds: 2,
      maxStages: 2,
      maxStepDeg: 4.5,
      postureWeight: 0.0000001
    });
    const stage2TimeoutMs = computeAdaptiveStage2TimeoutMs({
      stage1PointCount: stage1Interpolation.segment.points.length,
      stage1LatencyMs,
      stage1DurationSec: stage1Interpolation.segment.duration,
      stage2MinPointsPerSecond,
      minTimeoutMs: STAGE2_TIMEOUT_MIN_MS,
      maxTimeoutMs: STAGE2_TIMEOUT_MAX_MS
    });
    const stage2DeadlineMs = Date.now() + stage2TimeoutMs;

    set({
      planningState: 'stage2_refine',
      planningLatencyMs: stage1LatencyMs,
      planningNotes: [
        `Stage1 endpoint + coarse path done in ${stage1LatencyMs.toFixed(0)}ms`,
        stage1LatencyMs > DEFAULT_IK_PLANNING_BUDGET.stage1BudgetMs
          ? `Stage1 exceeded ${DEFAULT_IK_PLANNING_BUDGET.stage1BudgetMs}ms budget`
          : `Stage1 met ${DEFAULT_IK_PLANNING_BUDGET.stage1BudgetMs}ms budget`,
        `Stage1 coarse path: ${stage1Interpolation.segment.points.length} points @ ${stage1PointsPerSecond}Hz`,
        `Stage2 budget ${stage2TimeoutMs}ms (${STAGE2_SELECTION_POLICY})`,
        ...(stage1FallbackUsed ? ['Stage1 used relaxed fallback settings.'] : []),
        'Fast feasibility found, refining strict path...'
      ],
      planningDiagnostics: [
        ...plannerDiagnostics,
        CartesianPlanningService.diagnosticEvent('stage1_fast', 'Stage1 feasibility accepted', {
          failureCategory: undefined,
          errorCode: stage1Interpolation.lastIK?.errorCode,
          branchId: stage1Interpolation.lastIK?.branchId
        })
      ],
      ikDiagnostics: {
        attempts: primaryEndpoint.attempts.length,
        bestStage: endpointSolved?.stage || 'path_interpolation',
        bestResidualMm: (endpointSolved?.result.quality?.positionResidualM ?? stage1Interpolation.lastIK?.quality?.positionResidualM ?? 0) * 1000,
        branchId: endpointSolved?.result.quality?.branchId || stage1Interpolation.lastIK?.branchId || (reachabilityProbe.branchId || 'unknown'),
        seedIndex: endpointSolved?.result.quality?.seedIndex ?? 0
      }
    });

    try {
      const stage2 = await runStage2PlannerWorker({
        requestId,
        startAngles: currentLogicalAngles,
        targetPosition: position,
        targetOrientation: cartesianMode === 'pose_lock' ? lockedOrientation : undefined,
        urdfOffsetsDeg: offsets,
        jointLimitsUrdfDeg: urdfJointLimitsDeg,
        cartesianMode,
        ikEngineMode: FEATURE_IK_ENGINE_V2 ? ikEngineMode : 'legacy_dls_v1',
        speedMmS: plannedSpeedMmS,
        accelerationMmS2: CARTESIAN_DIRECT_ACCEL_MM_S2,
        minPointsPerSecond: stage2MinPointsPerSecond,
        maxPointsPerSecond: effectivePointsPerSecond,
        queuePointBudget,
        strictIkOptions: stage2StrictIkOptions,
        timeoutMs: stage2TimeoutMs,
        deadlineMs: stage2DeadlineMs,
        selectionPolicy: STAGE2_SELECTION_POLICY,
        maxCandidateAttempts: STAGE2_MAX_CANDIDATE_ATTEMPTS,
        maxAdditionalProbesAfterSuccess: STAGE2_MAX_ADDITIONAL_PROBES_AFTER_SUCCESS,
        onProgress: (msg, elapsedMs) => {
          if (requestId !== activePlannerRequestId) return;
          set((state) => ({
            planningState: 'stage2_refine',
            planningLatencyMs: stage1LatencyMs + elapsedMs,
            planningNotes: [...state.planningNotes.slice(0, 2), msg],
            planningDiagnostics: [
              ...state.planningDiagnostics,
              CartesianPlanningService.diagnosticEvent('stage2_refine', msg)
            ]
          }));
        }
      });

      if (requestId !== activePlannerRequestId) {
        return;
      }

      const interpolation = stage2.interpolation;
      const points = interpolation.segment.points;
      for (let i = 0; i < stage2.diagnostics.length; i++) {
        const diagnostic = stage2.diagnostics[i];
        const message = diagnostic.selected
          ? `Stage2 selected candidate #${diagnostic.candidateIndex} @ ${diagnostic.pointsPerSecond}Hz`
          : `Stage2 candidate #${diagnostic.candidateIndex} @ ${diagnostic.pointsPerSecond}Hz (${diagnostic.strictConverged ? 'strict-pass' : 'strict-fail'})`;
        plannerDiagnostics.push(
          CartesianPlanningService.diagnosticEvent('stage2_refine', message, {
            errorCode: diagnostic.selectionReason || (diagnostic.strictConverged ? 'STAGE2_STRICT_PASS' : 'STAGE2_STRICT_FAIL'),
            failureCategory: undefined
          })
        );
      }
      if (interpolation.lastIK) {
        interpolation.lastIK = IKRuntimeService.withContext(
          interpolation.lastIK,
          ikSolveContext,
          'stage2_refine',
          undefined,
          interpolation.lastIK.errorCode || 'IK_STAGE2_INTERPOLATION'
        );
      }
      if (!interpolation.success || points.length === 0) {
        setPlanningFailure(
          interpolation.error || 'Stage2 strict refinement failed',
          interpolation.lastIK,
          stage2.notes,
          interpolation.lastIK?.failureCategory || 'max_iterations'
        );
        return;
      }

      if (points.length > queuePointBudget) {
        setPlanningFailure(
          `Generated trajectory (${points.length} points) exceeds queue budget (${queuePointBudget}).`,
          interpolation.lastIK,
          stage2.notes,
          'queue_upload_failed'
        );
        return;
      }

      const violation = TrajectoryExecutionService.validateTrajectoryPointsAgainstConstraints(
        points,
        constraintContract
      );
      if (violation) {
        const detail = Number.isFinite(violation.value)
          ? `point=${violation.pointIndex} joint=J${violation.jointIndex + 1} value=${violation.value.toFixed(3)} range=[${violation.min.toFixed(3)}, ${violation.max.toFixed(3)}]`
          : 'firmware configuration unavailable for strict validation';
        setPlanningFailure(
          `Planned trajectory violates firmware joint limits (${detail}).`,
          interpolation.lastIK
            ? IKRuntimeService.withContext(
                {
                  ...interpolation.lastIK,
                  errorCode: 'TRAJECTORY_POINT_OUT_OF_LIMITS'
                },
                ikSolveContext,
                'stage2_refine'
              )
            : {
                jointAngles: stage1FeasibleAngles,
                failureCategory: 'joint_limit',
                errorCode: 'TRAJECTORY_POINT_OUT_OF_LIMITS',
                stage: 'stage2_refine',
                limitsSource: 'firmware',
                angleFrame: 'logical'
              },
          [...stage2.notes, detail],
          'joint_limit'
        );
        return;
      }

      const finalTargetAngles = TrajectoryExecutionService.finalTargetAnglesFromTrajectory(points);
      const finalUrdf = logicalToUrdfAngles([
        finalTargetAngles.J1,
        finalTargetAngles.J2,
        finalTargetAngles.J3,
        finalTargetAngles.J4,
        finalTargetAngles.J5,
        finalTargetAngles.J6
      ], offsets);
      const finalPose = ForwardKinematics.solve(finalUrdf);
      const commandedTargetPosition = finalPose.success
        ? finalPose.endEffectorPose.position
        : position;

      const successfulIk = interpolation.lastIK;
      const planningLatencyMs = performance.now() - planningStartedAt;
      const successNotes = [
        `Mode=${cartesianMode}`,
        `Path=${plannedSpeedMmS.toFixed(1)}mm/s @ ${stage2.chosenPps}Hz`,
        `Stage1 endpoint seed from ${endpointSolved?.stage || 'path_interpolation'}`,
        ...stage2.notes,
        'Low XYZ sensitivity joints can remain near-static by geometry (J6 is orientation-only).'
      ];
      plannerDiagnostics.push(
        CartesianPlanningService.diagnosticEvent('stage2_refine', 'Stage2 strict refinement accepted', {
          failureCategory: undefined,
          errorCode: successfulIk?.errorCode,
          branchId: successfulIk?.branchId
        })
      );

      set({
        planningState: 'ready',
        planningLatencyMs,
        planningNotes: successNotes,
        planningDiagnostics: [...plannerDiagnostics],
        ikStatus: {
          jointAngles: successfulIk?.jointAngles || finalUrdf,
          success: true,
          errorCode: successfulIk?.errorCode || 'IK_PLAN_READY',
          stage: 'stage2_refine',
          limitsSource: 'firmware',
          angleFrame: 'urdf',
          iterations: successfulIk?.iterations,
          residualError: successfulIk?.residualError,
          quality: successfulIk?.quality,
          notes: successNotes,
          error: cartesianMode === 'pose_lock' ? 'Pose lock active' : 'Position-only mode active'
        },
        ikDiagnostics: {
          attempts: primaryEndpoint.attempts.length,
          bestStage: endpointSolved?.stage || 'path_interpolation',
          bestResidualMm: (successfulIk?.quality?.positionResidualM ?? 0) * 1000,
          branchId: successfulIk?.quality?.branchId || (reachabilityProbe.branchId || 'unknown'),
          seedIndex: successfulIk?.quality?.seedIndex ?? -1
        }
      });

      const summary = summarizeIKResult(get().ikStatus);
      if (summary && FEATURE_IK_DIAGNOSTICS_LOG) {
        console.info(
          `IK quality | pos=${summary.positionResidualMm.toFixed(2)}mm ` +
          `ori=${summary.orientationResidualDeg.toFixed(2)}deg ` +
          `cond=${summary.conditionNumber.toFixed(1)} ` +
          `sigmaMin=${summary.minSingularValue.toExponential(2)} ` +
          `weak=${summary.weakJoints.join(',') || 'none'}`
        );
      }

      set({
        robotState: RobotState.MOVING,
        targetAngles: finalTargetAngles,
        targetPosition: commandedTargetPosition,
        moveInProgress: true,
        moveTargetSnapshot: { ...finalTargetAngles },
        moveStableCount: 0,
        queueProgress: { pointIndex: 0, elapsedMs: 0, count: points.length }
      });

      try {
        await TrajectoryExecutionService.uploadAndRunTrajectoryQueue(serialManager, points);
      } catch (uploadError) {
        const uploadMessage = uploadError instanceof Error ? uploadError.message : String(uploadError);
        throw new Stage2PlannerFailure(`Queue upload/run failed: ${uploadMessage}`, {
          failureCategory: 'queue_upload_failed',
          notes: [
            ...stage2.notes,
            `Queue upload/run failed for ${points.length} points`
          ],
          diagnostics: stage2.diagnostics
        });
      }
    } catch (error) {
      if (requestId !== activePlannerRequestId) {
        return;
      }
      const stage2Error = error instanceof Stage2PlannerFailure
        ? error
        : new Stage2PlannerFailure(error instanceof Error ? error.message : String(error));
      const message = stage2Error.message;
      const timeoutFailure = stage2Error.timedOut || stage2Error.failureCategory === 'stage2_timeout' || isStage2TimeoutMessage(message);
      const strictGateFailure = /final cartesian sample did not meet strict residual gate/i.test(message);
      const fallbackEligibleFailure = timeoutFailure || strictGateFailure;

      for (let i = 0; i < stage2Error.diagnostics.length; i++) {
        const diagnostic = stage2Error.diagnostics[i];
        plannerDiagnostics.push(
          CartesianPlanningService.diagnosticEvent(
            'stage2_refine',
            `Stage2 candidate #${diagnostic.candidateIndex} @ ${diagnostic.pointsPerSecond}Hz failed`,
            {
              failureCategory: stage2Error.failureCategory,
              errorCode: timeoutFailure
                ? 'STAGE2_TIMEOUT'
                : (strictGateFailure ? 'STAGE2_STRICT_GATE_FAILED' : 'STAGE2_EXECUTION_FAILED')
            }
          )
        );
      }

      if (fallbackEligibleFailure) {
        const stage1StrictConverged = isStrictInterpolationConverged(
          stage1Interpolation,
          stage2StrictIkOptions.tolerancePositionM ?? 0.0015,
          stage2StrictIkOptions.toleranceOrientationRad ?? Number.POSITIVE_INFINITY,
          stage2StrictIkOptions.toleranceWeighted ?? 0.01
        );
        const stage1Points = stage1Interpolation.segment.points;
        const stage1WithinBudget = stage1Points.length > 0 && stage1Points.length <= queuePointBudget;
        const stage1Violation = stage1WithinBudget
          ? TrajectoryExecutionService.validateTrajectoryPointsAgainstConstraints(stage1Points, constraintContract)
          : {
              pointIndex: 0,
              jointIndex: 0,
              value: Number.NaN,
              min: Number.NaN,
              max: Number.NaN
            };
        const canFallbackToStage1 =
          stage1Interpolation.success &&
          stage1StrictConverged &&
          stage1WithinBudget &&
          !stage1Violation;
        const fallbackErrorCode = timeoutFailure
          ? 'STAGE2_TIMEOUT_STAGE1_FALLBACK'
          : 'STAGE2_STRICT_GATE_STAGE1_FALLBACK';
        const fallbackReason = timeoutFailure
          ? `Stage2 timeout after ${stage2TimeoutMs}ms`
          : 'Stage2 strict residual gate failed';
        const fallbackSummary = timeoutFailure
          ? 'Stage2 timeout fallback: strict-valid Stage1 path'
          : 'Stage2 strict-gate fallback: strict-valid Stage1 path';

        if (canFallbackToStage1) {
          const fallbackFinalTargetAngles = TrajectoryExecutionService.finalTargetAnglesFromTrajectory(stage1Points);
          const fallbackFinalUrdf = logicalToUrdfAngles([
            fallbackFinalTargetAngles.J1,
            fallbackFinalTargetAngles.J2,
            fallbackFinalTargetAngles.J3,
            fallbackFinalTargetAngles.J4,
            fallbackFinalTargetAngles.J5,
            fallbackFinalTargetAngles.J6
          ], offsets);
          const fallbackFinalPose = ForwardKinematics.solve(fallbackFinalUrdf);
          const fallbackTargetPosition = fallbackFinalPose.success
            ? fallbackFinalPose.endEffectorPose.position
            : position;
          const fallbackLastIk = stage1Interpolation.lastIK
            ? IKRuntimeService.withContext(
                {
                  ...stage1Interpolation.lastIK,
                  errorCode: fallbackErrorCode
                },
                ikSolveContext,
                'stage2_refine'
              )
            : {
                jointAngles: fallbackFinalUrdf,
                success: true,
                errorCode: fallbackErrorCode,
                stage: 'stage2_refine' as IKSolveStage,
                limitsSource: 'firmware' as IKLimitsSource,
                angleFrame: 'urdf' as IKAngleFrame
              };
          const fallbackNotes = [
            `Mode=${cartesianMode}`,
            fallbackReason,
            `Fallback to strict-valid Stage1 path @ ${stage1PointsPerSecond}Hz`,
            ...stage2Error.notes
          ];
          plannerDiagnostics.push(
            CartesianPlanningService.diagnosticEvent('stage2_refine', 'Stage2 fallback activated', {
              failureCategory: undefined,
              errorCode: fallbackErrorCode,
              branchId: fallbackLastIk.branchId
            })
          );

          set({
            planningState: 'ready',
            planningLatencyMs: performance.now() - planningStartedAt,
            planningNotes: fallbackNotes,
            planningDiagnostics: [...plannerDiagnostics],
            ikStatus: {
              jointAngles: fallbackLastIk.jointAngles || fallbackFinalUrdf,
              success: true,
              errorCode: fallbackErrorCode,
              stage: 'stage2_refine',
              limitsSource: 'firmware',
              angleFrame: 'urdf',
              iterations: fallbackLastIk.iterations,
              residualError: fallbackLastIk.residualError,
              quality: fallbackLastIk.quality,
              notes: fallbackNotes,
              error: fallbackSummary
            }
          });

          set({
            robotState: RobotState.MOVING,
            targetAngles: fallbackFinalTargetAngles,
            targetPosition: fallbackTargetPosition,
            moveInProgress: true,
            moveTargetSnapshot: { ...fallbackFinalTargetAngles },
            moveStableCount: 0,
            queueProgress: { pointIndex: 0, elapsedMs: 0, count: stage1Points.length }
          });

          try {
            await TrajectoryExecutionService.uploadAndRunTrajectoryQueue(serialManager, stage1Points);
            return;
          } catch (uploadError) {
            const uploadMessage = uploadError instanceof Error ? uploadError.message : String(uploadError);
            setPlanningFailure(
              `Stage1 fallback execution failed: ${uploadMessage}`,
              {
                jointAngles: stage1FeasibleAngles,
                failureCategory: 'queue_upload_failed',
                errorCode: 'QUEUE_TRANSACTION_FAILED',
                stage: 'stage2_refine',
                limitsSource: 'firmware',
                angleFrame: 'logical'
              },
              [...fallbackNotes, 'Fallback upload/run failed'],
              'queue_upload_failed'
            );
            return;
          }
        }

        if (timeoutFailure) {
          const timeoutGateNotes = [
            'Fast feasibility found but strict refinement timed out.',
            `Stage2 timeout budget ${stage2TimeoutMs}ms`,
            `Stage1 strict gate: ${stage1StrictConverged ? 'pass' : 'fail'}`,
            `Stage1 queue budget gate: ${stage1WithinBudget ? 'pass' : 'fail'}`,
            ...(stage1Violation ? ['Stage1 joint-limit gate: fail'] : ['Stage1 joint-limit gate: pass']),
            ...stage2Error.notes
          ];
          setPlanningFailure(
            `Stage2 strict refinement failed: ${message}`,
            {
              jointAngles: stage1FeasibleAngles,
              failureCategory: 'stage2_timeout',
              errorCode: 'STAGE2_TIMEOUT',
              stage: 'stage2_refine',
              limitsSource: 'firmware',
              angleFrame: 'logical'
            },
            timeoutGateNotes,
            'stage2_timeout'
          );
          return;
        }

        const strictGateNotes = [
          'Fast feasibility found but strict refinement missed final residual gate.',
          `Stage1 strict gate: ${stage1StrictConverged ? 'pass' : 'fail'}`,
          `Stage1 queue budget gate: ${stage1WithinBudget ? 'pass' : 'fail'}`,
          ...(stage1Violation ? ['Stage1 joint-limit gate: fail'] : ['Stage1 joint-limit gate: pass']),
          ...stage2Error.notes
        ];
        const strictGateCategory: IKResult['failureCategory'] = stage2Error.failureCategory || 'max_iterations';
        setPlanningFailure(
          `Stage2 strict refinement failed: ${message}`,
          {
            jointAngles: stage1FeasibleAngles,
            failureCategory: strictGateCategory,
            errorCode: 'STAGE2_STRICT_GATE_FAILED',
            stage: 'stage2_refine',
            limitsSource: 'firmware',
            angleFrame: 'logical'
          },
          strictGateNotes,
          strictGateCategory
        );
        return;
      }

      const queueLayerError = /queue|tq|ack|transaction/i.test(message);
      const failureCategory: IKResult['failureCategory'] = stage2Error.failureCategory
        || (queueLayerError ? 'queue_upload_failed' : 'max_iterations');
      const queueFailure = failureCategory === 'queue_upload_failed';
      setPlanningFailure(
        `Stage2 strict refinement failed: ${message}`,
        {
          jointAngles: stage1FeasibleAngles,
          failureCategory,
          errorCode: queueFailure ? 'QUEUE_TRANSACTION_FAILED' : 'STAGE2_EXECUTION_FAILED',
          stage: 'stage2_refine',
          limitsSource: 'firmware',
          angleFrame: 'logical'
        },
        stage2Error.notes.length > 0
          ? stage2Error.notes
          : ['Fast feasibility found but strict refinement did not complete.'],
        failureCategory
      );
    }
  },

  jogCartesian: async (delta) => {
    const {
      currentAngles,
      firmwareConfig,
      kinematicsFrameReady,
      connectionStatus,
      motorsEnabled,
      homedJoints
    } = get();

    if (
      !kinematicsFrameReady ||
      connectionStatus !== ConnectionStatus.CONNECTED ||
      !motorsEnabled ||
      !firmwareConfig
    ) return;
    if (!homedJoints.J2 || !homedJoints.J3 || !homedJoints.J4 || !homedJoints.J5) return;

    const constraintContract = ConstraintAdapterService.fromFirmwareConfig(firmwareConfig);
    if (!constraintContract) return;

    const offsets = constraintContract.urdfOffsetsDeg;
    const currentLogicalAngles = [
      currentAngles.J1,
      currentAngles.J2,
      currentAngles.J3,
      currentAngles.J4,
      currentAngles.J5,
      currentAngles.J6
    ];
    const currentUrdfAngles = logicalToUrdfAngles(currentLogicalAngles, offsets);
    const currentPoseFk = ForwardKinematics.solve(currentUrdfAngles);
    if (!currentPoseFk.success) return;

    const pos = currentPoseFk.endEffectorPose.position;
    const rot = currentPoseFk.endEffectorPose.rotation;
    const dx = delta.dx ?? 0;
    const dy = delta.dy ?? 0;
    const dz = delta.dz ?? 0;
    const rx = delta.rx ?? 0;
    const ry = delta.ry ?? 0;
    const rz = delta.rz ?? 0;
    const frame = delta.frame ?? 'world';

    // Position delta: transform to world frame when jogging in tool frame
    let newPos: Vector3;
    if (frame === 'tool' && (dx !== 0 || dy !== 0 || dz !== 0)) {
      const R = QuaternionMath.toRotationMatrix(QuaternionMath.fromEuler(rot));
      newPos = {
        x: pos.x + R[0][0] * dx + R[0][1] * dy + R[0][2] * dz,
        y: pos.y + R[1][0] * dx + R[1][1] * dy + R[1][2] * dz,
        z: pos.z + R[2][0] * dx + R[2][1] * dy + R[2][2] * dz
      };
    } else {
      newPos = { x: pos.x + dx, y: pos.y + dy, z: pos.z + dz };
    }

    // Orientation delta: right-multiply for tool frame, left-multiply for world frame
    let newRot: Rotation3;
    if (rx === 0 && ry === 0 && rz === 0) {
      newRot = { ...rot };
    } else {
      const qCurrent = QuaternionMath.fromEuler(rot);
      const qDelta = QuaternionMath.fromEuler({ roll: rx, pitch: ry, yaw: rz });
      const qNew = frame === 'tool'
        ? QuaternionMath.multiply(qCurrent, qDelta)
        : QuaternionMath.multiply(qDelta, qCurrent);
      newRot = QuaternionMath.toEuler(qNew);
    }

    // Lightweight endpoint IK — no trajectory planning, no TQ upload.
    // The firmware J command uses its own trapezoidal profile for smooth motion.
    const { ikEngineMode, branchLockEnabled, serialManager } = get();
    if (!serialManager) return;

    const solver = ikEngineMode === 'hybrid_constrained_v2' ? hybridEndpointSolver : legacyEndpointSolver;
    const ikResult = solver.solvePoseWeighted(
      { position: newPos, rotation: newRot },
      currentUrdfAngles,
      {
        intent: 'endpoint_global',
        mode: 'pose_lock',
        branchLockEnabled,
        previousSolutionDeg: currentUrdfAngles,
      }
    );

    if (!ikResult.success || !ikResult.jointAngles) return;

    const logicalAngles = urdfToLogicalAngles(ikResult.jointAngles, offsets);
    await serialManager.moveToAngles({
      J1: logicalAngles[0],
      J2: logicalAngles[1],
      J3: logicalAngles[2],
      J4: logicalAngles[3],
      J5: logicalAngles[4],
      J6: logicalAngles[5],
    }, 30);
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
    const { waypoints, currentAngles, plannerConfig, firmwareConfig } = get();
    const constraintContract = ConstraintAdapterService.fromFirmwareConfig(firmwareConfig);
    const offsets = constraintContract?.urdfOffsetsDeg ?? getEffectiveUrdfOffsets(firmwareConfig);
    applyConstraintContractToSolvers(constraintContract);

    if (waypoints.length === 0) {
      set({ trajectory: null, trajectoryPositions: [] });
      return;
    }

    set({
      executionState: ExecutionState.PLANNING,
      executionProgress: { ...initialProgress, state: ExecutionState.PLANNING }
    });

    trajectoryPlanner.updateConfig(plannerConfig);
    trajectoryPlanner.setUrdfOffsets(offsets);

    const startAngles = [
      currentAngles.J1,
      currentAngles.J2,
      currentAngles.J3,
      currentAngles.J4,
      currentAngles.J5,
      currentAngles.J6
    ];

    const trajectory = trajectoryPlanner.planTrajectory(waypoints, startAngles);
    const positions = TrajectoryPlanner.getTrajectoryPositions(trajectory, offsets);

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

    if (!savedPath.kinematicsFrame) {
      console.warn(
        'Imported path has no kinematicsFrame tag (legacy file). Cartesian waypoints may use an old frame; re-teach for best accuracy.'
      );
    } else if (savedPath.kinematicsFrame === 'legacy_dh_v1') {
      console.warn(
        'Imported path uses legacy_dh_v1 frame. Cartesian waypoints may be offset; re-teach waypoints in urdf_chain_v1.'
      );
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
