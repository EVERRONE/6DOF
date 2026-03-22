import {
  CartesianMode,
  IKResult,
  IKSolveContext,
  IKSampleDiagnostic,
  IKSolveStage,
  Pose,
  Vector3
} from '../../kinematics/types';
import { CartesianInterpolationResult } from '../../motion/types';

export interface NormalizedJointConstraint {
  jointIndex: number;
  logicalMinDeg: number;
  logicalMaxDeg: number;
  urdfMinDeg: number;
  urdfMaxDeg: number;
}

export interface NormalizedConstraintContract {
  limitsSource: 'firmware' | 'urdf';
  angleFrame: 'urdf';
  urdfOffsetsDeg: number[];
  urdfDirections: number[];
  joints: NormalizedJointConstraint[];
  jointLimitsRad: {
    min: number[];
    max: number[];
  };
}

export interface CartesianPlannerConstraints {
  queuePointBudget: number;
  minPointsPerSecond: number;
  maxPointsPerSecond: number;
  targetPositionToleranceM: number;
  targetOrientationToleranceRad: number;
  weightedTolerance: number;
}

export interface CartesianRuntimePolicy {
  failClosedPoseLock: boolean;
  reliabilityFirst: boolean;
  allowResolvedRate: boolean;
}

export interface CartesianPlanRequest {
  requestId: number;
  mode: CartesianMode;
  startLogicalDeg: number[];
  startUrdfDeg: number[];
  targetPose: Pose;
  solveContext: IKSolveContext;
  constraints: CartesianPlannerConstraints;
  runtimePolicy: CartesianRuntimePolicy;
}

export interface CartesianPlannerDiagnosticEvent {
  stage: IKSolveStage;
  timestampMs: number;
  message: string;
  sample?: IKSampleDiagnostic;
  result?: Pick<IKResult, 'failureCategory' | 'errorCode' | 'branchId'>;
}

export type Stage2SelectionPolicy = 'first_success_plus_one_probe';

export interface Stage2CandidateDiagnostic {
  candidateIndex: number;
  pointsPerSecond: number;
  sampleCount: number;
  solveDurationMs: number;
  strictConverged: boolean;
  remainingBudgetMs: number;
  failureReason?: string;
  selected?: boolean;
  selectionReason?: string;
}

export interface CartesianPlanResult {
  status: 'ready' | 'failed';
  failureCategory?: IKResult['failureCategory'];
  interpolation?: CartesianInterpolationResult;
  trajectory?: {
    points: number;
    durationSec: number;
    distanceM: number;
    targetPosition: Vector3;
  };
  diagnostics: CartesianPlannerDiagnosticEvent[];
  stageTimingsMs: {
    endpoint: number;
    stage1: number;
    stage2: number;
    total: number;
  };
}
