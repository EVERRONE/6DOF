import { CartesianMode, IKEngineMode, IKSolveOptions, Rotation3, Vector3 } from '../kinematics/types';
import { CartesianInterpolationResult } from '../motion/types';
import { Stage2CandidateDiagnostic, Stage2SelectionPolicy } from '../services/cartesian/types';

export interface SerialMessage {
  type:
    | 'POS'
    | 'ENDSTOP'
    | 'OK'
    | 'ERROR'
    | 'HOMED'
    | 'CFG'
    | 'HP'
    | 'HOMEPOSE_REACHED'
    | 'TQ_READY'
    | 'TQ_PROG'
    | 'TQ_DONE'
    | 'TQ_ERR'
    | 'TQ_STAT'
    | 'MQ_STAT'
    | 'ACK';
  data: any;
  timestamp: number;
}

export interface Command {
  type: 'MOVE' | 'HOME' | 'QUERY' | 'ENABLE' | 'STOP';
  payload: any;
  id: string;
  timestamp: number;
}

export interface PlannerWorkerPlanPayload {
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
  deadlineMs?: number;
  selectionPolicy?: Stage2SelectionPolicy;
  maxCandidateAttempts?: number;
  maxAdditionalProbesAfterSuccess?: number;
}

export type PlannerWorkerRequest =
  | {
      type: 'plan_stage2';
      requestId: number;
      payload: PlannerWorkerPlanPayload;
    }
  | {
      type: 'cancel';
      requestId: number;
    };

export type PlannerWorkerResponse =
  | {
      type: 'progress';
      requestId: number;
      elapsedMs: number;
      candidatePps: number;
      message: string;
    }
  | {
      type: 'result';
      requestId: number;
      chosenPps: number;
      interpolation: CartesianInterpolationResult;
      notes: string[];
      diagnostics?: Stage2CandidateDiagnostic[];
    }
  | {
      type: 'error';
      requestId: number;
      error: string;
      notes: string[];
      failureCategory?: string;
      diagnostics?: Stage2CandidateDiagnostic[];
    };
