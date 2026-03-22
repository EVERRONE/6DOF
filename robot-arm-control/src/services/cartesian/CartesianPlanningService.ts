import { IKResult, IKSolveStage, IKSampleDiagnostic } from '../../kinematics/types';
import {
  CartesianPlanResult,
  CartesianPlannerDiagnosticEvent
} from './types';

const nowMs = (): number => Date.now();

export class CartesianPlanningService {
  static diagnosticEvent(
    stage: IKSolveStage,
    message: string,
    result?: Pick<IKResult, 'failureCategory' | 'errorCode' | 'branchId'>,
    sample?: IKSampleDiagnostic
  ): CartesianPlannerDiagnosticEvent {
    return {
      stage,
      timestampMs: nowMs(),
      message,
      result,
      sample
    };
  }

  static failedResult(
    failureCategory: IKResult['failureCategory'],
    diagnostics: CartesianPlannerDiagnosticEvent[],
    stageTimingsMs: CartesianPlanResult['stageTimingsMs']
  ): CartesianPlanResult {
    return {
      status: 'failed',
      failureCategory,
      diagnostics,
      stageTimingsMs
    };
  }
}
