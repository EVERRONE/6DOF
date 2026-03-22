import {
  IKResult,
  IKSolveContext,
  IKSampleDiagnostic,
  IKLimitsSource,
  IKSolveStage,
  IKBranchId
} from '../../kinematics/types';
import { logicalToUrdfAngles } from '../../kinematics/angleMapping';

export class IKRuntimeService {
  static createSolveContext(
    previousLogicalDeg: number[],
    urdfOffsetsDeg: number[],
    preferredBranch?: IKBranchId,
    limitsSource: IKLimitsSource = 'firmware'
  ): IKSolveContext {
    const previousSolutionUrdfDeg = logicalToUrdfAngles(previousLogicalDeg, urdfOffsetsDeg);
    return {
      angleFrame: 'urdf',
      limitsSource,
      previousSolutionUrdfDeg,
      preferredBranch
    };
  }

  static withContext(
    result: IKResult,
    context: IKSolveContext,
    stage: IKSolveStage,
    sampleIndex?: number,
    errorCode?: string
  ): IKResult {
    return {
      ...result,
      stage,
      sampleIndex,
      errorCode: errorCode || result.errorCode,
      limitsSource: context.limitsSource,
      angleFrame: context.angleFrame
    };
  }

  static toSampleDiagnostic(
    result: IKResult,
    stage: IKSolveStage,
    sampleIndex: number,
    tSec: number,
    note?: string
  ): IKSampleDiagnostic | null {
    if (!result.quality) return null;
    return {
      stage,
      sampleIndex,
      tSec,
      residualPosM: result.quality.positionResidualM,
      residualOriRad: result.quality.orientationResidualRad,
      weightedResidual: result.quality.weightedResidual,
      branchId: result.branchId || result.quality.branchId,
      flags: result.singularityFlags || [],
      note
    };
  }
}
