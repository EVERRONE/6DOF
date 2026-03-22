import { IKBranchId } from './types';
import { AnalyticalIKCandidate } from './AnalyticalPieperIK';

const jointDistance = (a: number[], b: number[]): number => {
  let sum = 0;
  for (let i = 0; i < Math.min(a.length, b.length); i++) {
    const delta = (a[i] || 0) - (b[i] || 0);
    sum += delta * delta;
  }
  return Math.sqrt(sum);
};

export interface ContinuitySelectionOptions {
  preferredBranch?: IKBranchId;
  previousSolutionDeg?: number[];
  branchLockEnabled?: boolean;
}

export class ContinuityPolicy {
  static rankCandidates(
    candidates: AnalyticalIKCandidate[],
    options?: ContinuitySelectionOptions
  ): AnalyticalIKCandidate[] {
    const preferred = options?.preferredBranch;
    const previous = options?.previousSolutionDeg;
    const lock = options?.branchLockEnabled ?? true;

    return [...candidates].sort((a, b) => {
      if (preferred) {
        const aPreferred = a.branchId === preferred ? 0 : 1;
        const bPreferred = b.branchId === preferred ? 0 : 1;
        if (lock && aPreferred !== bPreferred) return aPreferred - bPreferred;
      }

      const continuityA = previous ? jointDistance(a.jointAngles, previous) : 0;
      const continuityB = previous ? jointDistance(b.jointAngles, previous) : 0;
      if (Math.abs(continuityA - continuityB) > 1e-6) return continuityA - continuityB;

      const qualityA = a.positionResidualM * 1000 + a.orientationResidualRad * 50;
      const qualityB = b.positionResidualM * 1000 + b.orientationResidualRad * 50;
      if (Math.abs(qualityA - qualityB) > 1e-9) return qualityA - qualityB;

      return a.branchId.localeCompare(b.branchId);
    });
  }

  static detectBranchDiscontinuity(
    previousBranch: IKBranchId | undefined,
    selectedBranch: IKBranchId | undefined,
    lockEnabled: boolean
  ): boolean {
    if (!lockEnabled) return false;
    if (!previousBranch || !selectedBranch) return false;
    return previousBranch !== selectedBranch;
  }
}
