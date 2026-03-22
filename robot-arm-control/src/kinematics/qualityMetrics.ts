import { IKResult, Pose, Vector3 } from './types';
import { QuaternionMath } from './QuaternionMath';

const sub = (a: Vector3, b: Vector3): Vector3 => ({
  x: a.x - b.x,
  y: a.y - b.y,
  z: a.z - b.z
});

const dot = (a: Vector3, b: Vector3): number => a.x * b.x + a.y * b.y + a.z * b.z;

const norm = (v: Vector3): number => Math.sqrt(dot(v, v));

export const distance = (a: Vector3, b: Vector3): number => norm(sub(a, b));

export const maxLineDeviation = (samples: Vector3[], start: Vector3, end: Vector3): number => {
  const line = sub(end, start);
  const lineLen = norm(line);
  if (lineLen < 1e-9) return 0;

  let maxDeviation = 0;
  for (const p of samples) {
    const rel = sub(p, start);
    const t = dot(rel, line) / (lineLen * lineLen);
    const tClamped = Math.max(0, Math.min(1, t));
    const projection: Vector3 = {
      x: start.x + line.x * tClamped,
      y: start.y + line.y * tClamped,
      z: start.z + line.z * tClamped
    };
    maxDeviation = Math.max(maxDeviation, distance(p, projection));
  }
  return maxDeviation;
};

export const orientationDriftRad = (start: Pose, end: Pose): number => {
  const qStart = QuaternionMath.fromEuler(start.rotation);
  const qEnd = QuaternionMath.fromEuler(end.rotation);
  return QuaternionMath.angularDistance(qStart, qEnd);
};

export interface IKDiagnosticSummary {
  positionResidualMm: number;
  orientationResidualDeg: number;
  conditionNumber: number;
  minSingularValue: number;
  weakJoints: number[];
}

export const summarizeIKResult = (result: IKResult | null): IKDiagnosticSummary | null => {
  if (!result || !result.quality) return null;
  const quality = result.quality;
  const participation = quality.jointParticipation || [];
  const maxP = participation.reduce((acc, value) => Math.max(acc, value), 0);
  const weakThreshold = maxP * 0.1;
  const weakJoints = participation
    .map((value, index) => ({ index, value }))
    .filter((entry) => entry.value <= weakThreshold)
    .map((entry) => entry.index + 1);

  return {
    positionResidualMm: quality.positionResidualM * 1000,
    orientationResidualDeg: (quality.orientationResidualRad * 180) / Math.PI,
    conditionNumber: quality.conditionNumber,
    minSingularValue: quality.minSingularValue,
    weakJoints
  };
};
