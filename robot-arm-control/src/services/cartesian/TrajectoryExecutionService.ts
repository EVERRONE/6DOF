import { TrajectoryPoint } from '../../motion/types';
import { JointAngles } from '../../types/robot';
import { SerialManager } from '../../communication/SerialManager';
import { NormalizedConstraintContract } from './types';

export interface TrajectoryLimitViolation {
  pointIndex: number;
  jointIndex: number;
  value: number;
  min: number;
  max: number;
}

const MAX_QUEUE_UPLOAD_VELOCITY_DEG_S = 85;
const RETRY_QUEUE_UPLOAD_VELOCITY_DEG_S = 60;

const toJointAngles = (angles: number[]): JointAngles => ({
  J1: angles[0] ?? 0,
  J2: angles[1] ?? 0,
  J3: angles[2] ?? 0,
  J4: angles[3] ?? 0,
  J5: angles[4] ?? 0,
  J6: angles[5] ?? 0
});

export class TrajectoryExecutionService {
  static validateTrajectoryPointsAgainstConstraints(
    points: TrajectoryPoint[],
    constraints: NormalizedConstraintContract | null
  ): TrajectoryLimitViolation | null {
    if (!constraints || !Array.isArray(constraints.joints) || constraints.joints.length !== 6) {
      return {
        pointIndex: 0,
        jointIndex: 0,
        value: Number.NaN,
        min: Number.NaN,
        max: Number.NaN
      };
    }

    for (let pointIndex = 0; pointIndex < points.length; pointIndex++) {
      const point = points[pointIndex];
      for (let jointIndex = 0; jointIndex < 6; jointIndex++) {
        const value = point.jointAngles[jointIndex];
        const min = constraints.joints[jointIndex].logicalMinDeg;
        const max = constraints.joints[jointIndex].logicalMaxDeg;
        if (!Number.isFinite(value) || value < min || value > max) {
          return {
            pointIndex,
            jointIndex,
            value,
            min,
            max
          };
        }
      }
    }

    return null;
  }

  static toJointAnglesArray(jointAngles: JointAngles): number[] {
    return [jointAngles.J1, jointAngles.J2, jointAngles.J3, jointAngles.J4, jointAngles.J5, jointAngles.J6];
  }

  static finalTargetAnglesFromTrajectory(points: TrajectoryPoint[]): JointAngles {
    return toJointAngles(points[points.length - 1].jointAngles);
  }

  static normalizeQueueTimestampsMs(points: TrajectoryPoint[]): number[] {
    const normalized: number[] = [];
    let previousMs = -1;
    for (let i = 0; i < points.length; i++) {
      const rawMs = Math.round(Math.max(0, points[i].time * 1000));
      const nextMs = i === 0
        ? Math.max(0, rawMs)
        : Math.max(rawMs, previousMs + 1);
      normalized.push(nextMs);
      previousMs = nextMs;
    }
    return normalized;
  }

  static sanitizeQueueVelocityDegS(
    velocity: number[] | undefined,
    maxAbsDegS: number = MAX_QUEUE_UPLOAD_VELOCITY_DEG_S
  ): number[] {
    const clampAbs = Math.max(1, Math.abs(maxAbsDegS));
    const source = velocity && velocity.length === 6 ? velocity : [0, 0, 0, 0, 0, 0];
    return source.map((value) => {
      if (!Number.isFinite(value)) return 0;
      if (value > clampAbs) return clampAbs;
      if (value < -clampAbs) return -clampAbs;
      return value;
    });
  }

  static async uploadAndRunTrajectoryQueue(
    serialManager: SerialManager,
    points: TrajectoryPoint[]
  ): Promise<void> {
    const queueTimesMs = this.normalizeQueueTimestampsMs(points);
    const uploadWithVelocityLimit = async (maxAbsVelocityDegS: number): Promise<void> => {
      try {
        await serialManager.stopTrajectoryQueue();
      } catch {
        // Queue may already be stopped — safe to ignore.
      }
      await serialManager.clearTrajectoryQueue();
      for (let i = 0; i < points.length; i++) {
        const point = points[i];
        const joint = toJointAngles(point.jointAngles);
        const q = this.toJointAnglesArray(joint);
        const qd = this.sanitizeQueueVelocityDegS(point.velocity, maxAbsVelocityDegS);
        await serialManager.enqueueTrajectoryPoint(queueTimesMs[i], q, qd);
      }

      const preRunStatus = await serialManager.queryTrajectoryQueue();
      const queuedCount = Number(
        preRunStatus?.data?.count ??
        preRunStatus?.data?.queueCount ??
        0
      );
      if (queuedCount !== points.length) {
        throw new Error(
          `Trajectory queue verification failed: uploaded ${points.length}, device reports ${queuedCount}`
        );
      }

      await serialManager.runTrajectoryQueue();
      await serialManager.queryTrajectoryQueue();
    };

    try {
      await uploadWithVelocityLimit(MAX_QUEUE_UPLOAD_VELOCITY_DEG_S);
    } catch (error) {
      const message = error instanceof Error ? error.message : String(error);
      if (!/VELOCITY_RANGE|velocity range/i.test(message)) {
        throw error;
      }
      await uploadWithVelocityLimit(RETRY_QUEUE_UPLOAD_VELOCITY_DEG_S);
    }
  }
}
