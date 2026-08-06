import { TrajectoryExecutionService } from '../TrajectoryExecutionService';
import { NormalizedConstraintContract } from '../types';
import { TrajectoryPoint } from '../../../motion/types';

const constraints: NormalizedConstraintContract = {
  limitsSource: 'firmware',
  angleFrame: 'urdf',
  urdfOffsetsDeg: [0, 0, 0, 0, 0, 0],
  urdfDirections: [1, 1, 1, 1, -1, -1],
  joints: [
    { jointIndex: 0, logicalMinDeg: -180, logicalMaxDeg: 180, urdfMinDeg: -180, urdfMaxDeg: 180 },
    { jointIndex: 1, logicalMinDeg: -90, logicalMaxDeg: 90, urdfMinDeg: -90, urdfMaxDeg: 90 },
    { jointIndex: 2, logicalMinDeg: -90, logicalMaxDeg: 90, urdfMinDeg: -90, urdfMaxDeg: 90 },
    { jointIndex: 3, logicalMinDeg: -180, logicalMaxDeg: 180, urdfMinDeg: -180, urdfMaxDeg: 180 },
    { jointIndex: 4, logicalMinDeg: -120, logicalMaxDeg: 120, urdfMinDeg: -120, urdfMaxDeg: 120 },
    { jointIndex: 5, logicalMinDeg: -180, logicalMaxDeg: 180, urdfMinDeg: -180, urdfMaxDeg: 180 }
  ],
  jointLimitsRad: {
    min: [-Math.PI, -Math.PI / 2, -Math.PI / 2, -Math.PI, -2.094, -Math.PI],
    max: [Math.PI, Math.PI / 2, Math.PI / 2, Math.PI, 2.094, Math.PI]
  }
};

const points: TrajectoryPoint[] = [
  {
    time: 0,
    jointAngles: [0, 0, 0, 0, 0, 0],
    velocity: [0, 0, 0, 0, 0, 0]
  },
  {
    time: 0.2,
    jointAngles: [5, 5, -5, 10, 0, -10],
    velocity: [0, 0, 0, 0, 0, 0]
  }
];

const pointsWithNonMonotonicMs: TrajectoryPoint[] = [
  {
    time: 0,
    jointAngles: [0, 0, 0, 0, 0, 0],
    velocity: [0, 0, 0, 0, 0, 0]
  },
  {
    time: 0.0004,
    jointAngles: [1, 0, 0, 0, 0, 0],
    velocity: [0, 0, 0, 0, 0, 0]
  },
  {
    time: 0.0008,
    jointAngles: [2, 0, 0, 0, 0, 0],
    velocity: [0, 0, 0, 0, 0, 0]
  }
];

const pointsWithOutOfRangeVelocity: TrajectoryPoint[] = [
  {
    time: 0,
    jointAngles: [0, 0, 0, 0, 0, 0],
    velocity: [0, 0, 0, 0, 0, 0]
  },
  {
    time: 0.1,
    jointAngles: [5, 0, 0, 0, 0, 0],
    velocity: [500, -999, 10, 0, 240.5, -240.5]
  }
];

describe('TrajectoryExecutionService', () => {
  test('flags out-of-limit trajectory points against normalized constraints', () => {
    const violation = TrajectoryExecutionService.validateTrajectoryPointsAgainstConstraints(
      [
        points[0],
        { ...points[1], jointAngles: [500, 5, -5, 10, 0, -10] }
      ],
      constraints
    );
    expect(violation).not.toBeNull();
    expect(violation?.jointIndex).toBe(0);
    expect(violation?.pointIndex).toBe(1);
  });

  test('verifies queue point count before run', async () => {
    const serialManager = {
      clearTrajectoryQueue: jest.fn(async () => {}),
      enqueueTrajectoryPoint: jest.fn(async () => {}),
      queryTrajectoryQueue: jest.fn(async () => ({ type: 'TQ_READY', data: { count: 1 } })),
      runTrajectoryQueue: jest.fn(async () => {})
    };

    await expect(
      TrajectoryExecutionService.uploadAndRunTrajectoryQueue(serialManager as any, points)
    ).rejects.toThrow('Trajectory queue verification failed');
    expect(serialManager.runTrajectoryQueue).not.toHaveBeenCalled();
  });

  test('uploads and runs queue when count verification passes', async () => {
    const serialManager = {
      clearTrajectoryQueue: jest.fn(async () => {}),
      enqueueTrajectoryPoint: jest.fn(async () => {}),
      queryTrajectoryQueue: jest
        .fn()
        .mockResolvedValueOnce({ type: 'TQ_READY', data: { count: points.length } })
        .mockResolvedValueOnce({ type: 'TQ_STAT', data: { count: points.length, running: true } }),
      runTrajectoryQueue: jest.fn(async () => {})
    };

    await TrajectoryExecutionService.uploadAndRunTrajectoryQueue(serialManager as any, points);
    expect(serialManager.clearTrajectoryQueue).toHaveBeenCalledTimes(1);
    expect(serialManager.enqueueTrajectoryPoint).toHaveBeenCalledTimes(points.length);
    expect(serialManager.runTrajectoryQueue).toHaveBeenCalledTimes(1);
    expect(serialManager.queryTrajectoryQueue).toHaveBeenCalledTimes(2);
  });

  test('normalizes queue timestamps to strictly monotonic milliseconds', async () => {
    const serialManager = {
      clearTrajectoryQueue: jest.fn(async () => {}),
      enqueueTrajectoryPoint: jest.fn(async () => {}),
      queryTrajectoryQueue: jest
        .fn()
        .mockResolvedValueOnce({ type: 'TQ_READY', data: { count: pointsWithNonMonotonicMs.length } })
        .mockResolvedValueOnce({ type: 'TQ_STAT', data: { count: pointsWithNonMonotonicMs.length, running: true } }),
      runTrajectoryQueue: jest.fn(async () => {})
    };

    await TrajectoryExecutionService.uploadAndRunTrajectoryQueue(
      serialManager as any,
      pointsWithNonMonotonicMs
    );

    const sentTimes = (serialManager.enqueueTrajectoryPoint as jest.Mock).mock.calls.map((call) => call[0]);
    expect(sentTimes).toEqual([0, 1, 2]);
  });

  test('clamps queue upload velocities into firmware-safe range', async () => {
    const serialManager = {
      clearTrajectoryQueue: jest.fn(async () => {}),
      enqueueTrajectoryPoint: jest.fn(async () => {}),
      queryTrajectoryQueue: jest
        .fn()
        .mockResolvedValueOnce({ type: 'TQ_READY', data: { count: pointsWithOutOfRangeVelocity.length } })
        .mockResolvedValueOnce({ type: 'TQ_STAT', data: { count: pointsWithOutOfRangeVelocity.length, running: true } }),
      runTrajectoryQueue: jest.fn(async () => {})
    };

    await TrajectoryExecutionService.uploadAndRunTrajectoryQueue(
      serialManager as any,
      pointsWithOutOfRangeVelocity
    );

    const uploadedVelocities = (serialManager.enqueueTrajectoryPoint as jest.Mock).mock.calls.map((call) => call[2]);
    expect(uploadedVelocities[1]).toEqual([85, -85, 10, 0, 85, -85]);
  });

  test('throws when device reports zero queued points after upload', async () => {
    const serialManager = {
      clearTrajectoryQueue: jest.fn(async () => {}),
      enqueueTrajectoryPoint: jest.fn(async () => {}),
      queryTrajectoryQueue: jest.fn(async () => ({
        type: 'TQ_READY',
        data: { count: 0 } // device reports 0 — all points dropped
      })),
      runTrajectoryQueue: jest.fn(async () => {})
    };

    await expect(
      TrajectoryExecutionService.uploadAndRunTrajectoryQueue(serialManager as any, points)
    ).rejects.toThrow('Trajectory queue verification failed');
    expect(serialManager.runTrajectoryQueue).not.toHaveBeenCalled();
  });

  test('retries queue upload with stricter velocity clamp after VELOCITY_RANGE rejection', async () => {
    let enqueueCall = 0;
    const serialManager = {
      clearTrajectoryQueue: jest.fn(async () => {}),
      enqueueTrajectoryPoint: jest.fn(async () => {
        enqueueCall += 1;
        if (enqueueCall === 2) {
          throw new Error('TQ VELOCITY_RANGE: velocity range J1');
        }
      }),
      queryTrajectoryQueue: jest
        .fn()
        .mockResolvedValueOnce({ type: 'TQ_READY', data: { count: pointsWithOutOfRangeVelocity.length } })
        .mockResolvedValueOnce({ type: 'TQ_STAT', data: { count: pointsWithOutOfRangeVelocity.length, running: true } }),
      runTrajectoryQueue: jest.fn(async () => {})
    };

    await TrajectoryExecutionService.uploadAndRunTrajectoryQueue(
      serialManager as any,
      pointsWithOutOfRangeVelocity
    );

    expect(serialManager.clearTrajectoryQueue).toHaveBeenCalledTimes(2);
    const velocityCalls = (serialManager.enqueueTrajectoryPoint as jest.Mock).mock.calls.map((call) => call[2]);
    expect(velocityCalls[1]).toEqual([85, -85, 10, 0, 85, -85]);
    expect(velocityCalls[3]).toEqual([60, -60, 10, 0, 60, -60]);
  });
});
