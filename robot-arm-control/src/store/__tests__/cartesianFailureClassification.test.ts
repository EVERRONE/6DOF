import { useRobotStore } from '../robotStore';
import { ConnectionStatus, FirmwareConfig, RobotState } from '../../types/robot';
import { TrajectoryPlanner } from '../../motion/TrajectoryPlanner';
import { HybridIKSolver } from '../../kinematics/HybridIKSolver';
import { InverseKinematics } from '../../kinematics/InverseKinematics';

const createFirmwareConfig = (trajectoryMaxPoints = 256): FirmwareConfig => ({
  version: 1,
  capabilities: {
    trajectoryQueue: true,
    trajectoryMaxPoints,
    trajectoryPointFormat: 'hermite_v1'
  },
  joints: Array.from({ length: 6 }, () => ({
    min: -180,
    max: 180,
    stepsPerDeg: 1,
    invertDir: false,
    hasEndstop: true,
    homeTowardMin: true,
    homeLogicalDeg: 0,
    postHomeOffsetDeg: 0,
    urdfOffsetDeg: 0,
    cal: { scale: 1, offset: 0 }
  })),
  pulseWidthUs: 2,
  dirSetupUs: 2,
  endstopDebounceMs: 3
});

const createFailedInterpolation = (positionResidualM: number, orientationResidualRad = 0.0002) => ({
  success: false as const,
  segment: {
    startWaypoint: 0,
    endWaypoint: 0,
    points: [{ time: 0, jointAngles: [0, 0, 0, 0, 0, 0] }],
    duration: 0.72,
    distance: 0.01
  },
  error: `Failed to converge (pos ${positionResidualM.toFixed(4)}m, ori ${orientationResidualRad.toFixed(4)}rad)`,
  failedAtTime: 0.72,
  lastIK: {
    jointAngles: [0, 0, 0, 0, 0, 0],
    success: false,
    error: `Failed to converge (pos ${positionResidualM.toFixed(4)}m, ori ${orientationResidualRad.toFixed(4)}rad)`,
    failureCategory: 'max_iterations' as const,
    quality: {
      positionResidualM,
      orientationResidualRad,
      weightedResidual: positionResidualM,
      minSingularValue: 1e-4,
      conditionNumber: 1200,
      activeConstraints: ['position'],
      jointParticipation: [1, 1, 1, 1, 1, 1],
      iterations: 120,
      damping: 0.01
    }
  }
});

const createSuccessfulInterpolation = () => ({
  success: true as const,
  segment: {
    startWaypoint: 0,
    endWaypoint: 0,
    points: [
      { time: 0, jointAngles: [0, 0, 0, 0, 0, 0], velocity: [0, 0, 0, 0, 0, 0] },
      { time: 0.2, jointAngles: [0, -2, -2, 0, 0, 0], velocity: [0, 0, 0, 0, 0, 0] }
    ],
    duration: 0.2,
    distance: 0.02
  },
  lastIK: {
    jointAngles: [0, -2, -2, 0, 0, 0],
    success: true,
    quality: {
      positionResidualM: 0.0009,
      orientationResidualRad: 0.0002,
      weightedResidual: 0.0009,
      minSingularValue: 5e-4,
      conditionNumber: 600,
      activeConstraints: ['position'],
      jointParticipation: [1, 1, 1, 1, 1, 1],
      iterations: 60,
      damping: 0.01
    }
  }
});

const createOutOfLimitInterpolation = () => ({
  success: true as const,
  segment: {
    startWaypoint: 0,
    endWaypoint: 0,
    points: [
      { time: 0, jointAngles: [0, 0, 0, 0, 0, 0], velocity: [0, 0, 0, 0, 0, 0] },
      { time: 0.2, jointAngles: [250, -2, -2, 0, 0, 0], velocity: [0, 0, 0, 0, 0, 0] }
    ],
    duration: 0.2,
    distance: 0.02
  },
  lastIK: {
    jointAngles: [250, -2, -2, 0, 0, 0],
    success: true,
    quality: {
      positionResidualM: 0.0009,
      orientationResidualRad: 0.0002,
      weightedResidual: 0.0009,
      minSingularValue: 5e-4,
      conditionNumber: 600,
      activeConstraints: ['position'],
      jointParticipation: [1, 1, 1, 1, 1, 1],
      iterations: 60,
      damping: 0.01
    }
  }
});

const createTimeoutInterpolation = () => ({
  success: false as const,
  timedOut: true as const,
  timeoutAtSampleIndex: 3,
  timeoutAtTimeSec: 0.12,
  segment: {
    startWaypoint: 0,
    endWaypoint: 0,
    points: [{ time: 0, jointAngles: [0, 0, 0, 0, 0, 0] }],
    duration: 0.2,
    distance: 0.02
  },
  error: 'Stage2 interpolation deadline exceeded at sample 3',
  failedAtTime: 0.12,
  lastIK: {
    jointAngles: [0, 0, 0, 0, 0, 0],
    success: false,
    error: 'Stage2 interpolation deadline exceeded at sample 3',
    errorCode: 'STAGE2_TIMEOUT',
    failureCategory: 'stage2_timeout' as const
  }
});

const createStrictGateFailedInterpolation = () => ({
  success: false as const,
  segment: {
    startWaypoint: 0,
    endWaypoint: 0,
    points: [
      { time: 0, jointAngles: [0, 0, 0, 0, 0, 0], velocity: [0, 0, 0, 0, 0, 0] },
      { time: 0.2, jointAngles: [0, -2, -2, 0, 0, 0], velocity: [0, 0, 0, 0, 0, 0] }
    ],
    duration: 0.2,
    distance: 0.02
  },
  error: 'Final Cartesian sample did not meet strict residual gate',
  failedAtTime: 0.2,
  lastIK: {
    jointAngles: [0, -2, -2, 0, 0, 0],
    success: true,
    quality: {
      positionResidualM: 0.0017,
      orientationResidualRad: 0.0031,
      weightedResidual: 0.0105,
      minSingularValue: 2e-4,
      conditionNumber: 900,
      activeConstraints: ['position', 'orientation_hold'],
      jointParticipation: [1, 1, 1, 1, 1, 1],
      iterations: 24,
      damping: 0.01
    }
  }
});

const createLooseSuccessfulInterpolation = () => ({
  success: true as const,
  segment: {
    startWaypoint: 0,
    endWaypoint: 0,
    points: [
      { time: 0, jointAngles: [0, 0, 0, 0, 0, 0], velocity: [0, 0, 0, 0, 0, 0] },
      { time: 0.2, jointAngles: [0, -2, -2, 0, 0, 0], velocity: [0, 0, 0, 0, 0, 0] }
    ],
    duration: 0.2,
    distance: 0.02
  },
  lastIK: {
    jointAngles: [0, -2, -2, 0, 0, 0],
    success: true,
    quality: {
      positionResidualM: 0.0045,
      orientationResidualRad: 0.04,
      weightedResidual: 0.016,
      minSingularValue: 5e-4,
      conditionNumber: 600,
      activeConstraints: ['position', 'orientation_hold'],
      jointParticipation: [1, 1, 1, 1, 1, 1],
      iterations: 60,
      damping: 0.01
    }
  }
});

describe('robotStore Cartesian failure classification', () => {
  beforeEach(() => {
    const serialManager = {
      clearTrajectoryQueue: jest.fn(async () => {}),
      enqueueTrajectoryPoint: jest.fn(async () => {}),
      runTrajectoryQueue: jest.fn(async () => {}),
      queryTrajectoryQueue: jest.fn(async () => ({ type: 'TQ_READY', data: { count: 2 } }))
    };

    useRobotStore.setState({
      connectionStatus: ConnectionStatus.CONNECTED,
      serialManager: serialManager as any,
      robotState: RobotState.IDLE,
      currentAngles: { J1: 0, J2: 0, J3: 0, J4: 0, J5: 0, J6: 0 },
      targetAngles: { J1: 0, J2: 0, J3: 0, J4: 0, J5: 0, J6: 0 },
      endstopState: { J1: false, J2: false, J3: false, J4: false, J5: false, J6: false },
      homedJoints: { J1: false, J2: true, J3: true, J4: true, J5: true, J6: false },
      motorsEnabled: true,
      moveInProgress: false,
      moveTargetSnapshot: null,
      moveStableCount: 0,
      firmwareConfig: createFirmwareConfig(),
      kinematicsFrameReady: true,
      currentPosition: null,
      targetPosition: null,
      ikStatus: null,
      cartesianMode: 'pose_lock',
      ikEngineMode: 'hybrid_constrained_v2',
      ikPrimaryMode: 'analytic_first'
    });
  });

  afterEach(() => {
    jest.restoreAllMocks();
  });

  test('reports orientation_infeasible when pose-lock retries fail but position-only probe succeeds', async () => {
    let callCount = 0;
    const mockSolve = (targetPose: any, seed: any, options?: any) => {
      callCount += 1;
      const mode = options?.mode || ((options?.orientationWeight ?? 0) > 0 ? 'pose_lock' : 'position_only');
      if (mode === 'position_only') {
        return {
          jointAngles: [0, 0, 0, 0, 0, 0],
          success: true,
          quality: {
            positionResidualM: 0.0008,
            orientationResidualRad: 0.02,
            weightedResidual: 0.0008,
            minSingularValue: 2e-4,
            conditionNumber: 500,
            activeConstraints: ['position'],
            jointParticipation: [1, 1, 1, 1, 1, 1],
            iterations: 12,
            damping: 0.01
          }
        };
      }
      return {
        jointAngles: [0, 0, 0, 0, 0, 0],
        success: false,
        error: 'Failed to converge (pos 0.0012m, ori 0.0350rad)',
        failureCategory: 'max_iterations',
        quality: {
          positionResidualM: 0.0012,
          // High orientation residual (> 0.026 rad tolerance) → genuinely orientation-infeasible
          orientationResidualRad: 0.035,
          weightedResidual: 0.0120,
          minSingularValue: 1e-4,
          conditionNumber: 1200,
          activeConstraints: ['position', 'orientation_hold'],
          jointParticipation: [1, 1, 1, 1, 1, 1],
          iterations: 120,
          damping: 0.01
        }
      };
    };

    jest.spyOn(HybridIKSolver.prototype, 'solvePoseWeighted').mockImplementation(mockSolve as any);
    jest.spyOn(InverseKinematics.prototype, 'solvePoseWeighted').mockImplementation(mockSolve as any);

    await useRobotStore.getState().moveToPosition({ x: -0.22, y: -0.0005, z: 0.311 });

    const state = useRobotStore.getState();
    expect(state.ikStatus?.success).toBe(false);
    expect(state.ikStatus?.failureCategory).toBe('orientation_infeasible');
    expect(state.ikStatus?.error).toContain('Pose lock infeasible');
    expect(callCount).toBeGreaterThanOrEqual(2);
  });

  test('classifies stage1 pose-lock interpolation failure as orientation_infeasible when position-only probe succeeds', async () => {
    jest.spyOn(HybridIKSolver.prototype, 'solvePoseWeighted').mockImplementation(() => ({
      jointAngles: [0, 0, 0, 0, 0, 0],
      success: true,
      branchId: 'SL_EU_WN',
      quality: {
        positionResidualM: 0.0008,
        orientationResidualRad: 0.0002,
        weightedResidual: 0.0008,
        minSingularValue: 2e-4,
        conditionNumber: 500,
        activeConstraints: ['position', 'orientation_hold'],
        jointParticipation: [1, 1, 1, 1, 1, 1],
        iterations: 12,
        damping: 0.01
      }
    }) as any);

    const plannerSpy = jest
      .spyOn(TrajectoryPlanner.prototype, 'planPoseLockedCartesianMove')
      .mockImplementation((_, __, targetOrientation) => (
        // High orientation residual (> 0.026 rad tolerance) to simulate genuine orientation infeasibility
        targetOrientation ? createFailedInterpolation(0.0018, 0.035) : createSuccessfulInterpolation()
      ) as any);

    await useRobotStore.getState().moveToPosition({ x: -0.22, y: -0.0005, z: 0.311 });

    const state = useRobotStore.getState();
    expect(state.ikStatus?.success).toBe(false);
    expect(state.ikStatus?.failureCategory).toBe('orientation_infeasible');
    expect(state.ikStatus?.error).toContain('Pose lock infeasible along Cartesian path');
    expect(plannerSpy).toHaveBeenCalled();
  });

  test('does not classify as invalid_target when best residual is within strict threshold', async () => {
    jest
      .spyOn(TrajectoryPlanner.prototype, 'planPoseLockedCartesianMove')
      .mockImplementation(() => createFailedInterpolation(0.0020));

    useRobotStore.setState({ cartesianMode: 'position_only' });
    await useRobotStore.getState().moveToPosition({ x: -0.22, y: -0.0005, z: 0.311 });

    const state = useRobotStore.getState();
    expect(state.ikStatus?.success).toBe(false);
    expect(state.ikStatus?.failureCategory).not.toBe('invalid_target');
    expect(state.ikStatus?.failureCategory).toBe('max_iterations');
  });

  test('fails early with queue_upload_failed when trajectory point budget cannot satisfy minimum quality floor', async () => {
    const plannerSpy = jest.spyOn(TrajectoryPlanner.prototype, 'planPoseLockedCartesianMove');
    useRobotStore.setState({
      firmwareConfig: createFirmwareConfig(20),
      reachabilityAtlasReady: false
    });

    await useRobotStore.getState().moveToPosition({ x: -0.30, y: 0.0, z: 0.311 });

    const state = useRobotStore.getState();
    expect(state.ikStatus?.success).toBe(false);
    expect(state.ikStatus?.failureCategory).toBe('queue_upload_failed');
    expect(state.ikStatus?.error).toContain('trajectory queue budget');
    expect(plannerSpy).not.toHaveBeenCalled();
  });

  test('rejects trajectories that would require silent joint clamping before queue upload', async () => {
    jest.spyOn(HybridIKSolver.prototype, 'solvePoseWeighted').mockImplementation(() => ({
      jointAngles: [0, 0, 0, 0, 0, 0],
      success: true,
      branchId: 'SL_EU_WN',
      quality: {
        positionResidualM: 0.0008,
        orientationResidualRad: 0.0002,
        weightedResidual: 0.0008,
        minSingularValue: 2e-4,
        conditionNumber: 500,
        activeConstraints: ['position', 'orientation_hold'],
        jointParticipation: [1, 1, 1, 1, 1, 1],
        iterations: 12,
        damping: 0.01
      }
    }) as any);

    const plannerSpy = jest
      .spyOn(TrajectoryPlanner.prototype, 'planPoseLockedCartesianMove')
      .mockImplementation(() => createOutOfLimitInterpolation() as any);

    await useRobotStore.getState().moveToPosition({ x: -0.22, y: -0.0005, z: 0.311 });

    const state = useRobotStore.getState();
    const serialManager = state.serialManager as any;
    expect(state.ikStatus?.success).toBe(false);
    expect(state.ikStatus?.failureCategory).toBe('joint_limit');
    expect(state.ikStatus?.error).toContain('violates firmware joint limits');
    expect(plannerSpy).toHaveBeenCalled();
    expect(serialManager.clearTrajectoryQueue).not.toHaveBeenCalled();
  });

  test('classifies generic device errors during stage2 queue upload as queue_upload_failed', async () => {
    jest.spyOn(HybridIKSolver.prototype, 'solvePoseWeighted').mockImplementation(() => ({
      jointAngles: [0, 0, 0, 0, 0, 0],
      success: true,
      branchId: 'SL_EU_WN',
      quality: {
        positionResidualM: 0.0008,
        orientationResidualRad: 0.0002,
        weightedResidual: 0.0008,
        minSingularValue: 2e-4,
        conditionNumber: 500,
        activeConstraints: ['position', 'orientation_hold'],
        jointParticipation: [1, 1, 1, 1, 1, 1],
        iterations: 12,
        damping: 0.01
      }
    }) as any);
    jest
      .spyOn(TrajectoryPlanner.prototype, 'planPoseLockedCartesianMove')
      .mockImplementation(() => createSuccessfulInterpolation() as any);

    const serialManager = useRobotStore.getState().serialManager as any;
    serialManager.enqueueTrajectoryPoint.mockRejectedValueOnce(new Error('Device returned ERROR'));

    await useRobotStore.getState().moveToPosition({ x: -0.22, y: -0.0005, z: 0.311 });

    const state = useRobotStore.getState();
    expect(state.ikStatus?.success).toBe(false);
    expect(state.ikStatus?.failureCategory).toBe('queue_upload_failed');
    expect(state.ikStatus?.errorCode).toBe('QUEUE_TRANSACTION_FAILED');
    expect(state.ikStatus?.failureCategory).not.toBe('max_iterations');
    expect(state.ikStatus?.error || '').toContain('Queue upload/run failed');
  });

  test('uses strict-valid stage1 fallback when stage2 times out', async () => {
    jest.spyOn(HybridIKSolver.prototype, 'solvePoseWeighted').mockImplementation(() => ({
      jointAngles: [0, 0, 0, 0, 0, 0],
      success: true,
      branchId: 'SL_EU_WN',
      quality: {
        positionResidualM: 0.0008,
        orientationResidualRad: 0.0002,
        weightedResidual: 0.0008,
        minSingularValue: 2e-4,
        conditionNumber: 500,
        activeConstraints: ['position', 'orientation_hold'],
        jointParticipation: [1, 1, 1, 1, 1, 1],
        iterations: 12,
        damping: 0.01
      }
    }) as any);

    let callIndex = 0;
    jest
      .spyOn(TrajectoryPlanner.prototype, 'planPoseLockedCartesianMove')
      .mockImplementation(() => {
        callIndex += 1;
        return callIndex === 1 ? createSuccessfulInterpolation() as any : createTimeoutInterpolation() as any;
      });

    await useRobotStore.getState().moveToPosition({ x: -0.22, y: -0.0005, z: 0.311 });

    const state = useRobotStore.getState();
    const serialManager = state.serialManager as any;
    expect(state.ikStatus?.success).toBe(true);
    expect(state.ikStatus?.errorCode).toBe('STAGE2_TIMEOUT_STAGE1_FALLBACK');
    expect(state.planningState).toBe('ready');
    expect(serialManager.clearTrajectoryQueue).toHaveBeenCalledTimes(1);
  });

  test('fails closed with stage2_timeout when stage1 fallback is not strict-valid', async () => {
    jest.spyOn(HybridIKSolver.prototype, 'solvePoseWeighted').mockImplementation(() => ({
      jointAngles: [0, 0, 0, 0, 0, 0],
      success: true,
      branchId: 'SL_EU_WN',
      quality: {
        positionResidualM: 0.0008,
        orientationResidualRad: 0.0002,
        weightedResidual: 0.0008,
        minSingularValue: 2e-4,
        conditionNumber: 500,
        activeConstraints: ['position', 'orientation_hold'],
        jointParticipation: [1, 1, 1, 1, 1, 1],
        iterations: 12,
        damping: 0.01
      }
    }) as any);

    let callIndex = 0;
    jest
      .spyOn(TrajectoryPlanner.prototype, 'planPoseLockedCartesianMove')
      .mockImplementation(() => {
        callIndex += 1;
        return callIndex === 1 ? createLooseSuccessfulInterpolation() as any : createTimeoutInterpolation() as any;
      });

    await useRobotStore.getState().moveToPosition({ x: -0.22, y: -0.0005, z: 0.311 });

    const state = useRobotStore.getState();
    const serialManager = state.serialManager as any;
    expect(state.ikStatus?.success).toBe(false);
    expect(state.ikStatus?.failureCategory).toBe('stage2_timeout');
    expect(state.ikStatus?.errorCode).toBe('STAGE2_TIMEOUT');
    expect(serialManager.clearTrajectoryQueue).not.toHaveBeenCalled();
  });

  test('uses strict-valid stage1 fallback when stage2 fails on strict final gate', async () => {
    jest.spyOn(HybridIKSolver.prototype, 'solvePoseWeighted').mockImplementation(() => ({
      jointAngles: [0, 0, 0, 0, 0, 0],
      success: true,
      branchId: 'SL_EU_WN',
      quality: {
        positionResidualM: 0.0008,
        orientationResidualRad: 0.0002,
        weightedResidual: 0.0008,
        minSingularValue: 2e-4,
        conditionNumber: 500,
        activeConstraints: ['position', 'orientation_hold'],
        jointParticipation: [1, 1, 1, 1, 1, 1],
        iterations: 12,
        damping: 0.01
      }
    }) as any);

    let callIndex = 0;
    jest
      .spyOn(TrajectoryPlanner.prototype, 'planPoseLockedCartesianMove')
      .mockImplementation(() => {
        callIndex += 1;
        return callIndex === 1
          ? createSuccessfulInterpolation() as any
          : createStrictGateFailedInterpolation() as any;
      });

    await useRobotStore.getState().moveToPosition({ x: -0.22, y: -0.0005, z: 0.311 });

    const state = useRobotStore.getState();
    const serialManager = state.serialManager as any;
    expect(state.ikStatus?.success).toBe(true);
    expect(state.ikStatus?.errorCode).toBe('STAGE2_STRICT_GATE_STAGE1_FALLBACK');
    expect(state.planningState).toBe('ready');
    expect(serialManager.clearTrajectoryQueue).toHaveBeenCalledTimes(1);
  });
});
