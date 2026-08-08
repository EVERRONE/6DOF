import { AgentCommandExecutor, BridgeCommandError } from '../AgentCommandExecutor';
import { useRobotStore } from '../../store/robotStore';
import { ConnectionStatus, RobotState } from '../../types/robot';

/**
 * These tests drive the real store, overriding only the actions that would
 * reach hardware. That matters: the guards being checked here depend on how
 * robotStore actually transitions, and a hand-rolled fake would have agreed
 * with my assumptions rather than with the code.
 */

const firmwareConfig = {
  version: 1,
  capabilities: { trajectoryQueue: true, trajectoryMaxPoints: 256, trajectoryPointFormat: 'hermite_v1' },
  joints: Array.from({ length: 6 }, () => ({
    min: -120,
    max: 120,
    stepsPerDeg: 100,
    invertDir: false,
    hasEndstop: true,
    homeTowardMin: false,
    homeLogicalDeg: 0,
    postHomeOffsetDeg: 0,
    urdfOffsetDeg: 0,
    cal: { scale: 1, offset: 0 }
  })),
  pulseWidthUs: 3,
  dirSetupUs: 5,
  endstopDebounceMs: 20
} as any;

const readyState = () => ({
  connectionStatus: ConnectionStatus.CONNECTED,
  robotState: RobotState.IDLE,
  motorsEnabled: true,
  homedJoints: { J1: true, J2: true, J3: true, J4: true, J5: true, J6: true },
  kinematicsFrameReady: true,
  firmwareConfig,
  currentAngles: { J1: 0, J2: -45, J3: 60, J4: -90, J5: -90, J6: 0 },
  moveInProgress: false,
  planningState: 'idle' as const,
  planningNotes: [],
  ikStatus: null
});

let moveSpy: jest.Mock;
let stopSpy: jest.Mock;

beforeEach(() => {
  moveSpy = jest.fn().mockResolvedValue(undefined);
  stopSpy = jest.fn().mockResolvedValue(undefined);
  useRobotStore.setState({
    ...readyState(),
    moveToPosition: moveSpy,
    emergencyStop: stopSpy
  } as any);
});

const armed = () => {
  const executor = new AgentCommandExecutor({ settleTimeoutMs: 200 });
  executor.arm(10);
  return executor;
};

const relativeParams = (overrides = {}) => ({
  direction: 'right' as const,
  frame: 'base' as const,
  distanceMm: 5,
  requestedDistanceMm: 5,
  clamped: false,
  keepOrientation: true,
  wait: true,
  ...overrides
});

const expectRejection = async (promise: Promise<unknown>, code: string) => {
  await expect(promise).rejects.toMatchObject({ code });
};

describe('arming', () => {
  it('refuses motion when not armed', async () => {
    const executor = new AgentCommandExecutor();
    await expectRejection(executor.moveRelative(relativeParams()), 'NOT_ARMED');
    expect(moveSpy).not.toHaveBeenCalled();
  });

  it('treats an expired window as not armed', async () => {
    const executor = new AgentCommandExecutor();
    executor.arm(-1);
    expect(executor.isArmed()).toBe(false);
    await expectRejection(executor.moveRelative(relativeParams()), 'NOT_ARMED');
  });

  it('never gates stop behind arming', async () => {
    const executor = new AgentCommandExecutor();
    await executor.stopMotion();
    expect(stopSpy).toHaveBeenCalled();
  });

  it('disarms after an emergency stop, so a human has to approve again', async () => {
    const executor = armed();
    expect(executor.isArmed()).toBe(true);
    await executor.stopMotion();
    expect(executor.isArmed()).toBe(false);
  });
});

describe('a latched emergency stop', () => {
  // Regression: emergencyStop sets robotState ESTOPPED and leaves it there.
  // moveToPosition does NOT refuse in that state — it sets MOVING and drives
  // the arm — so without this guard the agent got a failure report while the
  // arm actually moved.
  it('blocks a move instead of letting it through', async () => {
    useRobotStore.setState({ robotState: RobotState.ESTOPPED } as any);
    const executor = armed();

    await expectRejection(executor.moveRelative(relativeParams()), 'ESTOP_LATCHED');
    expect(moveSpy).not.toHaveBeenCalled();
  });

  it('is reported by get_status and readiness text', () => {
    useRobotStore.setState({ robotState: RobotState.ESTOPPED } as any);
    const executor = armed();

    const status = executor.buildStatus() as any;
    expect(status.emergencyStopLatched).toBe(true);
    expect(status.readyToMove).toMatch(/Emergency stop is latched/);
  });
});

describe('other refusals', () => {
  it('refuses while a move is already running', async () => {
    useRobotStore.setState({ moveInProgress: true } as any);
    await expectRejection(armed().moveRelative(relativeParams()), 'BUSY');
  });

  it('refuses while planning is in flight', async () => {
    useRobotStore.setState({ planningState: 'stage2_refine' } as any);
    await expectRejection(armed().moveRelative(relativeParams()), 'BUSY');
  });

  it('refuses when J2..J5 are not homed, and says so', async () => {
    useRobotStore.setState({
      homedJoints: { J1: true, J2: false, J3: true, J4: true, J5: true, J6: true }
    } as any);
    const executor = armed();
    await expect(executor.moveRelative(relativeParams())).rejects.toMatchObject({
      code: 'NOT_READY',
      message: expect.stringMatching(/homed/i)
    });
  });

  it('refuses an absolute target beyond the per-command ceiling', async () => {
    const executor = armed();
    await expectRejection(
      executor.moveToAbsolute({ targetMm: { x: 9999, y: 0, z: 0 }, keepOrientation: true, wait: true }),
      'TOO_FAR'
    );
    expect(moveSpy).not.toHaveBeenCalled();
  });
});

describe('move outcome', () => {
  const driveToArrival = () => {
    useRobotStore.setState({ moveInProgress: true, robotState: RobotState.MOVING } as any);
    useRobotStore.setState({ moveInProgress: false, robotState: RobotState.IDLE } as any);
  };

  it('reports arrival only after the completion signal', async () => {
    const executor = armed();
    const promise = executor.moveRelative(relativeParams());

    await Promise.resolve();
    driveToArrival();

    await expect(promise).resolves.toMatchObject({ outcome: 'arrived' });
  });

  it('passes a planning failure through verbatim rather than inventing a code', async () => {
    const executor = armed();
    const promise = executor.moveRelative(relativeParams());

    await Promise.resolve();
    useRobotStore.setState({
      planningState: 'failed',
      ikStatus: { success: false, error: 'Pose lock infeasible along Cartesian path at current limits.' },
      planningNotes: ['Mode=pose_lock']
    } as any);

    await expect(promise).rejects.toMatchObject({
      code: 'PLANNING_FAILED',
      message: 'Pose lock infeasible along Cartesian path at current limits.'
    });
  });

  it('reports an interruption when the stop happens during the move', async () => {
    const executor = armed();
    const promise = executor.moveRelative(relativeParams());

    await Promise.resolve();
    useRobotStore.setState({ moveInProgress: true, robotState: RobotState.MOVING } as any);
    useRobotStore.setState({ moveInProgress: false, robotState: RobotState.ESTOPPED } as any);

    await expectRejection(promise, 'STOPPED');
  });

  it('reports unknown rather than success when nothing settles in time', async () => {
    const executor = armed();
    const promise = executor.moveRelative(relativeParams());

    await Promise.resolve();
    useRobotStore.setState({ moveInProgress: true, robotState: RobotState.MOVING } as any);

    await expect(promise).rejects.toMatchObject({
      code: 'UNKNOWN_OUTCOME',
      message: expect.stringMatching(/may still be moving/)
    });
  });

  it('returns as soon as the queue starts when wait is false', async () => {
    const executor = armed();
    await expect(
      executor.moveRelative(relativeParams({ wait: false }))
    ).resolves.toMatchObject({ outcome: 'started' });
  });
});

describe('commanded target', () => {
  it('offsets 5 mm along -Y for "right" and locks the tool orientation', async () => {
    const executor = armed();
    const before = executor.computeCurrentPose()!;

    const promise = executor.moveRelative(relativeParams());
    await Promise.resolve();

    const [target, orientation] = moveSpy.mock.calls[0];
    expect(target.y).toBeCloseTo(before.position.y - 0.005, 9);
    expect(target.x).toBeCloseTo(before.position.x, 9);
    expect(target.z).toBeCloseTo(before.position.z, 9);

    // An explicit rotation is what forces pose_lock regardless of the app's
    // Cartesian mode.
    expect(orientation).toBeDefined();

    useRobotStore.setState({ moveInProgress: true } as any);
    useRobotStore.setState({ moveInProgress: false, robotState: RobotState.IDLE } as any);
    await promise;
  });

  it('honours the view frame yaw when resolving a direction', async () => {
    const executor = armed();
    executor.setViewYawDeg(90);
    const before = executor.computeCurrentPose()!;

    const promise = executor.moveRelative(relativeParams({ frame: 'view' }));
    await Promise.resolve();

    // Facing +Y, the operator's right hand points along +X.
    const [target] = moveSpy.mock.calls[0];
    expect(target.x).toBeCloseTo(before.position.x + 0.005, 9);
    expect(target.y).toBeCloseTo(before.position.y, 9);

    useRobotStore.setState({ moveInProgress: true } as any);
    useRobotStore.setState({ moveInProgress: false, robotState: RobotState.IDLE } as any);
    await promise;
  });
});

describe('preview_move', () => {
  it('sends nothing to the robot', () => {
    const executor = armed();
    const result = executor.previewMove(relativeParams()) as any;

    expect(moveSpy).not.toHaveBeenCalled();
    expect(result.executed).toBe(false);
    expect(result.resolved.worldDeltaMm).toEqual({ x: 0, y: -5, z: 0 });
  });

  it('works while not armed, so the frame can be checked before anything moves', () => {
    const executor = new AgentCommandExecutor();
    const result = executor.previewMove(relativeParams()) as any;
    expect(result.toMm).toBeDefined();
    expect(result.readyToMove).toMatch(/not armed/i);
  });

  it('echoes the configured yaw when the view frame is used', () => {
    const executor = new AgentCommandExecutor();
    executor.setViewYawDeg(45);
    const result = executor.previewMove(relativeParams({ frame: 'view' })) as any;
    expect(result.resolved.viewYawDeg).toBe(45);
  });

  it('reports base-frame requests as yaw-independent', () => {
    const executor = new AgentCommandExecutor();
    executor.setViewYawDeg(45);
    const result = executor.previewMove(relativeParams({ frame: 'base' })) as any;
    expect(result.resolved.viewYawDeg).toBe(0);
    expect(result.resolved.worldDeltaMm).toEqual({ x: 0, y: -5, z: 0 });
  });
});

describe('BridgeCommandError', () => {
  it('carries a code the broker can map to a status', () => {
    const error = new BridgeCommandError('BUSY', 'busy');
    expect(error.code).toBe('BUSY');
    expect(error).toBeInstanceOf(Error);
  });
});
