import { useRobotStore } from '../robotStore';
import { ConnectionStatus, FirmwareConfig, RobotState } from '../../types/robot';

jest.mock('../../communication/SerialManager', () => {
  class MockSerialManager {
    static lastInstance: MockSerialManager | null = null;

    private callback: ((msg: any) => void) | null = null;

    constructor() {
      MockSerialManager.lastInstance = this;
    }

    connect = jest.fn(async () => {});
    disconnect = jest.fn(async () => {});
    onMessage = jest.fn((cb: (msg: any) => void) => {
      this.callback = cb;
    });
    queryConfig = jest.fn(async () => {});
    queryTrajectoryQueue = jest.fn(async () => {});
    queryPosition = jest.fn(async () => {});
    moveToAngles = jest.fn(async () => {});
    homeJoints = jest.fn(async () => {});
    enableMotors = jest.fn(async () => {});
    emergencyStop = jest.fn(async () => {});

    emit(msg: any): void {
      this.callback?.(msg);
    }
  }

  return { SerialManager: MockSerialManager };
});

const createFirmwareConfig = (): FirmwareConfig => ({
  version: 1,
  capabilities: {
    trajectoryQueue: true,
    trajectoryMaxPoints: 256,
    trajectoryPointFormat: 'hermite_v1'
  },
  joints: [
    {
      min: -180,
      max: 180,
      stepsPerDeg: 1,
      invertDir: false,
      hasEndstop: false,
      homeTowardMin: true,
      homeLogicalDeg: 0,
      postHomeOffsetDeg: 0,
      urdfOffsetDeg: 0,
      cal: { scale: 1, offset: 0 }
    },
    {
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
    },
    {
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
    },
    {
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
    },
    {
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
    },
    {
      min: -180,
      max: 180,
      stepsPerDeg: 1,
      invertDir: false,
      hasEndstop: false,
      homeTowardMin: true,
      homeLogicalDeg: 0,
      postHomeOffsetDeg: 0,
      urdfOffsetDeg: 0,
      cal: { scale: 1, offset: 0 }
    }
  ],
  pulseWidthUs: 2,
  dirSetupUs: 2,
  endstopDebounceMs: 3
});

const getMockManager = (): any => {
  const managerModule = jest.requireMock('../../communication/SerialManager');
  return managerModule.SerialManager.lastInstance;
};

describe('robotStore Cartesian marker sync', () => {
  beforeEach(() => {
    useRobotStore.setState({
      connectionStatus: ConnectionStatus.DISCONNECTED,
      serialManager: null,
      robotState: RobotState.IDLE,
      currentAngles: { J1: 0, J2: 0, J3: 0, J4: 0, J5: 0, J6: 0 },
      targetAngles: { J1: 0, J2: 0, J3: 0, J4: 0, J5: 0, J6: 0 },
      endstopState: { J1: false, J2: false, J3: false, J4: false, J5: false, J6: false },
      homedJoints: { J1: false, J2: false, J3: false, J4: false, J5: false, J6: false },
      motorsEnabled: false,
      moveInProgress: false,
      moveTargetSnapshot: null,
      moveStableCount: 0,
      firmwareConfig: null,
      kinematicsFrameReady: false,
      currentPosition: null,
      targetPosition: null,
      ikStatus: null
    });
  });

  test('computes current position in URDF frame once CFG and POS are received', async () => {
    await useRobotStore.getState().connect();
    const manager = getMockManager();
    expect(manager).toBeTruthy();

    manager.emit({
      type: 'CFG',
      data: createFirmwareConfig(),
      timestamp: Date.now()
    });
    manager.emit({
      type: 'POS',
      data: { J1: 0, J2: 0, J3: 0, J4: 0, J5: 0, J6: 0 },
      timestamp: Date.now()
    });

    const { currentPosition, kinematicsFrameReady } = useRobotStore.getState();
    expect(kinematicsFrameReady).toBe(true);
    expect(currentPosition).not.toBeNull();
    expect(currentPosition?.x).toBeCloseTo(-0.20223, 3);
    expect(currentPosition?.y).toBeCloseTo(-0.00048, 3);
    expect(currentPosition?.z).toBeCloseTo(0.31105, 3);
  });

  test('snaps target position to current position when movement settles', async () => {
    await useRobotStore.getState().connect();
    const manager = getMockManager();
    expect(manager).toBeTruthy();

    manager.emit({
      type: 'CFG',
      data: createFirmwareConfig(),
      timestamp: Date.now()
    });

    useRobotStore.setState({
      robotState: RobotState.MOVING,
      moveInProgress: true,
      moveTargetSnapshot: { J1: 0, J2: 0, J3: 0, J4: 0, J5: 0, J6: 0 },
      moveStableCount: 2,
      targetPosition: { x: 0, y: 0, z: 0 }
    });

    manager.emit({
      type: 'POS',
      data: { J1: 0, J2: 0, J3: 0, J4: 0, J5: 0, J6: 0 },
      timestamp: Date.now()
    });

    const state = useRobotStore.getState();
    expect(state.robotState).toBe(RobotState.IDLE);
    expect(state.moveInProgress).toBe(false);
    expect(state.currentPosition).not.toBeNull();
    expect(state.targetPosition).toEqual(state.currentPosition);
  });
});
