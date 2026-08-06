// Tests for the store's trajectory sender.
//
// This is the piece that turns a planned path into a stream of commands, and its
// correctness is not obvious: it has to keep the firmware queue fed without
// overrunning it, resend a point the firmware refused, wait for the arm to
// actually stop rather than for the last command to be accepted, and abandon
// everything cleanly on a cancel or a dropped cable.
//
// The store is driven against a fake serial port that answers commands the way the
// firmware would, so all of that is reachable without hardware.

import { useRobotStore } from './robotStore';
import { SerialManager } from '../communication/SerialManager';
import { ConnectionStatus, RobotState } from '../types/robot';
import { ExecutionState, Waypoint } from '../motion/types';
import { TrajectoryPlanner } from '../motion/TrajectoryPlanner';
import { ForwardKinematics } from '../kinematics/ForwardKinematics';
import { HOME_POSE_DEG } from '../kinematics/robotModel';
import { FakePort, FakeSerial, commandsOfType } from '../testUtils/fakeSerial';

const flush = () => new Promise<void>(resolve => setTimeout(resolve, 0));

/** Let the sender make progress across many awaits. */
async function settle(rounds = 60): Promise<void> {
  for (let k = 0; k < rounds; k++) await flush();
}

const STATUS_IDLE = 'STATUS IDLE 23 0 1 30 1\n';
const STATUS_MOVING = 'STATUS MOVING 20 1 1 30 1\n';

interface Harness {
  port: FakePort;
  manager: SerialManager;
}

/**
 * Connect the store to a fake port whose firmware accepts every move and reports
 * itself busy while there is anything to run.
 */
async function connectStore(
  respond?: (command: string) => string | null
): Promise<Harness> {
  const port = new FakePort();
  const serial = new FakeSerial(port);
  const manager = new SerialManager({ serial, autoReconnect: false });

  port.autoRespond =
    respond ??
    (command => (command.startsWith('J ') ? 'OK J 20\n' : null));

  await useRobotStore.getState().connect(manager);
  await flush();

  return { port, manager };
}

/** The `J` commands the currently planned trajectory should produce, in order. */
function plannedCommands(): string[] {
  const trajectory = useRobotStore.getState().trajectory!;
  const speed = useRobotStore.getState().plannerConfig.maxJointSpeed;

  return TrajectoryPlanner.flattenTrajectory(trajectory).map(point => {
    const angles = point.jointAngles.map(v => v.toFixed(3)).join(' ');
    return `J ${angles} ${speed.toFixed(2)}`;
  });
}

function makeWaypoint(angles: number[], label: string): Waypoint {
  return {
    id: label,
    position: ForwardKinematics.position(angles),
    jointAngles: [...angles],
    speed: 40,
    label
  };
}

/** Plan a short two-waypoint path and return how many points it holds. */
function planShortPath(): number {
  const store = useRobotStore.getState();
  store.clearWaypoints();
  store.updatePlannerConfig({ interpolationMode: 'joint', pointsPerSecond: 4 });
  store.addWaypoint(makeWaypoint([0, 12, 55, 129, 131, 0], 'a'));
  store.addWaypoint(makeWaypoint([0, 20, 58, 129, 131, 0], 'b'));
  store.planTrajectory();

  const trajectory = useRobotStore.getState().trajectory;
  expect(trajectory).not.toBeNull();
  // What the sender actually emits is the flattened list, which drops the
  // duplicated point where two segments meet. trajectory.pointCount counts that
  // seam twice, so comparing against it only passed while the seam happened to
  // fall on distinct timestamps.
  return TrajectoryPlanner.flattenTrajectory(trajectory!).length;
}

beforeEach(() => {
  useRobotStore.setState({
    connectionStatus: ConnectionStatus.DISCONNECTED,
    connectionDetail: null,
    serialManager: null,
    robotState: RobotState.IDLE,
    firmwareStatus: null,
    firmwareStatusAt: 0,
    motorsEnabled: false,
    waypoints: [],
    trajectory: null,
    trajectoryPositions: [],
    executionState: ExecutionState.IDLE,
    ikStatus: null,
    currentAngles: {
      J1: HOME_POSE_DEG[0], J2: HOME_POSE_DEG[1], J3: HOME_POSE_DEG[2],
      J4: HOME_POSE_DEG[3], J5: HOME_POSE_DEG[4], J6: HOME_POSE_DEG[5]
    }
  });
});

afterEach(async () => {
  const { serialManager } = useRobotStore.getState();
  if (serialManager) {
    useRobotStore.getState().cancelExecution();
    await serialManager.disconnect().catch(() => undefined);
    serialManager.dispose();
  }
  useRobotStore.setState({ serialManager: null });
});

// ---------------------------------------------------------------------------

describe('store: connection', () => {
  it('tracks the manager state and asks the firmware for a report', async () => {
    const { port } = await connectStore();

    expect(useRobotStore.getState().connectionStatus).toBe(ConnectionStatus.CONNECTED);
    expect(port.written.join('')).toContain('Q');
  });

  it('takes the robot state from the firmware, not from its own guesses', async () => {
    const { port } = await connectStore();

    port.push(STATUS_MOVING);
    await flush();
    expect(useRobotStore.getState().robotState).toBe(RobotState.MOVING);

    port.push('STATUS ESTOP 23 0 0 30 1\n');
    await flush();
    expect(useRobotStore.getState().robotState).toBe(RobotState.ESTOPPED);
  });

  it('clears everything it knew when the link drops', async () => {
    const { port } = await connectStore();

    port.push(STATUS_IDLE);
    await flush();
    useRobotStore.setState({ motorsEnabled: true });
    expect(useRobotStore.getState().firmwareStatus).not.toBeNull();

    port.endStream();
    await settle(10);

    const state = useRobotStore.getState();
    expect(state.firmwareStatus).toBeNull();
    expect(state.motorsEnabled).toBe(false);
    expect(state.connectionStatus).not.toBe(ConnectionStatus.CONNECTED);
  });
});

describe('store: streaming a trajectory', () => {
  it('sends every planned point, in order', async () => {
    const { port } = await connectStore();
    const pointCount = planShortPath();

    const run = useRobotStore.getState().executeTrajectory();

    // Report idle once so the sender can finish waiting.
    await settle(40);
    port.push(STATUS_IDLE);
    await run;

    const moves = commandsOfType(port, 'J ');
    expect(moves).toHaveLength(pointCount);
    expect(useRobotStore.getState().executionState).toBe(ExecutionState.COMPLETED);
    expect(useRobotStore.getState().executionProgress.overallProgress).toBe(100);
  });

  it('resends a point the firmware refused, rather than skipping it', async () => {
    // Answer BUSY the first time each of the first three points is offered.
    const refused = new Map<string, number>();
    const { port } = await connectStore(command => {
      if (!command.startsWith('J ')) return null;
      const seen = refused.get(command) ?? 0;
      refused.set(command, seen + 1);
      return seen === 0 && refused.size <= 3 ? 'BUSY 0\n' : 'OK J 20\n';
    });

    const pointCount = planShortPath();
    const expected = plannedCommands();

    const run = useRobotStore.getState().executeTrajectory();
    await settle(120);
    port.push(STATUS_IDLE);
    await run;

    const moves = commandsOfType(port, 'J ');

    // Every planned point arrived. Two points can hold the same angles, so this
    // checks each planned command rather than counting distinct ones.
    for (const command of expected) {
      expect(moves).toContain(command);
    }
    // ...and the refused ones were sent more than once.
    expect(moves.length).toBeGreaterThan(pointCount);
    expect(useRobotStore.getState().executionState).toBe(ExecutionState.COMPLETED);
  });

  it('waits for the arm to stop, not for the last command to be accepted', async () => {
    const { port } = await connectStore();
    planShortPath();

    const run = useRobotStore.getState().executeTrajectory();
    await settle(40);

    // Every point is accepted, but the firmware still has a queue to drain.
    port.push(STATUS_MOVING);
    await settle(10);
    expect(useRobotStore.getState().executionState).toBe(ExecutionState.EXECUTING);

    port.push(STATUS_IDLE);
    await run;
    expect(useRobotStore.getState().executionState).toBe(ExecutionState.COMPLETED);
  });

  it('ignores a stale idle report from before the last command', async () => {
    const { port } = await connectStore();
    planShortPath();

    // An idle status that arrived early must not be read as "finished".
    port.push(STATUS_IDLE);
    await flush();

    const run = useRobotStore.getState().executeTrajectory();
    await settle(40);

    expect(useRobotStore.getState().executionState).toBe(ExecutionState.EXECUTING);

    port.push(STATUS_IDLE);
    await run;
    expect(useRobotStore.getState().executionState).toBe(ExecutionState.COMPLETED);
  });

  it('does nothing without a planned trajectory', async () => {
    const { port } = await connectStore();
    await useRobotStore.getState().executeTrajectory();

    expect(commandsOfType(port, 'J ')).toHaveLength(0);
  });
});

describe('store: stopping', () => {
  it('cancelling stops the arm and is not reported as an error', async () => {
    const { port } = await connectStore();
    planShortPath();

    const run = useRobotStore.getState().executeTrajectory();
    await settle(6);

    useRobotStore.getState().cancelExecution();
    await run;
    await settle(5);

    // The firmware is holding queued moves, so the arm has to be told too.
    expect(commandsOfType(port, 'A')).toHaveLength(1);

    const state = useRobotStore.getState();
    expect(state.executionState).toBe(ExecutionState.IDLE);
    expect(state.executionState).not.toBe(ExecutionState.ERROR);
  });

  it('pausing stops the arm and skips no point on resume', async () => {
    // Hold the firmware queue closed after the first point, so the sender is
    // parked in its retry loop and there is a deterministic window to pause in.
    let accept = 1;
    const { port } = await connectStore(command => {
      if (!command.startsWith('J ')) return null;
      if (accept > 0) {
        accept--;
        return 'OK J 20\n';
      }
      return 'BUSY 0\n';
    });

    planShortPath();
    const expected = plannedCommands();

    const run = useRobotStore.getState().executeTrajectory();
    await settle(20);

    useRobotStore.getState().pauseExecution();
    await settle(20);

    // A pause has to reach the arm; the firmware's queue would otherwise keep
    // running everything already accepted.
    expect(commandsOfType(port, 'A').length).toBeGreaterThanOrEqual(1);

    accept = 999; // queue drained
    useRobotStore.getState().resumeExecution();
    await settle(120);
    port.push(STATUS_IDLE);
    await run;

    // Nothing was dropped over the pause.
    const moves = commandsOfType(port, 'J ');
    for (const command of expected) {
      expect(moves).toContain(command);
    }
    expect(useRobotStore.getState().executionState).toBe(ExecutionState.COMPLETED);
  });

  it('abandons a run when the link drops, without reporting an error', async () => {
    const { port } = await connectStore();
    planShortPath();

    const run = useRobotStore.getState().executeTrajectory();
    await settle(6);

    port.endStream();
    await run;
    await settle(5);

    const state = useRobotStore.getState();
    expect(state.executionState).toBe(ExecutionState.IDLE);
    expect(state.firmwareStatus).toBeNull();
  });

  it('emergency stop halts the sender and latches the state', async () => {
    const { port } = await connectStore();
    planShortPath();

    const run = useRobotStore.getState().executeTrajectory();
    await settle(6);

    await useRobotStore.getState().emergencyStop();
    await run;

    expect(commandsOfType(port, 'S')).toHaveLength(1);
    expect(useRobotStore.getState().robotState).toBe(RobotState.ESTOPPED);
    expect(useRobotStore.getState().executionState).toBe(ExecutionState.IDLE);
  });
});

describe('store: bench controls', () => {
  it('adopts the reported position as the target on connect', async () => {
    // Regression, and the most dangerous thing in the old UI: targetAngles started
    // at all zeros and was never seeded, so the first "Move to Target" click after
    // connecting swung J5 220 degrees and J4 129 degrees at once.
    const { port } = await connectStore();

    port.push('POS 0.00 5.00 55.00 129.00 220.00 0.00\n');
    await flush();

    expect(useRobotStore.getState().targetAngles).toEqual({
      J1: 0, J2: 5, J3: 55, J4: 129, J5: 220, J6: 0
    });
  });

  it('does not keep overwriting the target as the arm moves', async () => {
    const { port } = await connectStore();

    port.push('POS 0.00 5.00 55.00 129.00 220.00 0.00\n');
    await flush();

    useRobotStore.getState().setTargetAngles({ J2: 30 });
    port.push('POS 0.00 6.00 55.00 129.00 220.00 0.00\n');
    await flush();

    // Seeding is one-shot; after that the target is the operator's to set.
    expect(useRobotStore.getState().targetAngles.J2).toBe(30);
  });

  it('re-seeds the target after a reconnect', async () => {
    const { port } = await connectStore();

    port.push('POS 0.00 5.00 55.00 129.00 220.00 0.00\n');
    await flush();
    useRobotStore.getState().setTargetAngles({ J2: 30 });

    port.endStream();
    await settle(10);

    // The controller restarts on a reconnect, so stale targets must not survive.
    await useRobotStore.getState().connect(useRobotStore.getState().serialManager!);
    await flush();
    port.push('POS 0.00 5.00 55.00 129.00 220.00 0.00\n');
    await flush();

    expect(useRobotStore.getState().targetAngles.J2).toBe(5);
  });

  it('jogs a joint relative to the target, so repeats accumulate', async () => {
    const { port } = await connectStore();
    port.push('POS 0.00 5.00 55.00 129.00 220.00 0.00\n');
    await flush();

    await useRobotStore.getState().jogJoint('J2', 5);
    await useRobotStore.getState().jogJoint('J2', 5);
    await flush();

    expect(useRobotStore.getState().targetAngles.J2).toBe(15);

    const moves = commandsOfType(port, 'J ');
    expect(moves).toHaveLength(2);
    expect(moves[1]).toContain('15.000');
  });

  it('clamps a jog at the joint limit and says so', async () => {
    const { port } = await connectStore();
    port.push('POS 0.00 5.00 55.00 129.00 220.00 0.00\n');
    await flush();

    // J2 stops at 60.
    await useRobotStore.getState().jogJoint('J2', 500);
    await flush();

    expect(useRobotStore.getState().targetAngles.J2).toBe(60);
    expect(
      useRobotStore.getState().events.some(e => /clamped/i.test(e.text))
    ).toBe(true);
  });

  it('takes the motor state from the firmware, not from its own command', async () => {
    const { port } = await connectStore();

    await useRobotStore.getState().enableMotors(true);
    await flush();

    // The E command went out, but nothing is assumed until the arm confirms.
    expect(commandsOfType(port, 'E ')).toHaveLength(1);
    expect(useRobotStore.getState().motorsEnabled).toBe(false);

    port.push('STATUS IDLE 23 0 1 30 1\n');
    await flush();
    expect(useRobotStore.getState().motorsEnabled).toBe(true);

    port.push('STATUS IDLE 23 0 1 30 0\n');
    await flush();
    expect(useRobotStore.getState().motorsEnabled).toBe(false);
  });

  it('logs what the arm says, including errors', async () => {
    const { port } = await connectStore();

    port.push('ERROR Endstop triggered during move on J3\n');
    port.push('HOMED 2\n');
    await flush();

    const events = useRobotStore.getState().events;

    // This used to go only to console.error, which hid it from the one person
    // standing next to the arm.
    expect(
      events.some(e => e.kind === 'error' && /endstop triggered/i.test(e.text))
    ).toBe(true);
    expect(events.some(e => e.kind === 'ok' && /J2 homed/.test(e.text))).toBe(true);
  });

  it('sends a raw command as typed, and logs it', async () => {
    const { port } = await connectStore();

    await useRobotStore.getState().sendRawCommand('  J 0 0 10 0 0 0 5  ');
    await flush();

    expect(port.lastCommand()).toBe('J 0 0 10 0 0 0 5');
    expect(
      useRobotStore.getState().events.some(e => e.kind === 'sent' && e.text === 'J 0 0 10 0 0 0 5')
    ).toBe(true);
  });

  it('ignores an empty raw command', async () => {
    const { port } = await connectStore();
    const before = port.written.length;

    await useRobotStore.getState().sendRawCommand('   ');
    expect(port.written).toHaveLength(before);
  });

  it('bounds the event log', async () => {
    const { port } = await connectStore();

    for (let k = 0; k < 400; k++) {
      port.push(`ERROR problem ${k}\n`);
    }
    await settle(20);

    const events = useRobotStore.getState().events;
    expect(events.length).toBeLessThanOrEqual(200);
    // The newest are the ones kept.
    expect(events[events.length - 1].text).toContain('399');
  });
});

describe('store: Cartesian moves', () => {
  it('sends the IK solution when the target is reachable', async () => {
    const { port } = await connectStore();

    const target = ForwardKinematics.position([0, 10, 55, 129, 131, 0]);
    await useRobotStore.getState().moveToPosition(target);
    await flush();

    expect(commandsOfType(port, 'J ')).toHaveLength(1);
    expect(useRobotStore.getState().ikStatus?.success).toBe(true);
    // The marker in the 3D view is driven by this.
    expect(useRobotStore.getState().targetPosition).toBeNull();
  });

  it('sends nothing for an unreachable target, and reports why', async () => {
    const { port } = await connectStore();

    await useRobotStore.getState().moveToPosition({ x: 3, y: 3, z: 3 });
    await flush();

    expect(commandsOfType(port, 'J ')).toHaveLength(0);

    const ik = useRobotStore.getState().ikStatus;
    expect(ik?.success).toBe(false);
    expect(ik?.error).toMatch(/not reachable/i);
    expect(ik?.residualError).toBeGreaterThan(0);
  });

  it('tracks the tool position from reported joint angles', async () => {
    const { port } = await connectStore();

    // Report the home pose itself rather than a copy of its numbers, so the
    // test keeps comparing like with like when the pose is recalibrated.
    port.push(`POS ${HOME_POSE_DEG.map(v => v.toFixed(2)).join(' ')}\n`);
    await flush();

    const position = useRobotStore.getState().currentPosition;
    expect(position).not.toBeNull();

    const expected = ForwardKinematics.position(HOME_POSE_DEG);
    expect(position!.x).toBeCloseTo(expected.x, 6);
    expect(position!.z).toBeCloseTo(expected.z, 6);
  });
});

describe('store: a path needs a datum', () => {
  // A path is a list of absolute joint angles. If the firmware says its position
  // is not trusted - not homed since power-up, or steps possibly lost to a stop -
  // those angles are measured from a datum that does not exist. The firmware will
  // not stop this: it accepts J commands whenever the motors are on and no stop
  // is latched, reports the doubt in STATUS, and leaves the call to the host.

  const STATUS_UNTRUSTED = 'STATUS IDLE 23 0 0 0 1\n';
  const STATUS_MOTORS_OFF = 'STATUS IDLE 23 0 1 30 0\n';

  it('refuses to execute when the position is not trusted', async () => {
    const { port } = await connectStore();
    planShortPath();

    port.push(STATUS_UNTRUSTED);
    await flush();

    await useRobotStore.getState().executeTrajectory();

    expect(commandsOfType(port, 'J ')).toHaveLength(0);
    expect(useRobotStore.getState().executionState).toBe(ExecutionState.IDLE);
    expect(
      useRobotStore.getState().events.some(e => e.kind === 'error' && /homed/i.test(e.text))
    ).toBe(true);
  });

  it('refuses to execute with the motors off', async () => {
    const { port } = await connectStore();
    planShortPath();

    port.push(STATUS_MOTORS_OFF);
    await flush();

    await useRobotStore.getState().executeTrajectory();

    expect(commandsOfType(port, 'J ')).toHaveLength(0);
    expect(useRobotStore.getState().executionState).toBe(ExecutionState.IDLE);
  });

  it('refuses to teach a waypoint from an untrusted position', async () => {
    const { port } = await connectStore();
    useRobotStore.getState().clearWaypoints();

    port.push(STATUS_UNTRUSTED);
    port.push(`POS ${HOME_POSE_DEG.map(v => v.toFixed(2)).join(' ')}\n`);
    await flush();

    useRobotStore.getState().teachCurrentPosition('should not appear');

    expect(useRobotStore.getState().waypoints).toHaveLength(0);
  });

  it('runs once the arm is homed', async () => {
    const { port } = await connectStore();
    const pointCount = planShortPath();

    // Untrusted first, then homed: the gate must lift, not latch.
    port.push(STATUS_UNTRUSTED);
    await flush();
    port.push(STATUS_IDLE);
    await flush();

    const run = useRobotStore.getState().executeTrajectory();
    await settle(40);
    port.push(STATUS_IDLE);
    await run;

    expect(commandsOfType(port, 'J ')).toHaveLength(pointCount);
    expect(useRobotStore.getState().executionState).toBe(ExecutionState.COMPLETED);
  });
});
