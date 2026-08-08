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
import { TrajectoryPlanner, DEFAULT_PLANNER_CONFIG, resolveWaypoint } from '../motion/TrajectoryPlanner';
import { rotationLog, multiply3, transpose3 } from '../kinematics/linalg';
import { ForwardKinematics } from '../kinematics/ForwardKinematics';
import { JOINT_LIMITS_DEG, HOME_POSE_DEG, degToRad } from '../kinematics/robotModel';
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

/**
 * The `J` commands the currently planned trajectory should produce, in order.
 *
 * The speed is per point - the joint travel that point costs over the time the
 * planner allotted it - because one figure for the whole path cannot deliver a
 * constant tool speed. Derived here the same way the sender derives it, so this
 * checks the angles and the ordering rather than restating the arithmetic.
 */
function plannedCommands(): string[] {
  const trajectory = useRobotStore.getState().trajectory!;
  const points = TrajectoryPlanner.flattenTrajectory(trajectory);

  return points.map((point, i) => {
    const angles = point.jointAngles.map(v => v.toFixed(3)).join(' ');

    let speed = DEFAULT_PLANNER_CONFIG.defaultSpeed;
    if (i > 0) {
      const dt = point.time - points[i - 1].time;
      if (dt > 1e-6) {
        speed = Math.max(
          ...point.jointAngles.map((v, j) => Math.abs(v - points[i - 1].jointAngles[j]))
        ) / dt;
      }
    }
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
    trajectoryCollision: null,
    axisLimits: [],
    ikStatus: null,
    events: [],
    // Cartesian state. Leaving these out let a tool lock set by one test carry
    // into the next, which is the kind of failure that reads as a bug in the
    // code under test rather than in the fixture.
    currentPosition: null,
    currentRotation: null,
    targetPosition: null,
    toolLocked: false,
    lockedRotation: null,
    cartesianMode: 'linear',
    cartesianSpeed: 50,
    cartesianAccel: 200,
    workObjects: [],
    activeWorkObject: null,
    ioState: null,
    ioNames: { outputs: [], inputs: [] },
    jogFrame: 'base',
    jogStepMm: 5,
    jogStepDeg: 5,
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

    // Read the limit rather than repeating it: this is a test of clamping, and
    // hardcoding the number turns it into a test of one particular limit table.
    const j2Max = JOINT_LIMITS_DEG.max[1];
    await useRobotStore.getState().jogJoint('J2', 500);
    await flush();

    expect(useRobotStore.getState().targetAngles.J2).toBe(j2Max);
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

/**
 * Park the arm clear of its wrist singularity, and report the pose.
 *
 * At the parked pose J5 sits at 131 degrees, which is URDF zero for that joint -
 * exactly where a spherical wrist loses a degree of freedom. There J4 and J6
 * turn the tool about the same axis, so only their sum matters and the split
 * between them is free. A linear move from there is refused, deliberately, which
 * makes it the wrong pose to test ordinary linear motion from.
 *
 * 20 degrees away is well clear: the worst per-sample joint step falls from 44
 * degrees to 1.9.
 */
function offSingularity(): number[] {
  const angles = [...HOME_POSE_DEG];
  angles[4] = 151;
  useRobotStore.setState({
    currentAngles: {
      J1: angles[0], J2: angles[1], J3: angles[2],
      J4: angles[3], J5: angles[4], J6: angles[5]
    }
  });
  return angles;
}

describe('store: Cartesian moves', () => {
  it('sends one move in joint mode', async () => {
    const { port } = await connectStore();
    useRobotStore.getState().setCartesianMode('joint');

    const target = ForwardKinematics.position([0, 10, 55, 129, 131, 0]);
    await useRobotStore.getState().moveToPosition(target);
    await settle();

    expect(commandsOfType(port, 'J ')).toHaveLength(1);
    expect(useRobotStore.getState().ikStatus?.success).toBe(true);
    // The marker in the 3D view is driven by this.
    expect(useRobotStore.getState().targetPosition).toBeNull();
  });

  it('streams the straight line in linear mode, and holds it', async () => {
    const { port } = await connectStore();

    const start = offSingularity();
    const from = ForwardKinematics.position(start);
    const to = { x: from.x, y: from.y + 0.1, z: from.z };

    await useRobotStore.getState().moveToPosition(to);
    await settle(400);

    const moves = commandsOfType(port, 'J ');
    // 100 mm at the 2 mm chord limit. One solve and one move is the behaviour
    // this replaced.
    expect(moves.length).toBeGreaterThan(20);

    // Every point sent has to lie on the line, not merely the last one. A pure
    // move in Y must leave X and Z alone the whole way; the single-move version
    // wandered 7 mm in X and 4 mm in Z at the midpoint.
    for (const move of moves) {
      const angles = move.split(/\s+/).slice(1, 7).map(Number);
      const p = ForwardKinematics.position(angles);
      expect(Math.abs(p.x - from.x) * 1000).toBeLessThan(1);
      expect(Math.abs(p.z - from.z) * 1000).toBeLessThan(1);
    }
  });

  it('holds the tool orientation at every point, not only at the ends', async () => {
    const { port } = await connectStore();

    // This is the whole reason the linear path exists. Solving only the two ends
    // and letting the joints run proportionally satisfies the orientation at
    // both of them and tips the tool 11.13 degrees in between, because nothing
    // constrains the middle.
    const start = offSingularity();
    const from = ForwardKinematics.position(start);
    const R0 = ForwardKinematics.solveRad(degToRad(start)).rotation;

    // The lock captures the orientation from the reported angles itself.
    useRobotStore.getState().setToolLocked(true);
    expect(useRobotStore.getState().toolLocked).toBe(true);

    await useRobotStore.getState().moveToPosition({ x: from.x, y: from.y + 0.1, z: from.z });
    await settle(400);

    const moves = commandsOfType(port, 'J ');
    expect(moves.length).toBeGreaterThan(20);

    let worst = 0;
    for (const move of moves) {
      const angles = move.split(/\s+/).slice(1, 7).map(Number);
      const R = ForwardKinematics.solveRad(degToRad(angles)).rotation;
      const w = rotationLog(multiply3(R0, transpose3(R)));
      worst = Math.max(worst, (Math.hypot(w.x, w.y, w.z) * 180) / Math.PI);
    }

    // Well inside the solver's own 0.5 degree acceptance, and two orders of
    // magnitude better than the 11.13 it replaced.
    expect(worst).toBeLessThan(0.5);
  });

  it('turns the wrist into position before a line that starts at the singularity', async () => {
    const { port } = await connectStore();

    // From the parked pose, where J5 is at URDF zero and the wrist has lost a
    // degree of freedom: J4 and J6 turn the tool about the same axis, so solving
    // forwards moves 44 degrees from one into the other between two samples 2 mm
    // apart. Solved from the far end instead - which is 30 degrees clear of the
    // singularity - the line comes out smooth, and what it costs is that the
    // wrist has to be turned into the right configuration first.
    const start = [...HOME_POSE_DEG];
    const from = ForwardKinematics.position(start);
    useRobotStore.getState().setToolLocked(true);

    expect(useRobotStore.getState().toolLocked).toBe(true);

    // The reconfiguration is a move of its own, and the line does not start
    // until the arm has finished it - so the fake firmware has to report itself
    // idle, exactly as the real one would.
    const run = useRobotStore.getState().moveToPosition({
      x: from.x, y: from.y + 0.1, z: from.z
    });
    await settle(40);
    port.push(STATUS_IDLE);
    await run;
    await settle(400);

    const moves = commandsOfType(port, 'J ');
    expect(moves.length).toBeGreaterThan(20);

    // The first command is the reconfiguration: a large wrist turn that leaves
    // the tool exactly where it is.
    const first = moves[0].split(/\s+/).slice(1, 7).map(Number);
    const swing = Math.max(...first.map((v, i) => Math.abs(v - start[i])));
    expect(swing).toBeGreaterThan(20);
    expect(
      Math.hypot(
        ...(['x', 'y', 'z'] as const).map(
          k => ForwardKinematics.position(first)[k] - from[k]
        )
      ) * 1000
    ).toBeLessThan(0.5);

    expect(
      useRobotStore.getState().events.some(e => /Turning the wrist/i.test(e.text))
    ).toBe(true);
  });

  it('takes the same move in joint mode, since only the path is the problem', async () => {
    const { port } = await connectStore();

    const from = ForwardKinematics.position([...HOME_POSE_DEG]);
    useRobotStore.getState().setToolLocked(true);
    useRobotStore.getState().setCartesianMode('joint');

    await useRobotStore.getState().moveToPosition({ x: from.x, y: from.y + 0.1, z: from.z });
    await settle();

    // The destination is perfectly reachable with the orientation held - it is
    // getting there in a straight line that is not.
    expect(commandsOfType(port, 'J ')).toHaveLength(1);
    expect(useRobotStore.getState().ikStatus?.success).toBe(true);
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

describe('store: tool orientation lock', () => {
  async function homedStore() {
    const { port } = await connectStore();
    port.push(STATUS_IDLE);
    port.push(`POS ${HOME_POSE_DEG.map(v => v.toFixed(2)).join(' ')}\n`);
    await flush();
    return port;
  }

  /** Joint angles from the last J line the store sent. */
  function lastCommandedAngles(port: ReturnType<typeof connectStore> extends Promise<infer H> ? any : any): number[] {
    const moves = commandsOfType(port, 'J ');
    const parts = moves[moves.length - 1].trim().split(/\s+/);
    return parts.slice(1, 7).map(Number);
  }

  function tumbleDeg(from: number[], to: number[]): number {
    const Ra = ForwardKinematics.solveRad(from.map(d => (d * Math.PI) / 180)).rotation;
    const Rb = ForwardKinematics.solveRad(to.map(d => (d * Math.PI) / 180)).rotation;
    const w = rotationLog(multiply3(Rb, transpose3(Ra)));
    return (Math.hypot(w.x, w.y, w.z) * 180) / Math.PI;
  }

  it('holds the tool through a Cartesian move when locked', async () => {
    const port = await homedStore();

    useRobotStore.getState().setToolLocked(true);
    expect(useRobotStore.getState().lockedRotation).not.toBeNull();

    const p = ForwardKinematics.position(HOME_POSE_DEG);
    await useRobotStore.getState().moveToPosition({ ...p, z: p.z + 0.02 });

    const commanded = lastCommandedAngles(port);
    expect(tumbleDeg(HOME_POSE_DEG, commanded)).toBeLessThan(0.5);
  });

  it('lets it tip when the lock is off', async () => {
    const port = await homedStore();
    expect(useRobotStore.getState().toolLocked).toBe(false);

    const p = ForwardKinematics.position(HOME_POSE_DEG);
    await useRobotStore.getState().moveToPosition({ ...p, z: p.z + 0.02 });

    const commanded = lastCommandedAngles(port);
    expect(tumbleDeg(HOME_POSE_DEG, commanded)).toBeGreaterThan(3);
  });

  it('refuses a Cartesian move without a datum', async () => {
    const { port } = await connectStore();
    port.push('STATUS IDLE 23 0 0 0 1\n');
    await flush();

    const p = ForwardKinematics.position(HOME_POSE_DEG);
    await useRobotStore.getState().moveToPosition({ ...p, z: p.z + 0.02 });

    expect(commandsOfType(port, 'J ')).toHaveLength(0);
  });

  it('drops the held orientation when unlocked', async () => {
    await homedStore();
    useRobotStore.getState().setToolLocked(true);
    useRobotStore.getState().setToolLocked(false);
    expect(useRobotStore.getState().lockedRotation).toBeNull();
  });
});

describe('store: figures are single waypoints', () => {
  async function homedStore() {
    const { port } = await connectStore();
    port.push(STATUS_IDLE);
    port.push(`POS ${HOME_POSE_DEG.map(v => v.toFixed(2)).join(' ')}\n`);
    await flush();
    return port;
  }

  it('adds one waypoint for a whole circle', async () => {
    await homedStore();
    useRobotStore.getState().clearWaypoints();

    const added = useRobotStore.getState().addCircle({ radius: 0.03, plane: 'XY' });

    expect(added).toBe(true);
    expect(useRobotStore.getState().waypoints).toHaveLength(1);
    expect(useRobotStore.getState().waypoints[0].shape).toMatchObject({
      kind: 'circle',
      radius: 0.03,
      plane: 'XY'
    });
  });

  it('removes the whole circle in one delete', async () => {
    await homedStore();
    useRobotStore.getState().clearWaypoints();
    useRobotStore.getState().addCircle({ radius: 0.03, plane: 'XY' });

    const id = useRobotStore.getState().waypoints[0].id;
    useRobotStore.getState().removeWaypoint(id);

    expect(useRobotStore.getState().waypoints).toHaveLength(0);
  });

  it('adds nothing at all when the circle does not fit', async () => {
    await homedStore();
    useRobotStore.getState().clearWaypoints();

    const added = useRobotStore.getState().addCircle({ radius: 0.5, plane: 'XY' });

    expect(added).toBe(false);
    expect(useRobotStore.getState().waypoints).toHaveLength(0);
    expect(
      useRobotStore.getState().events.some(e => e.kind === 'error' && /out of reach/.test(e.text))
    ).toBe(true);
  });

  it('plans a circle into a path the arm can stream', async () => {
    await homedStore();
    useRobotStore.getState().clearWaypoints();
    useRobotStore.getState().addCircle({ radius: 0.03, plane: 'XY' });
    useRobotStore.getState().planTrajectory();

    const trajectory = useRobotStore.getState().trajectory;
    expect(trajectory).not.toBeNull();
    // One waypoint, but an approach segment and the arc.
    expect(trajectory!.segments.length).toBeGreaterThanOrEqual(2);
    expect(TrajectoryPlanner.flattenTrajectory(trajectory!).length).toBeGreaterThan(20);
  });
});

describe('store: a path that folds the arm into itself', () => {
  async function homedStore() {
    const { port } = await connectStore();
    port.push(STATUS_IDLE);
    port.push(`POS ${HOME_POSE_DEG.map(v => v.toFixed(2)).join(' ')}\n`);
    await flush();
    return port;
  }

  /** A waypoint the arm can reach but cannot occupy without self-collision. */
  function collidingPose(): number[] {
    const q = [...HOME_POSE_DEG];
    q[2] = 130; // J3 well past where the forearm meets the shoulder
    return q;
  }

  it('reports where a planned path goes through the arm', async () => {
    await homedStore();
    const store = useRobotStore.getState();
    store.clearWaypoints();
    store.addWaypoint({
      id: 'bad',
      position: ForwardKinematics.position(collidingPose()),
      jointAngles: collidingPose(),
      speed: 30
    });
    store.planTrajectory();

    const collision = useRobotStore.getState().trajectoryCollision;
    expect(collision).not.toBeNull();
    expect(collision!.message).toMatch(/forearm/);
  });

  it('refuses to run it', async () => {
    const port = await homedStore();
    const store = useRobotStore.getState();
    store.clearWaypoints();
    store.addWaypoint({
      id: 'bad',
      position: ForwardKinematics.position(collidingPose()),
      jointAngles: collidingPose(),
      speed: 30
    });
    store.planTrajectory();

    await useRobotStore.getState().executeTrajectory();

    expect(commandsOfType(port, 'J ')).toHaveLength(0);
    expect(
      useRobotStore.getState().events.some(e => e.kind === 'error' && /folds the arm/.test(e.text))
    ).toBe(true);
  });

  it('lets a clear path through', async () => {
    const port = await homedStore();
    const store = useRobotStore.getState();
    store.clearWaypoints();
    const safe = [...HOME_POSE_DEG];
    safe[1] = 25;
    store.addWaypoint({
      id: 'ok',
      position: ForwardKinematics.position(safe),
      jointAngles: safe,
      speed: 30
    });
    store.planTrajectory();

    expect(useRobotStore.getState().trajectoryCollision).toBeNull();

    const run = useRobotStore.getState().executeTrajectory();
    await settle(40);
    port.push(STATUS_IDLE);
    await run;

    expect(commandsOfType(port, 'J ').length).toBeGreaterThan(0);
  });

  it('forgets the verdict when the waypoints are cleared', async () => {
    await homedStore();
    const store = useRobotStore.getState();
    store.clearWaypoints();
    store.addWaypoint({
      id: 'bad',
      position: ForwardKinematics.position(collidingPose()),
      jointAngles: collidingPose(),
      speed: 30
    });
    store.planTrajectory();
    expect(useRobotStore.getState().trajectoryCollision).not.toBeNull();

    useRobotStore.getState().clearWaypoints();
    expect(useRobotStore.getState().trajectoryCollision).toBeNull();
  });
});

describe('store: motion limits at runtime', () => {
  // Acceleration is set by ear, which needs run-listen-adjust in seconds rather
  // than a re-flash per attempt.

  it('asks the firmware what its limits are rather than assuming', async () => {
    const { port } = await connectStore();
    await useRobotStore.getState().refreshAxisLimits();
    await flush();

    expect(port.written.some(l => l.trim() === 'V')).toBe(true);
  });

  it('takes the values the firmware reports back', async () => {
    const { port } = await connectStore();
    port.push('LIMIT 2 22.00 66.00 40.00 100.00\n');
    await flush();

    const limit = useRobotStore.getState().axisLimits[1];
    expect(limit).toMatchObject({ axis: 1, speed: 22, accel: 66, maxSpeed: 40, maxAccel: 100 });
  });

  it('reads back after setting, instead of trusting what it asked for', async () => {
    // The firmware clamps to its own ceiling and refuses outright while moving,
    // so what was asked for is not what is in force.
    const { port } = await connectStore();
    await useRobotStore.getState().setAxisLimit(1, 9999, 9999);
    await flush();

    const sent = port.written.map(l => l.trim());
    expect(sent.some(l => l.startsWith('V 2 '))).toBe(true);
    expect(sent.filter(l => l === 'V').length).toBeGreaterThan(0);
  });
});

// ---------------------------------------------------------------------------

describe('store: work objects', () => {
  const jigPoints = (): [
    { x: number; y: number; z: number },
    { x: number; y: number; z: number },
    { x: number; y: number; z: number }
  ] => {
    const p = ForwardKinematics.position([...HOME_POSE_DEG]);
    return [
      { x: p.x, y: p.y, z: p.z },
      { x: p.x + 0.1, y: p.y, z: p.z },
      { x: p.x, y: p.y + 0.1, z: p.z }
    ];
  };

  it('teaches a point in the active frame, not in base coordinates', async () => {
    await connectStore();
    const store = useRobotStore.getState();

    const id = store.addWorkObject('Jig');
    expect(store.teachWorkObject(id, jigPoints())).toBe(true);

    useRobotStore.getState().updateCurrentPosition();
    const base = useRobotStore.getState().currentPosition!;
    useRobotStore.getState().teachCurrentPosition('hole');

    const taught = useRobotStore.getState().waypoints.at(-1)!;
    expect(taught.frame).toBe(id);

    // The arm is at the jig's origin, so in the jig's own frame the point is
    // near zero - and emphatically not the base coordinates, which are 200 mm
    // out in X.
    expect(Math.hypot(taught.position.x, taught.position.y, taught.position.z) * 1000)
      .toBeLessThan(1);
    expect(Math.hypot(base.x, base.y, base.z) * 1000).toBeGreaterThan(100);
  });

  it('moves the taught points when the frame is re-taught', async () => {
    await connectStore();
    const store = useRobotStore.getState();
    const id = store.addWorkObject('Jig');
    store.teachWorkObject(id, jigPoints());

    useRobotStore.getState().updateCurrentPosition();
    useRobotStore.getState().teachCurrentPosition('hole');

    // Solving from the position rather than replaying recorded joint angles is
    // what lets a point follow its frame at all.
    const wpId = useRobotStore.getState().waypoints.at(-1)!.id;
    useRobotStore.getState().dropJointAngles(wpId);

    const resolvedBefore = resolveWaypoint(
      useRobotStore.getState().waypoints.at(-1)!,
      useRobotStore.getState().workObjects
    ).position;

    // Somebody shifts the jig 40 mm in Z. Re-teach the same three features.
    const [a, b, c] = jigPoints();
    const shift = (p: typeof a) => ({ ...p, z: p.z - 0.04 });
    expect(
      useRobotStore.getState().teachWorkObject(id, [shift(a), shift(b), shift(c)])
    ).toBe(true);

    const resolvedAfter = resolveWaypoint(
      useRobotStore.getState().waypoints.at(-1)!,
      useRobotStore.getState().workObjects
    ).position;

    // The waypoint was never edited, and it has moved with the fixture.
    expect((resolvedBefore.z - resolvedAfter.z) * 1000).toBeCloseTo(40, 6);
  });

  it('says so when a deleted frame leaves waypoints behind', async () => {
    await connectStore();
    const store = useRobotStore.getState();
    const id = store.addWorkObject('Jig');
    store.teachWorkObject(id, jigPoints());
    useRobotStore.getState().updateCurrentPosition();
    useRobotStore.getState().teachCurrentPosition('hole');

    useRobotStore.getState().removeWorkObject(id);

    expect(
      useRobotStore.getState().events.some(
        e => e.kind === 'error' && /still name that work object/i.test(e.text)
      )
    ).toBe(true);
    // The waypoint keeps the id, so the mistake stays visible and recoverable.
    expect(useRobotStore.getState().waypoints.at(-1)!.frame).toBe(id);
  });

  it('carries the frames a path uses into its saved file, and no others', async () => {
    await connectStore();
    const store = useRobotStore.getState();
    const used = store.addWorkObject('Used');
    store.teachWorkObject(used, jigPoints());
    useRobotStore.getState().updateCurrentPosition();
    useRobotStore.getState().teachCurrentPosition('hole');

    // A second frame nothing points at.
    useRobotStore.getState().addWorkObject('Unused');

    const saved = useRobotStore.getState().exportPath('test');
    expect(saved.workObjects?.map(o => o.id)).toEqual([used]);
  });

  it('merges imported frames rather than replacing what is here', async () => {
    await connectStore();
    const store = useRobotStore.getState();
    const mine = store.addWorkObject('Mine');

    const ok = useRobotStore.getState().importPath({
      name: 'from elsewhere',
      version: '1.0',
      createdAt: new Date().toISOString(),
      config: { ...DEFAULT_PLANNER_CONFIG },
      waypoints: [
        { id: 'w1', position: { x: 0.01, y: 0, z: 0 }, speed: 50, frame: 'theirs' }
      ],
      workObjects: [
        { id: 'theirs', name: 'Theirs', origin: { x: 0.1, y: 0, z: 0 }, rpy: { roll: 0, pitch: 0, yaw: 0 } }
      ]
    });

    expect(ok).toBe(true);
    const ids = useRobotStore.getState().workObjects.map(o => o.id);
    expect(ids).toContain(mine);
    expect(ids).toContain('theirs');
  });
});

// ---------------------------------------------------------------------------

describe('store: digital I/O', () => {
  it('reads the names, and uses them to split an IO report', async () => {
    const { port } = await connectStore();

    port.push('IONAME OUT 1 gripper\nIONAME OUT 2 out2\nIONAME IN 1 part\n');
    await flush();
    port.push('IO 1 0 1\n');
    await flush();

    const state = useRobotStore.getState();
    expect(state.ioNames.outputs).toEqual(['gripper', 'out2']);
    expect(state.ioNames.inputs).toEqual(['part']);
    // Two outputs were named, so the first two digits are outputs and the rest
    // are inputs. The line itself does not say where the split is.
    expect(state.ioState).toEqual({ outputs: [true, false], inputs: [true] });
  });

  it('sends O with a one-based index', async () => {
    const { port } = await connectStore();
    await useRobotStore.getState().setOutput(0, true);
    await flush();
    expect(port.written.join('')).toContain('O 1 1');
  });

  it('fires a waypoint output only once the arm has arrived', async () => {
    const { port } = await connectStore();
    const store = useRobotStore.getState();

    store.clearWaypoints();
    store.updatePlannerConfig({ interpolationMode: 'joint', pointsPerSecond: 4 });
    store.addWaypoint(makeWaypoint([0, 12, 55, 129, 131, 0], 'a'));
    store.addWaypoint({
      ...makeWaypoint([0, 20, 58, 129, 131, 0], 'b'),
      setOutputs: [{ index: 0, high: true }]
    });
    store.planTrajectory();

    const run = useRobotStore.getState().executeTrajectory();
    await settle(40);

    // The whole path has been accepted by the firmware, which only means it is
    // queued - the arm is still working through it. Commanding the gripper here
    // would open it somewhere over the bench.
    expect(port.written.join('')).not.toContain('O 1 1');

    port.push(STATUS_IDLE);
    await run;

    // Arrived. Now it fires.
    expect(port.written.join('')).toContain('O 1 1');
  });

  it('leaves a waypoint without outputs alone', async () => {
    const { port } = await connectStore();
    planShortPath();

    const run = useRobotStore.getState().executeTrajectory();
    await settle(40);
    port.push(STATUS_IDLE);
    await run;

    expect(port.written.join('')).not.toContain('O ');
  });
});

// ---------------------------------------------------------------------------

describe('store: jogging the tool', () => {
  /** Where the arm ends up, from the last J command it was sent. */
  const landed = (port: FakePort) => {
    const moves = commandsOfType(port, 'J ');
    const angles = moves[moves.length - 1].split(/\s+/).slice(1, 7).map(Number);
    return {
      position: ForwardKinematics.position(angles),
      rotation: ForwardKinematics.solveRad(degToRad(angles)).rotation
    };
  };

  const startClear = () => {
    // Clear of the wrist singularity, so the jog is not refused for a reason
    // that has nothing to do with jogging.
    const q = [...HOME_POSE_DEG];
    q[4] = 151;
    useRobotStore.setState({
      currentAngles: { J1: q[0], J2: q[1], J3: q[2], J4: q[3], J5: q[4], J6: q[5] }
    });
    useRobotStore.getState().updateCurrentPosition();
    return q;
  };

  it('moves the tool by the distance asked, along the world axis', async () => {
    const { port } = await connectStore();
    startClear();
    const from = useRobotStore.getState().currentPosition!;

    useRobotStore.getState().setJogFrame('base');
    await useRobotStore.getState().jogCartesian('z', 20);
    await settle(400);

    const end = landed(port);
    expect((end.position.z - from.z) * 1000).toBeCloseTo(20, 1);
    // And only along that axis.
    expect(Math.abs(end.position.x - from.x) * 1000).toBeLessThan(0.5);
    expect(Math.abs(end.position.y - from.y) * 1000).toBeLessThan(0.5);
  });

  it('holds the tool angle, which is what makes it a jog', async () => {
    const { port } = await connectStore();
    const q = startClear();
    const before = ForwardKinematics.solveRad(degToRad(q)).rotation;

    useRobotStore.getState().setJogFrame('base');
    await useRobotStore.getState().jogCartesian('z', 20);
    await settle(400);

    // Unconstrained, a 20 mm move in Z tips the tool 8 degrees.
    const w = rotationLog(multiply3(landed(port).rotation, transpose3(before)));
    expect((Math.hypot(w.x, w.y, w.z) * 180) / Math.PI).toBeLessThan(0.5);
  });

  it('follows the tool own axes when asked to', async () => {
    const { port } = await connectStore();
    const q = startClear();
    const from = useRobotStore.getState().currentPosition!;
    const R = ForwardKinematics.solveRad(degToRad(q)).rotation;

    useRobotStore.getState().setJogFrame('tool');
    await useRobotStore.getState().jogCartesian('z', 20);
    await settle(400);

    // 20 mm along the tool's own Z, expressed in base coordinates. This is a
    // different direction from world Z unless the tool happens to point up.
    const end = landed(port);
    expect((end.position.x - from.x) * 1000).toBeCloseTo(R[0][2] * 20, 0);
    expect((end.position.y - from.y) * 1000).toBeCloseTo(R[1][2] * 20, 0);
    expect((end.position.z - from.z) * 1000).toBeCloseTo(R[2][2] * 20, 0);
  });

  it('turns the tool without moving the tip', async () => {
    const { port } = await connectStore();
    const q = startClear();
    const from = useRobotStore.getState().currentPosition!;
    const before = ForwardKinematics.solveRad(degToRad(q)).rotation;

    useRobotStore.getState().setJogFrame('tool');
    await useRobotStore.getState().jogRotation('z', 10);
    await settle(100);

    const end = landed(port);
    const w = rotationLog(multiply3(end.rotation, transpose3(before)));
    expect((Math.hypot(w.x, w.y, w.z) * 180) / Math.PI).toBeCloseTo(10, 0);
    // The tip is the thing that must not move.
    expect(
      Math.hypot(end.position.x - from.x, end.position.y - from.y, end.position.z - from.z) * 1000
    ).toBeLessThan(0.5);
  });

  it('refuses to jog an arm whose position is not trusted', async () => {
    const { port } = await connectStore();
    startClear();

    port.push('STATUS IDLE 23 0 0 30 1\n');
    await flush();

    const before = commandsOfType(port, 'J ').length;
    await useRobotStore.getState().jogCartesian('z', 20);
    await settle(50);

    expect(commandsOfType(port, 'J ')).toHaveLength(before);
  });
});
