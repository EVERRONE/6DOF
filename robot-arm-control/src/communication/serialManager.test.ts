// Tests for the serial transport and its connection lifecycle.
//
// SerialManager takes the Serial interface as a constructor argument, so these
// drive it against a fake port: bytes can be pushed in a chunk at a time, the
// cable can be "unplugged" mid-command, and writes can be made to fail. None of
// that is reachable with a real USB device, and this is the layer where a fault
// shows up as "the arm sometimes just stops responding".

import { SerialManager } from './SerialManager';
import { ConnectionStatus } from '../types/robot';
import { SerialMessage } from './types';
import { FakePort, FakeSerial } from '../testUtils/fakeSerial';

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

/** Let queued microtasks and timers settle. */
const flush = () => new Promise<void>(resolve => setTimeout(resolve, 0));

async function connected(options: { autoReconnect?: boolean } = {}) {
  const port = new FakePort();
  const serial = new FakeSerial(port);
  const manager = new SerialManager({ serial, autoReconnect: options.autoReconnect });

  const messages: SerialMessage[] = [];
  const states: ConnectionStatus[] = [];
  manager.onMessage(m => messages.push(m));
  manager.onStateChange(s => states.push(s));

  await manager.connect();
  await flush();

  return { manager, port, serial, messages, states };
}

const STATUS_LINE = 'STATUS IDLE 23 0 1 30\n';

// ---------------------------------------------------------------------------

describe('serial manager: connecting', () => {
  it('opens the port and reports connected', async () => {
    const { manager, port, states } = await connected();

    expect(port.opened).toBe(true);
    expect(manager.connectionState).toBe(ConnectionStatus.CONNECTED);
    expect(states).toContain(ConnectionStatus.CONNECTING);
    expect(states).toContain(ConnectionStatus.CONNECTED);

    manager.dispose();
  });

  it('asks the firmware for its state on connect, rather than assuming', async () => {
    const { manager, port } = await connected();

    // The controller may have restarted while the link was down.
    expect(port.written.join('')).toContain('Q');
    expect(manager.status).toBeNull();

    manager.dispose();
  });

  it('reports an error when no port is chosen', async () => {
    const serial = new FakeSerial(); // no ports
    const manager = new SerialManager({ serial });
    const states: ConnectionStatus[] = [];
    manager.onStateChange(s => states.push(s));

    await expect(manager.connect()).rejects.toThrow();
    expect(manager.connectionState).toBe(ConnectionStatus.ERROR);
    expect(states).toEqual([ConnectionStatus.CONNECTING, ConnectionStatus.ERROR]);

    manager.dispose();
  });

  it('refuses to connect where Web Serial is unavailable', async () => {
    const manager = new SerialManager({ serial: null });
    await expect(manager.connect()).rejects.toThrow(/not available/i);
    manager.dispose();
  });
});

describe('serial manager: parsing', () => {
  it('reassembles lines split across chunks', async () => {
    const { manager, port, messages } = await connected();

    // A report arriving in three pieces, with a partial line left over.
    port.push('POS 1.00 2.00 3.0');
    port.push('0 4.00 5.00 6.00\nENDST');
    await flush();

    expect(messages.filter(m => m.type === 'POS')).toHaveLength(1);
    const pos = messages.find(m => m.type === 'POS');
    expect(pos?.data).toEqual({ J1: 1, J2: 2, J3: 3, J4: 4, J5: 5, J6: 6 });

    port.push('OP 0 1 0 0 0 0\n');
    await flush();

    const endstop = messages.find(m => m.type === 'ENDSTOP');
    expect(endstop?.data).toEqual({
      J1: false, J2: true, J3: false, J4: false, J5: false, J6: false
    });

    manager.dispose();
  });

  it('handles several lines in one chunk', async () => {
    const { manager, port, messages } = await connected();

    port.push('POS 0 0 0 0 0 0\nENDSTOP 0 0 0 0 0 0\n' + STATUS_LINE);
    await flush();

    expect(messages.filter(m => m.type === 'POS')).toHaveLength(1);
    expect(messages.filter(m => m.type === 'ENDSTOP')).toHaveLength(1);
    expect(messages.filter(m => m.type === 'STATUS')).toHaveLength(1);

    manager.dispose();
  });

  it('parses a status line into queue space, trust and homed flags', async () => {
    const { manager, port, messages } = await connected();

    port.push('STATUS MOVING 17 1 1 30\n');
    await flush();

    const status = messages.find(m => m.type === 'STATUS');
    expect(status?.data).toEqual({
      state: 'MOVING',
      queueFree: 17,
      moving: true,
      positionTrusted: true,
      // mask 30 = 0b011110 = J2..J5
      homed: [false, true, true, true, true, false]
    });
    expect(manager.status?.queueFree).toBe(17);

    manager.dispose();
  });

  it('ignores malformed lines instead of emitting rubbish', async () => {
    const { manager, port, messages } = await connected();

    port.push('POS 1 2 3\n');           // too few fields
    port.push('POS a b c d e f\n');     // not numbers
    port.push('STATUS IDLE\n');         // truncated
    port.push('GARBAGE ~~~\n');         // unknown
    await flush();

    expect(messages.filter(m => m.type === 'POS')).toHaveLength(0);
    expect(messages.filter(m => m.type === 'STATUS')).toHaveLength(0);

    manager.dispose();
  });

  it('discards an endless run of bytes with no line ending', async () => {
    const { manager, port, messages } = await connected();

    // A confused device emitting binary noise must not grow the buffer forever.
    for (let k = 0; k < 20; k++) port.push('x'.repeat(100));
    await flush();

    // Still working afterwards.
    port.push('\n' + STATUS_LINE);
    await flush();
    expect(messages.filter(m => m.type === 'STATUS')).toHaveLength(1);

    manager.dispose();
  });
});

describe('serial manager: move acknowledgements', () => {
  it('resolves a move once the firmware accepts it, with the queue space', async () => {
    const { manager, port } = await connected();

    const pending = manager.moveToAngles(
      { J1: 0, J2: 10, J3: 20, J4: 30, J5: 40, J6: 50 },
      25
    );
    await flush();

    expect(port.lastCommand()).toBe('J 0.000 10.000 20.000 30.000 40.000 50.000 25.00');

    port.push('OK J 22\n');
    const ack = await pending;

    expect(ack).toEqual({ accepted: true, queueFree: 22 });

    manager.dispose();
  });

  it('reports a full queue as back-pressure, not a failure', async () => {
    const { manager, port } = await connected();

    const pending = manager.moveToAngles(
      { J1: 0, J2: 0, J3: 0, J4: 0, J5: 0, J6: 0 },
      25
    );
    await flush();
    port.push('BUSY 0\n');

    await expect(pending).resolves.toEqual({ accepted: false, queueFree: 0 });

    manager.dispose();
  });

  it('acknowledges concurrent moves in the order they were sent', async () => {
    const { manager, port } = await connected();

    const angles = { J1: 0, J2: 0, J3: 0, J4: 0, J5: 0, J6: 0 };
    const first = manager.moveToAngles(angles, 10);
    const second = manager.moveToAngles(angles, 10);
    const third = manager.moveToAngles(angles, 10);
    await flush();

    port.push('OK J 22\nBUSY 0\nOK J 21\n');

    expect(await first).toEqual({ accepted: true, queueFree: 22 });
    expect(await second).toEqual({ accepted: false, queueFree: 0 });
    expect(await third).toEqual({ accepted: true, queueFree: 21 });

    manager.dispose();
  });

  it('fails a move the firmware rejects, rather than leaving it hanging', async () => {
    const { manager, port } = await connected();

    const pending = manager.moveToAngles(
      { J1: 0, J2: 0, J3: 0, J4: 0, J5: 0, J6: 0 },
      25
    );
    await flush();
    port.push('ERROR Motors disabled\n');

    await expect(pending).rejects.toThrow(/motors disabled/i);

    manager.dispose();
  });

  it('fails a move when the write itself fails', async () => {
    const { manager, port } = await connected();
    port.failWrites = true;

    await expect(
      manager.moveToAngles({ J1: 0, J2: 0, J3: 0, J4: 0, J5: 0, J6: 0 }, 25)
    ).rejects.toThrow();

    // A failed write means the link is gone, whatever the reader thinks.
    expect(manager.connectionState).not.toBe(ConnectionStatus.CONNECTED);

    manager.dispose();
  });
});

describe('serial manager: losing the link', () => {
  it('notices the device closing the stream', async () => {
    const { manager, port, states } = await connected({ autoReconnect: false });

    port.endStream();
    await flush();

    expect(manager.connectionState).toBe(ConnectionStatus.ERROR);
    expect(states).toContain(ConnectionStatus.ERROR);

    manager.dispose();
  });

  it('notices a read that throws', async () => {
    const { manager, port } = await connected({ autoReconnect: false });

    port.push('POS 0 0 0 0 0 0\n');
    await flush();
    port.failStream();
    port.push('anything to wake the reader\n');
    await flush();

    expect(manager.connectionState).not.toBe(ConnectionStatus.CONNECTED);

    manager.dispose();
  });

  it('notices the browser disconnect event', async () => {
    const { manager, serial, port } = await connected({ autoReconnect: false });

    serial.emitDisconnect(port);
    await flush();

    expect(manager.connectionState).toBe(ConnectionStatus.ERROR);

    manager.dispose();
  });

  it('fails every move in flight when the link drops', async () => {
    const { manager, port } = await connected({ autoReconnect: false });

    const angles = { J1: 0, J2: 0, J3: 0, J4: 0, J5: 0, J6: 0 };

    // Assert first, so a handler is attached before the rejection happens.
    // Neither may sit unresolved: the trajectory sender awaits these.
    const firstRejects = expect(manager.moveToAngles(angles, 10)).rejects.toThrow();
    const secondRejects = expect(manager.moveToAngles(angles, 10)).rejects.toThrow();
    await flush();

    port.endStream();
    await flush();

    await firstRejects;
    await secondRejects;

    manager.dispose();
  });

  it('drops the cached firmware status, so nothing stale is trusted', async () => {
    const { manager, port } = await connected({ autoReconnect: false });

    port.push(STATUS_LINE);
    await flush();
    expect(manager.status).not.toBeNull();

    port.endStream();
    await flush();
    expect(manager.status).toBeNull();

    manager.dispose();
  });

  it('gives up on a link that goes quiet, even with no error', async () => {
    jest.useFakeTimers();
    try {
      const port = new FakePort();
      const serial = new FakeSerial(port);
      const manager = new SerialManager({ serial, autoReconnect: false });

      await manager.connect();
      expect(manager.connectionState).toBe(ConnectionStatus.CONNECTED);

      // The firmware reports every 50 ms, so silence is a dead link even when
      // no error is raised and the port still looks open.
      jest.advanceTimersByTime(5000);

      expect(manager.connectionState).not.toBe(ConnectionStatus.CONNECTED);
      manager.dispose();
    } finally {
      jest.useRealTimers();
    }
  });

  it('keeps the link alive while reports keep arriving', async () => {
    jest.useFakeTimers();
    try {
      const port = new FakePort();
      const serial = new FakeSerial(port);
      const manager = new SerialManager({ serial, autoReconnect: false });
      await manager.connect();

      for (let k = 0; k < 20; k++) {
        port.push(STATUS_LINE);
        await Promise.resolve();
        await Promise.resolve();
        jest.advanceTimersByTime(200);
      }

      expect(manager.connectionState).toBe(ConnectionStatus.CONNECTED);
      manager.dispose();
    } finally {
      jest.useRealTimers();
    }
  });
});

describe('serial manager: reconnecting', () => {
  it('reopens a granted port by itself after a drop', async () => {
    jest.useFakeTimers();
    try {
      const port = new FakePort();
      const serial = new FakeSerial(port);
      const manager = new SerialManager({ serial, autoReconnect: true });

      await manager.connect();
      const requestsBefore = serial.requestCount;

      port.failStream();
      port.push('wake\n');
      await Promise.resolve();
      await Promise.resolve();
      expect(manager.connectionState).toBe(ConnectionStatus.RECONNECTING);

      // getPorts() returns ports the user already granted, so coming back needs
      // no new user gesture.
      jest.advanceTimersByTime(500);
      for (let k = 0; k < 20; k++) await Promise.resolve();

      expect(manager.connectionState).toBe(ConnectionStatus.CONNECTED);
      expect(serial.requestCount).toBe(requestsBefore);

      manager.dispose();
    } finally {
      jest.useRealTimers();
    }
  });

  it('keeps retrying while the cable is still out, then gives up', async () => {
    jest.useFakeTimers();
    try {
      const port = new FakePort();
      const serial = new FakeSerial(port);
      const manager = new SerialManager({ serial, autoReconnect: true });

      await manager.connect();

      serial.granted = []; // cable still unplugged
      port.endStream();
      await Promise.resolve();
      await Promise.resolve();
      expect(manager.connectionState).toBe(ConnectionStatus.RECONNECTING);

      // Exhaust the backoff schedule.
      for (let k = 0; k < 12; k++) {
        jest.advanceTimersByTime(10000);
        for (let n = 0; n < 20; n++) await Promise.resolve();
      }

      // It stops rather than retrying forever, and says so.
      expect(manager.connectionState).toBe(ConnectionStatus.ERROR);

      manager.dispose();
    } finally {
      jest.useRealTimers();
    }
  });

  it('stops trying when the user disconnects', async () => {
    jest.useFakeTimers();
    try {
      const port = new FakePort();
      const serial = new FakeSerial(port);
      const manager = new SerialManager({ serial, autoReconnect: true });

      await manager.connect();
      port.endStream();
      await Promise.resolve();
      await Promise.resolve();
      expect(manager.connectionState).toBe(ConnectionStatus.RECONNECTING);

      await manager.disconnect();
      expect(manager.connectionState).toBe(ConnectionStatus.DISCONNECTED);

      jest.advanceTimersByTime(60000);
      for (let n = 0; n < 20; n++) await Promise.resolve();

      // No reconnect happened behind the user's back.
      expect(manager.connectionState).toBe(ConnectionStatus.DISCONNECTED);

      manager.dispose();
    } finally {
      jest.useRealTimers();
    }
  });
});

describe('serial manager: disconnecting on purpose', () => {
  it('closes the port and reports disconnected', async () => {
    const { manager, port } = await connected();

    await manager.disconnect();

    expect(port.closed).toBe(true);
    expect(manager.connectionState).toBe(ConnectionStatus.DISCONNECTED);
    expect(manager.status).toBeNull();

    manager.dispose();
  });

  it('is not mistaken for a link loss', async () => {
    const { manager, states } = await connected({ autoReconnect: true });

    await manager.disconnect();
    await flush();

    expect(states).not.toContain(ConnectionStatus.RECONNECTING);
    expect(manager.connectionState).toBe(ConnectionStatus.DISCONNECTED);

    manager.dispose();
  });

  it('refuses commands once disconnected', async () => {
    const { manager } = await connected();
    await manager.disconnect();

    await expect(manager.sendCommand('Q')).rejects.toThrow(/not connected/i);

    manager.dispose();
  });
});
