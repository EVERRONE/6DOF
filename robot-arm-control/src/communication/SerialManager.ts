import { FirmwareStatus, MoveAck, SerialMessage } from './types';
import { JointAngles, EndstopState, ConnectionStatus } from '../types/robot';
import {
  SerialLike,
  SerialPortEventLike,
  SerialPortLike,
  SerialReaderLike,
  SerialWriterLike,
  getBrowserSerial
} from './webSerial';

/** How long to wait for a move to be acknowledged before giving up. */
const MOVE_ACK_TIMEOUT_MS = 2000;

/**
 * How long the link may go quiet before it is presumed dead.
 *
 * The firmware pushes a report every REPORT_INTERVAL_MS (50 ms) and nothing in
 * it blocks, so silence for this long means the cable, the port or the firmware
 * has stopped - even when no error was raised. Without this the app sat there
 * showing "connected" and a frozen position forever.
 */
const LINK_TIMEOUT_MS = 2000;

/** Watchdog poll interval. */
const WATCHDOG_INTERVAL_MS = 500;

/** Reconnect backoff, in milliseconds, one entry per attempt. */
const RECONNECT_BACKOFF_MS = [400, 800, 1500, 3000, 5000, 8000];

/**
 * Longest run of bytes tolerated without a newline.
 *
 * A confused device can emit binary noise indefinitely; without a cap the receive
 * buffer would grow until the tab died.
 */
const MAX_LINE_BYTES = 512;

export interface SerialManagerOptions {
  /** Injected for tests; defaults to navigator.serial. */
  serial?: SerialLike | null;
  /** Reopen the port by itself after an unexpected loss. Default true. */
  autoReconnect?: boolean;
}

type StateListener = (state: ConnectionStatus, detail?: string) => void;

/**
 * Web Serial transport for the arm, including the connection lifecycle.
 *
 * Move commands are acknowledged, and the acknowledgement carries the free space
 * left in the firmware's motion queue, which is what lets the host keep the queue
 * full while streaming a trajectory.
 *
 * Losing the link is treated as a normal event rather than an exception: unplug
 * the USB cable mid-move and the manager notices (by error, by end of stream, or
 * by the watchdog), fails everything in flight, reports the loss, and tries to
 * reopen the port on a backoff. Previously the reader loop swallowed the error and
 * the UI went on claiming to be connected.
 */
export class SerialManager {
  private readonly serial: SerialLike | null;
  private readonly autoReconnect: boolean;

  private port: SerialPortLike | null = null;
  private reader: SerialReaderLike | null = null;
  private writer: SerialWriterLike | null = null;

  private state: ConnectionStatus = ConnectionStatus.DISCONNECTED;

  /**
   * How many of the digits on an IO line are outputs.
   *
   * The line itself does not say - it is `IO <o1..oN> <i1..iN>` - so the split
   * is learned from the IONAME replies. Zero until those arrive, which makes an
   * IO line before them read as all inputs rather than as a confident wrong
   * answer about which pins are being driven.
   */
  private outputCount = 0;

  private messageListeners: ((msg: SerialMessage) => void)[] = [];
  private stateListeners: StateListener[] = [];

  /** Resolvers for move commands awaiting acknowledgement, oldest first. */
  private pendingMoveAcks: {
    resolve: (ack: MoveAck) => void;
    reject: (error: Error) => void;
    timer: ReturnType<typeof setTimeout>;
  }[] = [];

  private lastStatus: FirmwareStatus | null = null;
  private lastLineAt = 0;

  private watchdog: ReturnType<typeof setInterval> | null = null;
  private reconnectTimer: ReturnType<typeof setTimeout> | null = null;
  private reconnectAttempt = 0;

  /** True while an intentional teardown is in progress, so it is not a "loss". */
  private closing = false;
  private readLoopId = 0;

  private readonly onPortDisconnected = (event: SerialPortEventLike) => {
    // Fired by the browser when the device is physically removed.
    if (this.port && event.target && event.target !== this.port) return;
    this.handleLinkLost('Device disconnected');
  };

  constructor(options: SerialManagerOptions = {}) {
    this.serial = options.serial !== undefined ? options.serial : getBrowserSerial();
    this.autoReconnect = options.autoReconnect ?? true;

    this.serial?.addEventListener('disconnect', this.onPortDisconnected);
  }

  static isSupported(): boolean {
    return getBrowserSerial() !== null;
  }

  get connectionState(): ConnectionStatus {
    return this.state;
  }

  get status(): FirmwareStatus | null {
    return this.lastStatus;
  }

  get isConnected(): boolean {
    return this.state === ConnectionStatus.CONNECTED;
  }

  // -------------------------------------------------------------------------
  // Lifecycle
  // -------------------------------------------------------------------------

  /** Ask the user for a port and open it. Needs a user gesture. */
  async connect(): Promise<void> {
    if (!this.serial) {
      throw new Error('Web Serial is not available in this browser');
    }

    this.closing = false;
    this.cancelScheduledReconnect();
    this.setState(ConnectionStatus.CONNECTING);

    try {
      const port = await this.serial.requestPort();
      await this.openPort(port);
    } catch (error) {
      this.setState(
        ConnectionStatus.ERROR,
        error instanceof Error ? error.message : 'Could not open the port'
      );
      throw error;
    }
  }

  /** Close the link on purpose. Cancels auto-reconnect. */
  async disconnect(): Promise<void> {
    this.closing = true;
    this.cancelScheduledReconnect();
    this.reconnectAttempt = 0;

    await this.teardown();
    this.rejectPendingAcks(new Error('Disconnected'));
    this.lastStatus = null;
    this.setState(ConnectionStatus.DISCONNECTED);
  }

  /** Release listeners and timers. Call when discarding the manager. */
  dispose(): void {
    this.serial?.removeEventListener('disconnect', this.onPortDisconnected);
    this.closing = true;
    this.cancelScheduledReconnect();
    this.stopWatchdog();
    this.messageListeners = [];
    this.stateListeners = [];
  }

  onMessage(callback: (msg: SerialMessage) => void): void {
    this.messageListeners.push(callback);
  }

  onStateChange(callback: StateListener): void {
    this.stateListeners.push(callback);
  }

  private async openPort(port: SerialPortLike): Promise<void> {
    await port.open({ baudRate: 115200 });

    if (!port.readable || !port.writable) {
      await port.close().catch(() => undefined);
      throw new Error('Port opened without readable or writable streams');
    }

    this.port = port;
    this.reader = port.readable.getReader();
    this.writer = port.writable.getWriter();
    this.reconnectAttempt = 0;
    this.lastLineAt = Date.now();

    this.setState(ConnectionStatus.CONNECTED);
    this.startWatchdog();
    void this.readLoop(++this.readLoopId);

    // The firmware may have restarted while the link was down, so never carry
    // forward what we thought its state was - ask.
    this.lastStatus = null;
    await this.sendCommand('Q').catch(() => undefined);
  }

  /** Tear down reader, writer and port without changing the reported state. */
  private async teardown(): Promise<void> {
    this.readLoopId++;
    this.stopWatchdog();

    const { reader, writer, port } = this;
    this.reader = null;
    this.writer = null;
    this.port = null;

    if (reader) {
      await reader.cancel().catch(() => undefined);
      try {
        reader.releaseLock();
      } catch {
        // Already released, or the stream errored.
      }
    }

    if (writer) {
      try {
        writer.releaseLock();
      } catch {
        // As above.
      }
    }

    if (port) {
      await port.close().catch(() => undefined);
    }
  }

  /**
   * The link went away without being asked to. Fail everything in flight, report
   * it, and start trying to get it back.
   */
  private handleLinkLost(reason: string): void {
    if (this.closing) return;
    if (this.state === ConnectionStatus.DISCONNECTED) return;

    void this.teardown();

    this.rejectPendingAcks(new Error(reason));
    this.lastStatus = null;

    if (this.autoReconnect) {
      this.setState(ConnectionStatus.RECONNECTING, reason);
      this.scheduleReconnect();
    } else {
      this.setState(ConnectionStatus.ERROR, reason);
    }
  }

  private scheduleReconnect(): void {
    this.cancelScheduledReconnect();

    if (this.reconnectAttempt >= RECONNECT_BACKOFF_MS.length) {
      this.setState(
        ConnectionStatus.ERROR,
        'Could not reopen the port; reconnect by hand'
      );
      return;
    }

    const delay = RECONNECT_BACKOFF_MS[this.reconnectAttempt];
    this.reconnectAttempt++;

    this.reconnectTimer = setTimeout(() => {
      this.reconnectTimer = null;
      void this.attemptReconnect();
    }, delay);
  }

  private cancelScheduledReconnect(): void {
    if (this.reconnectTimer !== null) {
      clearTimeout(this.reconnectTimer);
      this.reconnectTimer = null;
    }
  }

  /**
   * Try to reopen a port the user has already granted.
   *
   * getPorts() returns those, so this needs no new user gesture - replug the
   * cable and the app comes back on its own.
   */
  private async attemptReconnect(): Promise<void> {
    if (this.closing || !this.serial) return;

    try {
      const ports = await this.serial.getPorts();
      if (ports.length === 0) {
        this.scheduleReconnect();
        return;
      }

      await this.openPort(ports[0]);
    } catch {
      this.scheduleReconnect();
    }
  }

  // -------------------------------------------------------------------------
  // Watchdog
  // -------------------------------------------------------------------------

  private startWatchdog(): void {
    this.stopWatchdog();
    this.watchdog = setInterval(() => {
      if (this.state !== ConnectionStatus.CONNECTED) return;
      if (Date.now() - this.lastLineAt <= LINK_TIMEOUT_MS) return;
      this.handleLinkLost('No response from the controller');
    }, WATCHDOG_INTERVAL_MS);
  }

  private stopWatchdog(): void {
    if (this.watchdog !== null) {
      clearInterval(this.watchdog);
      this.watchdog = null;
    }
  }

  // -------------------------------------------------------------------------
  // Commands
  // -------------------------------------------------------------------------

  async sendCommand(cmd: string): Promise<void> {
    const writer = this.writer;
    if (!writer) throw new Error('Not connected');

    try {
      await writer.write(new TextEncoder().encode(cmd + '\n'));
    } catch (error) {
      // A failed write means the link is gone, whatever the reader thinks.
      this.handleLinkLost('Write failed');
      throw error instanceof Error ? error : new Error('Write failed');
    }
  }

  /**
   * Queue a joint move and wait for the firmware to acknowledge it.
   *
   * Resolves with `accepted: false` when the queue was full. That is normal
   * back-pressure while streaming, not an error: wait for space and resend.
   */
  async moveToAngles(angles: JointAngles, speed: number): Promise<MoveAck> {
    const values = [angles.J1, angles.J2, angles.J3, angles.J4, angles.J5, angles.J6]
      .map(v => v.toFixed(3))
      .join(' ');

    const ack = this.awaitMoveAck();
    try {
      await this.sendCommand(`J ${values} ${speed.toFixed(2)}`);
    } catch (error) {
      // sendCommand already failed the pending acks; keep the rejection.
      await ack.catch(() => undefined);
      throw error;
    }
    return ack;
  }

  async homeJoints(joints: string): Promise<void> {
    await this.sendCommand(`H ${joints}`);
  }

  async queryStatus(): Promise<void> {
    await this.sendCommand('Q');
  }

  async enableMotors(enable: boolean): Promise<void> {
    await this.sendCommand(`E ${enable ? 1 : 0}`);
  }

  /** Immediate stop. May lose steps, so the arm needs re-homing afterwards. */
  async emergencyStop(): Promise<void> {
    this.rejectPendingAcks(new Error('Emergency stop'));
    await this.sendCommand('S');
  }

  /** Graceful stop: decelerate, drop the queue, keep the position. */
  async abort(): Promise<void> {
    this.rejectPendingAcks(new Error('Aborted'));
    await this.sendCommand('A');
  }

  // -------------------------------------------------------------------------
  // Move acknowledgements
  // -------------------------------------------------------------------------

  private awaitMoveAck(): Promise<MoveAck> {
    return new Promise<MoveAck>((resolve, reject) => {
      const timer = setTimeout(() => {
        this.pendingMoveAcks = this.pendingMoveAcks.filter(p => p.timer !== timer);
        reject(new Error('Timed out waiting for the firmware to acknowledge a move'));
      }, MOVE_ACK_TIMEOUT_MS);

      this.pendingMoveAcks.push({ resolve, reject, timer });
    });
  }

  private resolveMoveAck(ack: MoveAck): void {
    const pending = this.pendingMoveAcks.shift();
    if (!pending) return;
    clearTimeout(pending.timer);
    pending.resolve(ack);
  }

  private rejectPendingAcks(error: Error): void {
    const pending = this.pendingMoveAcks;
    this.pendingMoveAcks = [];
    for (const p of pending) {
      clearTimeout(p.timer);
      p.reject(error);
    }
  }

  // -------------------------------------------------------------------------
  // Receive
  // -------------------------------------------------------------------------

  private async readLoop(loopId: number): Promise<void> {
    const decoder = new TextDecoder();
    let buffer = '';

    while (loopId === this.readLoopId) {
      const reader = this.reader;
      if (!reader) break;

      let chunk: { value?: Uint8Array; done: boolean };
      try {
        chunk = await reader.read();
      } catch (error) {
        if (loopId === this.readLoopId) {
          this.handleLinkLost(
            error instanceof Error ? error.message : 'Serial read failed'
          );
        }
        return;
      }

      if (loopId !== this.readLoopId) return;

      if (chunk.done) {
        // The device closed the stream. Intentional teardown bumps readLoopId
        // first, so reaching here means it happened on its own.
        this.handleLinkLost('Connection closed by the device');
        return;
      }

      if (!chunk.value) continue;

      buffer += decoder.decode(chunk.value, { stream: true });

      let newline = buffer.indexOf('\n');
      while (newline >= 0) {
        const line = buffer.slice(0, newline).trim();
        buffer = buffer.slice(newline + 1);
        if (line.length > 0) this.processLine(line);
        newline = buffer.indexOf('\n');
      }

      // Nothing but noise: drop it rather than buffer without limit.
      if (buffer.length > MAX_LINE_BYTES) {
        console.warn(
          `Discarding ${buffer.length} bytes of serial data with no line ending`
        );
        buffer = '';
      }
    }
  }

  private processLine(line: string): void {
    // Any well-formed line proves the link is alive.
    this.lastLineAt = Date.now();

    const parts = line.split(/\s+/);
    const timestamp = Date.now();

    switch (parts[0]) {
      case 'POS': {
        if (parts.length !== 7) return;
        const values = parts.slice(1).map(parseFloat);
        if (values.some(Number.isNaN)) return;
        this.emit({ type: 'POS', data: toJointAngles(values), timestamp });
        return;
      }

      case 'ENDSTOP': {
        if (parts.length !== 7) return;
        this.emit({
          type: 'ENDSTOP',
          data: toEndstopState(parts.slice(1).map(v => v === '1')),
          timestamp
        });
        return;
      }

      case 'LIMIT': {
        // LIMIT <axis> <speed> <accel> <maxSpeed> <maxAccel>
        if (parts.length === 6) {
          this.emit({
            type: 'LIMIT',
            data: {
              axis: parseInt(parts[1], 10) - 1,
              speed: parseFloat(parts[2]),
              accel: parseFloat(parts[3]),
              maxSpeed: parseFloat(parts[4]),
              maxAccel: parseFloat(parts[5])
            },
            timestamp: Date.now()
          });
        }
        return;
      }

      case 'IO': {
        // IO <o1..oN> <i1..iN>. The split between them is not in the line, so it
        // comes from the names the firmware reported - which is why the host asks
        // for those on connect rather than assuming a count.
        const flags = parts.slice(1).map(v => v === '1');
        if (flags.length === 0) return;
        this.emit({
          type: 'IO',
          data: {
            outputs: flags.slice(0, this.outputCount),
            inputs: flags.slice(this.outputCount)
          },
          timestamp
        });
        return;
      }

      case 'IONAME': {
        // IONAME OUT|IN <n> <name>
        if (parts.length < 4) return;
        const direction = parts[1].toUpperCase() === 'OUT' ? 'out' : 'in';
        const index = parseInt(parts[2], 10) - 1;
        if (!Number.isFinite(index) || index < 0) return;

        // Counting them here is what lets the IO line above be split correctly.
        if (direction === 'out') this.outputCount = Math.max(this.outputCount, index + 1);

        this.emit({
          type: 'IONAME',
          data: { direction, index, name: parts.slice(3).join(' ') },
          timestamp
        });
        return;
      }

      case 'STATUS': {
        // STATUS <state> <queueFree> <moving> <positionTrusted> <homedMask> <enabled>
        if (parts.length < 6) return;

        const queueFree = parseInt(parts[2], 10);
        if (Number.isNaN(queueFree)) return;

        const homedMask = parseInt(parts[5], 10);
        const status: FirmwareStatus = {
          state: parts[1] as FirmwareStatus['state'],
          queueFree,
          moving: parts[3] === '1',
          positionTrusted: parts[4] === '1',
          homed: Array.from({ length: 6 }, (_, i) =>
            Number.isNaN(homedMask) ? false : (homedMask & (1 << i)) !== 0
          ),
          // Absent from older firmware, which never reported it.
          enabled: parts[6] === '1'
        };

        this.lastStatus = status;
        this.emit({ type: 'STATUS', data: status, timestamp });
        return;
      }

      case 'BUSY': {
        const queueFree = parseInt(parts[1] ?? '0', 10);
        const ack: MoveAck = {
          accepted: false,
          queueFree: Number.isNaN(queueFree) ? 0 : queueFree
        };
        this.resolveMoveAck(ack);
        this.emit({ type: 'MOVE_ACK', data: ack, timestamp });
        return;
      }

      case 'OK': {
        // "OK J <free>" acknowledges a queued move; any other OK is informational.
        if (parts[1] === 'J') {
          const queueFree = parseInt(parts[2] ?? '0', 10);
          const ack: MoveAck = {
            accepted: true,
            queueFree: Number.isNaN(queueFree) ? 0 : queueFree
          };
          this.resolveMoveAck(ack);
          this.emit({ type: 'MOVE_ACK', data: ack, timestamp });
          return;
        }
        this.emit({ type: 'OK', data: parts.slice(1).join(' '), timestamp });
        return;
      }

      case 'ERROR': {
        const message = parts.slice(1).join(' ');
        // A rejected move must not leave its sender waiting for an ack that will
        // never come.
        this.rejectPendingAcks(new Error(message));
        this.emit({ type: 'ERROR', data: message, timestamp });
        return;
      }

      case 'HOMED': {
        const joint = parts[1] !== undefined ? parseInt(parts[1], 10) : NaN;
        this.emit({
          type: 'HOMED',
          data: Number.isNaN(joint) ? null : joint,
          timestamp
        });
        return;
      }

      default:
        return;
    }
  }

  private emit(message: SerialMessage): void {
    for (const callback of this.messageListeners) {
      callback(message);
    }
  }

  private setState(state: ConnectionStatus, detail?: string): void {
    if (this.state === state) return;
    this.state = state;
    for (const listener of this.stateListeners) {
      listener(state, detail);
    }
  }
}

function toJointAngles(values: number[]): JointAngles {
  return {
    J1: values[0],
    J2: values[1],
    J3: values[2],
    J4: values[3],
    J5: values[4],
    J6: values[5]
  };
}

function toEndstopState(values: boolean[]): EndstopState {
  return {
    J1: values[0],
    J2: values[1],
    J3: values[2],
    J4: values[3],
    J5: values[4],
    J6: values[5]
  };
}
