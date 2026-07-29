import { FirmwareStatus, MoveAck, SerialMessage } from './types';
import { JointAngles, EndstopState } from '../types/robot';

/** Milliseconds to wait for a move to be acknowledged before giving up. */
const MOVE_ACK_TIMEOUT_MS = 2000;

/**
 * Web Serial transport for the arm.
 *
 * Move commands are acknowledged, and the acknowledgement carries the free space
 * left in the firmware's motion queue. That is what lets the host keep the queue
 * full while streaming a trajectory: the firmware plans ahead across the moves it
 * is holding, so depth is what turns a series of points into one continuous
 * motion instead of a stop at every point.
 */
export class SerialManager {
  private port: SerialPort | null = null;
  private reader: ReadableStreamDefaultReader | null = null;
  private writer: WritableStreamDefaultWriter | null = null;

  private messageCallbacks: ((msg: SerialMessage) => void)[] = [];
  private isReading = false;

  /** Resolvers for move commands awaiting their acknowledgement, in order. */
  private pendingMoveAcks: {
    resolve: (ack: MoveAck) => void;
    reject: (error: Error) => void;
    timer: ReturnType<typeof setTimeout>;
  }[] = [];

  private lastStatus: FirmwareStatus | null = null;

  static isSupported(): boolean {
    return 'serial' in navigator;
  }

  async connect(): Promise<void> {
    this.port = await navigator.serial.requestPort();
    await this.port.open({ baudRate: 115200 });

    this.reader = this.port.readable!.getReader();
    this.writer = this.port.writable!.getWriter();

    this.startReading();
  }

  async disconnect(): Promise<void> {
    this.isReading = false;
    this.rejectPendingAcks(new Error('Disconnected'));

    if (this.reader) {
      await this.reader.cancel().catch(() => undefined);
      this.reader.releaseLock();
      this.reader = null;
    }

    if (this.writer) {
      this.writer.releaseLock();
      this.writer = null;
    }

    if (this.port) {
      await this.port.close().catch(() => undefined);
      this.port = null;
    }
  }

  get status(): FirmwareStatus | null {
    return this.lastStatus;
  }

  async sendCommand(cmd: string): Promise<void> {
    if (!this.writer) throw new Error('Not connected');
    await this.writer.write(new TextEncoder().encode(cmd + '\n'));
  }

  /**
   * Queue a joint move and wait for the firmware to acknowledge it.
   *
   * Resolves with `accepted: false` when the queue was full, which is normal
   * back-pressure while streaming, not an error - the caller should wait for
   * space and send the same point again.
   */
  async moveToAngles(angles: JointAngles, speed: number): Promise<MoveAck> {
    const values = [angles.J1, angles.J2, angles.J3, angles.J4, angles.J5, angles.J6]
      .map(v => v.toFixed(3))
      .join(' ');

    const ack = this.awaitMoveAck();
    await this.sendCommand(`J ${values} ${speed.toFixed(2)}`);
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

  onMessage(callback: (msg: SerialMessage) => void): void {
    this.messageCallbacks.push(callback);
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

  private async startReading(): Promise<void> {
    this.isReading = true;
    const decoder = new TextDecoder();
    let buffer = '';

    try {
      while (this.isReading && this.reader) {
        const { value, done } = await this.reader.read();
        if (done) break;

        buffer += decoder.decode(value, { stream: true });

        const lines = buffer.split('\n');
        buffer = lines.pop() ?? '';

        for (const line of lines) {
          this.processLine(line.trim());
        }
      }
    } catch (error) {
      this.isReading = false;
      this.rejectPendingAcks(
        error instanceof Error ? error : new Error('Serial read failed')
      );
    }
  }

  private processLine(line: string): void {
    if (line.length === 0) return;

    const parts = line.split(/\s+/);
    const timestamp = Date.now();

    switch (parts[0]) {
      case 'POS': {
        // POS j1 j2 j3 j4 j5 j6
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

      case 'STATUS': {
        // STATUS <state> <queueFree> <moving> <positionTrusted> <homedMask>
        if (parts.length !== 6) return;

        const homedMask = parseInt(parts[5], 10);
        const status: FirmwareStatus = {
          state: parts[1] as FirmwareStatus['state'],
          queueFree: parseInt(parts[2], 10),
          moving: parts[3] === '1',
          positionTrusted: parts[4] === '1',
          homed: Array.from({ length: 6 }, (_, i) =>
            Number.isNaN(homedMask) ? false : (homedMask & (1 << i)) !== 0
          )
        };

        if (Number.isNaN(status.queueFree)) return;

        this.lastStatus = status;
        this.emit({ type: 'STATUS', data: status, timestamp });
        return;
      }

      case 'BUSY': {
        // Queue full; the move was not accepted.
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
        // "OK J <free>" acknowledges a queued move; every other OK is informational.
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
        // An error in reply to a move must not leave the sender hanging.
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
    for (const callback of this.messageCallbacks) {
      callback(message);
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
