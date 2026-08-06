import { SerialMessage } from './types';
import { JointAngles, EndstopState, FirmwareConfig } from '../types/robot';

export interface CommandTransactionOptions {
  expectedTypes?: SerialMessage['type'][];
  timeoutMs?: number;
  retries?: number;
  predicate?: (msg: SerialMessage) => boolean;
}

export class SerialManager {
  private port: SerialPort | null = null;
  private reader: ReadableStreamDefaultReader | null = null;
  private writer: WritableStreamDefaultWriter | null = null;

  private messageCallbacks: ((msg: SerialMessage) => void)[] = [];
  private isReading = false;
  private static readonly DEFAULT_TRANSACTION_TIMEOUT_MS = 1200;
  private static readonly DEFAULT_TRANSACTION_RETRIES = 1;

  private static encodeCommand(cmd: string): Uint8Array {
    const line = `${cmd}\n`;
    const bytes = new Uint8Array(line.length);
    for (let i = 0; i < line.length; i++) {
      const code = line.charCodeAt(i);
      if (code > 0x7f) {
        throw new Error(`Serial command contains non-ASCII character at index ${i}`);
      }
      bytes[i] = code;
    }
    return bytes;
  }

  private static formatDeviceError(msg: SerialMessage, cmd?: string): string {
    const normalizedScope = cmd
      ? cmd.trim().split(/\s+/)[0]?.toUpperCase().replace(/[^A-Z0-9_]/g, '') || ''
      : '';
    if (msg.type === 'TQ_ERR') {
      if (typeof msg.data === 'string') {
        return `TQ ${msg.data}`;
      }
      const code = msg.data?.code ? String(msg.data.code) : 'error';
      const detail = msg.data?.detail ? String(msg.data.detail) : '';
      return detail.length > 0 ? `TQ ${code}: ${detail}` : `TQ ${code}`;
    }
    if (msg.type === 'ERROR') {
      return typeof msg.data === 'string' && msg.data.length > 0
        ? msg.data
        : (normalizedScope.length > 0
          ? `${normalizedScope} DEVICE_ERROR`
          : 'Device returned ERROR');
    }
    return normalizedScope.length > 0
      ? `${normalizedScope} DEVICE_ERROR`
      : 'Device returned ERROR';
  }

  // Check if Web Serial API is supported
  static isSupported(): boolean {
    return 'serial' in navigator;
  }

  // Connect to the Teensy
  async connect(): Promise<void> {
    try {
      // Request port from user
      this.port = await navigator.serial.requestPort();

      // Open with correct baud rate
      await this.port.open({ baudRate: 115200 });

      // Get reader and writer
      this.reader = this.port.readable!.getReader();
      this.writer = this.port.writable!.getWriter();

      // Start reading
      this.startReading();

      console.log('Connected to robot arm');
    } catch (error) {
      console.error('Connection failed:', error);
      throw error;
    }
  }

  // Disconnect
  async disconnect(): Promise<void> {
    this.isReading = false;

    if (this.reader) {
      await this.reader.cancel();
      this.reader.releaseLock();
      this.reader = null;
    }

    if (this.writer) {
      this.writer.releaseLock();
      this.writer = null;
    }

    if (this.port) {
      await this.port.close();
      this.port = null;
    }

    console.log('Disconnected from robot arm');
  }

  // Send command
  async sendCommand(cmd: string): Promise<void> {
    if (!this.writer) {
      throw new Error('Not connected');
    }

    const data = SerialManager.encodeCommand(cmd);

    await this.writer.write(data);
    console.log('Sent:', cmd);
  }

  async sendAndAwait(cmd: string, options?: CommandTransactionOptions): Promise<SerialMessage> {
    const expectedTypes = options?.expectedTypes && options.expectedTypes.length > 0
      ? options.expectedTypes
      : ['OK'];
    const timeoutMs = Math.max(50, options?.timeoutMs ?? SerialManager.DEFAULT_TRANSACTION_TIMEOUT_MS);
    const retries = Math.max(0, options?.retries ?? SerialManager.DEFAULT_TRANSACTION_RETRIES);
    const predicate = options?.predicate;
    let lastError: Error | null = null;

    for (let attempt = 0; attempt <= retries; attempt++) {
      const waiter = this.createMessageWaiter(
        (msg) => {
          if (msg.type === 'ERROR' || msg.type === 'TQ_ERR') {
            throw new Error(SerialManager.formatDeviceError(msg, cmd));
          }
          const typeMatch = expectedTypes.includes(msg.type);
          if (!typeMatch) return false;
          return predicate ? predicate(msg) : true;
        },
        timeoutMs
      );
      try {
        await this.sendCommand(cmd);
        return await waiter.promise;
      } catch (error) {
        waiter.cancel();
        lastError = error instanceof Error ? error : new Error(String(error));
        if (attempt >= retries) {
          break;
        }
      }
    }

    throw (lastError || new Error(`Command transaction failed for: ${cmd}`));
  }

  // Command builders
  async moveToAngles(angles: JointAngles, speed: number): Promise<void> {
    const cmd = `J ${angles.J1} ${angles.J2} ${angles.J3} ${angles.J4} ${angles.J5} ${angles.J6} ${speed}`;
    await this.sendCommand(cmd);
  }

  async homeJoints(joints: string): Promise<void> {
    const cmd = `H ${joints}`;
    await this.sendCommand(cmd);
  }

  async queryPosition(): Promise<void> {
    await this.sendCommand('Q');
  }

  async enableMotors(enable: boolean): Promise<void> {
    const cmd = `E ${enable ? 1 : 0}`;
    await this.sendCommand(cmd);
  }

  async emergencyStop(): Promise<void> {
    await this.sendCommand('S');
  }

  async queryConfig(): Promise<void> {
    await this.sendCommand('CFG?');
  }

  async queryHomePose(): Promise<void> {
    await this.sendCommand('HP?');
  }

  async setConfig(config: FirmwareConfig): Promise<void> {
    const cmd = `CFG ${JSON.stringify(config)}`;
    await this.sendCommand(cmd);
  }

  async setHomePoseEnabled(enabled: boolean): Promise<void> {
    await this.sendCommand(`HP EN ${enabled ? 1 : 0}`);
  }

  async setHomePoseJoint(joint: number, angleDeg: number): Promise<void> {
    await this.sendCommand(`HP SET J${joint} ${angleDeg}`);
  }

  async setHomePoseAll(j2: number, j3: number, j4: number, j5: number): Promise<void> {
    await this.sendCommand(`HP SETALL ${j2} ${j3} ${j4} ${j5}`);
  }

  async setHomePoseSpeed(speedDegS: number): Promise<void> {
    await this.sendCommand(`HP SPD ${speedDegS}`);
  }

  async saveHomePose(): Promise<void> {
    await this.sendCommand('HP SAVE');
  }

  async loadHomePose(): Promise<void> {
    await this.sendCommand('HP LOAD');
  }

  async resetHomePose(): Promise<void> {
    await this.sendCommand('HP RESET');
  }

  async setCalibration(joint: number, scale: number, offset: number): Promise<void> {
    const cmd = `CAL SET J${joint} ${scale} ${offset}`;
    await this.sendCommand(cmd);
  }

  async zeroCalibration(joint: number, logicalDeg: number): Promise<void> {
    const cmd = `CAL ZERO J${joint} ${logicalDeg}`;
    await this.sendCommand(cmd);
  }

  async saveCalibration(): Promise<void> {
    await this.sendCommand('CAL SAVE');
  }

  async loadCalibration(): Promise<void> {
    await this.sendCommand('CAL LOAD');
  }

  async resetCalibration(): Promise<void> {
    await this.sendCommand('CAL RESET');
  }

  async jogRelative(joint: number, deltaDeg: number, speed: number): Promise<void> {
    const cmd = `JR J${joint} ${deltaDeg} ${speed}`;
    await this.sendCommand(cmd);
  }

  async clearTrajectoryQueue(): Promise<void> {
    await this.sendAndAwait('TQ CLEAR', {
      expectedTypes: ['TQ_READY', 'ACK'],
      predicate: (msg) => (
        msg.type === 'ACK' || (msg.type === 'TQ_READY' && (msg.data?.count ?? 0) === 0)
      ),
      timeoutMs: 1500,
      retries: 2
    });
  }

  async enqueueTrajectoryPoint(tMs: number, qDeg: number[], qdDegS: number[]): Promise<void> {
    if (qDeg.length !== 6 || qdDegS.length !== 6) {
      throw new Error('Trajectory point requires 6 joint angles and 6 joint velocities');
    }
    const values = [Math.round(tMs), ...qDeg, ...qdDegS].map((v) => Number(v).toString());
    await this.sendAndAwait(`TQ PT ${values.join(' ')}`, {
      expectedTypes: ['OK', 'ACK'],
      predicate: (msg) => (
        msg.type === 'ACK' ||
        (msg.type === 'OK' && typeof msg.data === 'string' && msg.data.includes('TQ pt'))
      ),
      timeoutMs: 1500,
      retries: 2
    });
  }

  async runTrajectoryQueue(): Promise<void> {
    await this.sendAndAwait('TQ RUN', {
      expectedTypes: ['OK', 'ACK'],
      predicate: (msg) => (
        msg.type === 'ACK' ||
        (msg.type === 'OK' && typeof msg.data === 'string' && msg.data.includes('TQ running'))
      ),
      timeoutMs: 2000,
      retries: 1
    });
  }

  async stopTrajectoryQueue(): Promise<void> {
    await this.sendAndAwait('TQ STOP', {
      expectedTypes: ['OK', 'ACK'],
      predicate: (msg) => (
        msg.type === 'ACK' ||
        (msg.type === 'OK' && typeof msg.data === 'string' && msg.data.includes('TQ stopped'))
      ),
      timeoutMs: 1500,
      retries: 1
    });
  }

  async queryTrajectoryQueue(): Promise<SerialMessage> {
    return this.sendAndAwait('TQ?', {
      expectedTypes: ['TQ_READY', 'TQ_STAT'],
      timeoutMs: 1500,
      retries: 1
    });
  }

  async queryMotionKernel(): Promise<SerialMessage> {
    return this.sendAndAwait('MQ?', {
      expectedTypes: ['MQ_STAT'],
      timeoutMs: 1500,
      retries: 1
    });
  }

  // Subscribe to messages
  onMessage(callback: (msg: SerialMessage) => void): () => void {
    this.messageCallbacks.push(callback);
    return () => {
      this.messageCallbacks = this.messageCallbacks.filter((cb) => cb !== callback);
    };
  }

  // Read loop
  private async startReading(): Promise<void> {
    this.isReading = true;
    const decoder = new TextDecoder();
    let buffer = '';

    try {
      while (this.isReading && this.reader) {
        const { value, done } = await this.reader.read();

        if (done) break;

        buffer += decoder.decode(value, { stream: true });

        // Process complete lines
        const lines = buffer.split('\n');
        buffer = lines.pop() || '';  // Keep incomplete line in buffer

        for (const line of lines) {
          this.processLine(line.trim());
        }
      }
    } catch (error) {
      console.error('Read error:', error);
      this.isReading = false;
    }
  }

  // Parse incoming messages
  private processLine(line: string): void {
    if (line.length === 0) return;

    const parts = line.split(' ');
    const type = parts[0];

    let message: SerialMessage | null = null;

    switch (type) {
      case 'POS':
        // POS 0.00 5.00 55.00 129.00 220.00 0.00
        if (parts.length === 7) {
          message = {
            type: 'POS',
            data: {
              J1: parseFloat(parts[1]),
              J2: parseFloat(parts[2]),
              J3: parseFloat(parts[3]),
              J4: parseFloat(parts[4]),
              J5: parseFloat(parts[5]),
              J6: parseFloat(parts[6])
            } as JointAngles,
            timestamp: Date.now()
          };
        }
        break;

      case 'ENDSTOP':
        // ENDSTOP 0 1 0 0 0 0
        if (parts.length === 7) {
          message = {
            type: 'ENDSTOP',
            data: {
              J1: parts[1] === '1',
              J2: parts[2] === '1',
              J3: parts[3] === '1',
              J4: parts[4] === '1',
              J5: parts[5] === '1',
              J6: parts[6] === '1'
            } as EndstopState,
            timestamp: Date.now()
          };
        }
        break;

      case 'OK':
        message = {
          type: 'OK',
          data: parts.slice(1).join(' '),
          timestamp: Date.now()
        };
        break;

      case 'ERROR':
        if (parts[1] === 'TQ') {
          message = {
            type: 'TQ_ERR',
            data: parts.slice(2).join(' '),
            timestamp: Date.now()
          };
          break;
        }
        message = {
          type: 'ERROR',
          data: parts.slice(1).join(' '),
          timestamp: Date.now()
        };
        break;

      case 'HOMED':
        message = {
          type: 'HOMED',
          data: parts[1] ? parseInt(parts[1]) : null,
          timestamp: Date.now()
        };
        break;
      case 'CFG':
        {
          const json = line.slice(4).trim();
          try {
            const config = JSON.parse(json);
            message = {
              type: 'CFG',
              data: config,
              timestamp: Date.now()
            };
          } catch (error) {
            console.warn('Failed to parse CFG message:', error);
          }
        }
        break;
      case 'HP':
        {
          const json = line.slice(3).trim();
          try {
            const homePose = JSON.parse(json);
            message = {
              type: 'HP',
              data: homePose,
              timestamp: Date.now()
            };
          } catch (error) {
            console.warn('Failed to parse HP message:', error);
          }
        }
        break;
      case 'HOMEPOSE_REACHED':
        message = {
          type: 'HOMEPOSE_REACHED',
          data: null,
          timestamp: Date.now()
        };
        break;
      case 'TQ':
        if (parts[1] === 'READY' && parts.length >= 3) {
          message = {
            type: 'TQ_READY',
            data: {
              count: Number.parseInt(parts[2], 10) || 0
            },
            timestamp: Date.now()
          };
        } else if (parts[1] === 'PROG' && parts.length >= 4) {
          message = {
            type: 'TQ_PROG',
            data: {
              pointIndex: Number.parseInt(parts[2], 10) || 0,
              elapsedMs: Number.parseInt(parts[3], 10) || 0
            },
            timestamp: Date.now()
          };
        } else if (parts[1] === 'DONE') {
          message = {
            type: 'TQ_DONE',
            data: null,
            timestamp: Date.now()
          };
        } else if (parts[1] === 'ERR' && parts.length >= 3) {
          message = {
            type: 'TQ_ERR',
            data: {
              code: parts[2],
              detail: parts.slice(3).join(' ')
            },
            timestamp: Date.now()
          };
        } else if (parts[1] === 'STAT' && parts.length >= 7) {
          message = {
            type: 'TQ_STAT',
            data: {
              count: Number.parseInt(parts[2], 10) || 0,
              running: parts[3] === '1',
              pointIndex: Number.parseInt(parts[4], 10) || 0,
              elapsedMs: Number.parseInt(parts[5], 10) || 0,
              seq: Number.parseInt(parts[6], 10) || 0
            },
            timestamp: Date.now()
          };
        }
        break;
      case 'MQ':
        if (parts[1] === 'STAT' && parts.length >= 5) {
          message = {
            type: 'MQ_STAT',
            data: {
              tickJitterUs: Number.parseInt(parts[2], 10) || 0,
              queueUnderrun: Number.parseInt(parts[3], 10) || 0,
              stepOverrun: Number.parseInt(parts[4], 10) || 0
            },
            timestamp: Date.now()
          };
        }
        break;
      case 'ACK':
        message = {
          type: 'ACK',
          data: {
            seq: parts.length >= 2 ? Number.parseInt(parts[1], 10) || 0 : 0,
            scope: parts.length >= 3 ? parts[2] : '',
            detail: parts.slice(3).join(' ')
          },
          timestamp: Date.now()
        };
        break;
    }

    if (message) {
      this.notifyListeners(message);
    }
  }

  private notifyListeners(message: SerialMessage): void {
    for (const callback of this.messageCallbacks) {
      callback(message);
    }
  }

  private createMessageWaiter(
    predicate: (msg: SerialMessage) => boolean,
    timeoutMs: number
  ): { promise: Promise<SerialMessage>; cancel: () => void } {
    let settled = false;
    let unsubscribe: (() => void) | null = null;
    let timer: ReturnType<typeof setTimeout> | null = null;

    const cleanup = (): void => {
      if (timer) {
        clearTimeout(timer);
        timer = null;
      }
      if (unsubscribe) {
        unsubscribe();
        unsubscribe = null;
      }
    };

    const promise = new Promise<SerialMessage>((resolve, reject) => {
      unsubscribe = this.onMessage((msg) => {
        if (settled) return;
        try {
          if (!predicate(msg)) return;
          settled = true;
          cleanup();
          resolve(msg);
        } catch (error) {
          settled = true;
          cleanup();
          reject(error instanceof Error ? error : new Error(String(error)));
        }
      });
      timer = setTimeout(() => {
        if (settled) return;
        settled = true;
        cleanup();
        reject(new Error(`Command timed out after ${timeoutMs}ms`));
      }, timeoutMs);
    });

    return {
      promise,
      cancel: () => {
        if (settled) return;
        settled = true;
        cleanup();
      }
    };
  }
}
