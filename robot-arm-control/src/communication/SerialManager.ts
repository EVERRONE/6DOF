import { SerialMessage } from './types';
import { JointAngles, EndstopState } from '../types/robot';

export class SerialManager {
  private port: SerialPort | null = null;
  private reader: ReadableStreamDefaultReader | null = null;
  private writer: WritableStreamDefaultWriter | null = null;

  private messageCallbacks: ((msg: SerialMessage) => void)[] = [];
  private isReading = false;

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

    const encoder = new TextEncoder();
    const data = encoder.encode(cmd + '\n');

    await this.writer.write(data);
    console.log('Sent:', cmd);
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

  // Subscribe to messages
  onMessage(callback: (msg: SerialMessage) => void): void {
    this.messageCallbacks.push(callback);
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
}
