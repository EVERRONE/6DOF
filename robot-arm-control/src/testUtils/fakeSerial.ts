// Fake Web Serial, shared by the transport and store tests.
//
// SerialManager takes the Serial interface as a constructor argument, so tests can
// push bytes a chunk at a time, unplug the cable mid-command, and make writes
// fail - none of which is reachable against a real USB device.

import {
  SerialLike,
  SerialPortEventLike,
  SerialPortLike,
  SerialReaderLike,
  SerialWriterLike
} from '../communication/webSerial';

export class FakePort implements SerialPortLike {
  opened = false;
  closed = false;
  written: string[] = [];

  /** Set to fail the next write, standing in for a dead link. */
  failWrites = false;

  /**
   * Optional stand-in for the firmware: given a command, return the line to
   * reply with, or null for no reply. Called synchronously on write, so a test
   * driving a command stream needs no timers.
   */
  autoRespond: ((command: string) => string | null) | null = null;

  private chunks: Uint8Array[] = [];
  private waiting: ((r: { value?: Uint8Array; done: boolean }) => void) | null = null;
  private ended = false;
  private readError: Error | null = null;
  private cancelled = false;

  readonly readable = {
    getReader: (): SerialReaderLike => ({
      read: () => this.read(),
      cancel: async () => {
        this.cancelled = true;
        this.flushWaiting({ done: true });
      },
      releaseLock: () => undefined
    })
  };

  readonly writable = {
    getWriter: (): SerialWriterLike => ({
      write: async (data: Uint8Array) => {
        if (this.failWrites) throw new Error('device gone');
        const text = new TextDecoder().decode(data);
        this.written.push(text);

        const reply = this.autoRespond?.(text.trim());
        if (reply) this.push(reply);
      },
      releaseLock: () => undefined
    })
  };

  async open(): Promise<void> {
    this.opened = true;
    this.closed = false;
    // A reopened port hands out a fresh stream, so clear whatever ended the last
    // one. Without this a reconnect would immediately see the old end-of-stream.
    this.cancelled = false;
    this.ended = false;
    this.readError = null;
    this.chunks = [];
  }

  async close(): Promise<void> {
    this.closed = true;
    this.opened = false;
  }

  /** Deliver bytes to the reader, exactly as given so chunking can be tested. */
  push(text: string): void {
    const bytes = new TextEncoder().encode(text);
    if (this.waiting) {
      const resolve = this.waiting;
      this.waiting = null;
      resolve({ value: bytes, done: false });
      return;
    }
    this.chunks.push(bytes);
  }

  /** The device closed the stream, as on a clean unplug. */
  endStream(): void {
    this.ended = true;
    this.flushWaiting({ done: true });
  }

  /** The read threw, as on an abrupt unplug. */
  failStream(message = 'The device has been lost'): void {
    this.readError = new Error(message);
    const resolve = this.waiting;
    this.waiting = null;
    if (resolve) {
      // A pending read must reject; resolve with done and let the next read throw.
      resolve({ done: true });
    }
  }

  /** The most recent command written, without its newline. */
  lastCommand(): string {
    return (this.written[this.written.length - 1] ?? '').trim();
  }

  private read(): Promise<{ value?: Uint8Array; done: boolean }> {
    if (this.readError) {
      const error = this.readError;
      this.readError = null;
      return Promise.reject(error);
    }
    const next = this.chunks.shift();
    if (next) return Promise.resolve({ value: next, done: false });
    if (this.ended || this.cancelled) return Promise.resolve({ done: true });

    return new Promise(resolve => {
      this.waiting = resolve;
    });
  }

  private flushWaiting(result: { value?: Uint8Array; done: boolean }): void {
    const resolve = this.waiting;
    this.waiting = null;
    resolve?.(result);
  }
}

export class FakeSerial implements SerialLike {
  ports: FakePort[] = [];
  requestCount = 0;

  /** Ports getPorts() reports. Emptied to model a cable still unplugged. */
  granted: FakePort[] = [];

  private listeners: ((event: SerialPortEventLike) => void)[] = [];

  constructor(port?: FakePort) {
    if (port) {
      this.ports.push(port);
      this.granted.push(port);
    }
  }

  async requestPort(): Promise<SerialPortLike> {
    this.requestCount++;
    if (this.ports.length === 0) throw new Error('No port selected');
    return this.ports[0];
  }

  async getPorts(): Promise<SerialPortLike[]> {
    return this.granted;
  }

  addEventListener(_type: 'connect' | 'disconnect', listener: (e: SerialPortEventLike) => void): void {
    this.listeners.push(listener);
  }

  removeEventListener(
    _type: 'connect' | 'disconnect',
    listener: (e: SerialPortEventLike) => void
  ): void {
    this.listeners = this.listeners.filter(l => l !== listener);
  }

  /** Fire the browser's disconnect event, as on physical removal. */
  emitDisconnect(target?: unknown): void {
    for (const listener of [...this.listeners]) listener({ target });
  }
}

/** Commands of a given kind that were written, in order. */
export function commandsOfType(port: FakePort, prefix: string): string[] {
  return port.written.map(w => w.trim()).filter(w => w.startsWith(prefix));
}
