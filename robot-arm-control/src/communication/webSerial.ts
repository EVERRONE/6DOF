// Minimal structural types for the parts of the Web Serial API this app uses.
//
// The real navigator.serial satisfies these, and SerialManager takes one as a
// constructor argument. That is what makes the connection lifecycle testable: a
// test can supply a fake port, drop the link mid-command, and check that the app
// recovers - none of which is possible against a real USB device in CI.
//
// Only what is actually called is declared. Anything wider would be a promise
// this code does not keep.

export interface SerialReaderLike {
  read(): Promise<{ value?: Uint8Array; done: boolean }>;
  cancel(): Promise<void>;
  releaseLock(): void;
}

export interface SerialWriterLike {
  write(data: Uint8Array): Promise<void>;
  releaseLock(): void;
}

export interface SerialPortLike {
  readonly readable: { getReader(): SerialReaderLike } | null;
  readonly writable: { getWriter(): SerialWriterLike } | null;
  open(options: { baudRate: number }): Promise<void>;
  close(): Promise<void>;
}

/** The `disconnect` event carries the port that went away. */
export interface SerialPortEventLike {
  target?: unknown;
}

export interface SerialLike {
  requestPort(): Promise<SerialPortLike>;
  /** Ports the user has already granted, reopenable without a new gesture. */
  getPorts(): Promise<SerialPortLike[]>;
  addEventListener(
    type: 'connect' | 'disconnect',
    listener: (event: SerialPortEventLike) => void
  ): void;
  removeEventListener(
    type: 'connect' | 'disconnect',
    listener: (event: SerialPortEventLike) => void
  ): void;
}

/**
 * The browser's Serial interface, or null where the API is unavailable -
 * every browser except Chrome and Edge, and any test environment.
 */
export function getBrowserSerial(): SerialLike | null {
  if (typeof navigator === 'undefined') return null;
  const candidate = (navigator as unknown as { serial?: SerialLike }).serial;
  return candidate ?? null;
}
