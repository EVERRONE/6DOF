// Web Serial API type declarations
// https://wicg.github.io/serial/

interface SerialPort {
  readonly readable: ReadableStream<Uint8Array> | null;
  readonly writable: WritableStream<Uint8Array> | null;
  open(options: { baudRate: number }): Promise<void>;
  close(): Promise<void>;
}

interface SerialPortRequestOptions {
  filters?: { usbVendorId?: number; usbProductId?: number }[];
}

interface Serial {
  requestPort(options?: SerialPortRequestOptions): Promise<SerialPort>;
  getPorts(): Promise<SerialPort[]>;
}

interface Navigator {
  readonly serial: Serial;
}
