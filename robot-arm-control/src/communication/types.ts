import { JointAngles, EndstopState } from '../types/robot';

/**
 * Firmware controller state, as reported in a STATUS line.
 */
export type FirmwareState = 'IDLE' | 'MOVING' | 'HOMING' | 'ERROR' | 'ESTOP';

/**
 * Contents of a STATUS line.
 *
 * `queueFree` is what makes streaming work: the firmware plans ahead across the
 * moves it holds, so the host keeps the queue full rather than pacing points off
 * its own clock. See docs/SERIAL_PROTOCOL.md.
 */
export interface FirmwareStatus {
  state: FirmwareState;
  /** Free slots in the firmware motion queue. */
  queueFree: number;
  /** True while step generation is active. */
  moving: boolean;
  /**
   * False when a hard stop may have lost steps, so the reported angles cannot be
   * relied on until the arm is re-homed.
   */
  positionTrusted: boolean;
  /** Per-joint: has this joint been homed since power-up? */
  homed: boolean[];
}

/** Reply to a queued move. */
export interface MoveAck {
  /** True when the move was queued, false when the queue was full. */
  accepted: boolean;
  /** Free slots remaining after the command. */
  queueFree: number;
}

export type SerialMessage =
  | { type: 'POS'; data: JointAngles; timestamp: number }
  | { type: 'ENDSTOP'; data: EndstopState; timestamp: number }
  | { type: 'STATUS'; data: FirmwareStatus; timestamp: number }
  | { type: 'MOVE_ACK'; data: MoveAck; timestamp: number }
  | { type: 'OK'; data: string; timestamp: number }
  | { type: 'ERROR'; data: string; timestamp: number }
  | { type: 'HOMED'; data: number | null; timestamp: number };
