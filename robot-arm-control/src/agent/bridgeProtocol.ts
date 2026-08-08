/**
 * Wire contract with the broker.
 * Keep in sync with robot-agent-bridge/src/protocol.js.
 */

import { AgentDirection, AgentFrame } from './frames';

export const BRIDGE_PROTOCOL_VERSION = 1;

export type BridgeCommandType = 'get_status' | 'preview_move' | 'stop';

export interface BridgeCommand {
  v: number;
  id: string;
  type: BridgeCommandType;
  params: Record<string, unknown>;
}

export interface PreviewMoveParams {
  direction: AgentDirection;
  frame: AgentFrame;
  distanceMm: number;
  requestedDistanceMm: number;
  clamped: boolean;
}

export interface BridgeError {
  code: string;
  message: string;
  notes?: string[];
}

export type BridgeResponse =
  | { v: number; id: string; ok: true; result: unknown }
  | { v: number; id: string; ok: false; error: BridgeError };

export interface BridgeTelemetry {
  armed: boolean;
  armedUntil: number | null;
  connected: boolean;
  motorsEnabled: boolean;
  cartesianReady: boolean;
  robotState: string;
  planningState: string;
  busy: boolean;
}

export type BridgeLinkState =
  | 'disabled'
  | 'connecting'
  | 'connected'
  | 'reconnecting'
  | 'error';
