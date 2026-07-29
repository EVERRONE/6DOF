export interface JointAngles {
  J1: number;
  J2: number;
  J3: number;
  J4: number;
  J5: number;
  J6: number;
}

export interface JointLimits {
  min: number;
  max: number;
}

export interface RobotConfig {
  joints: {
    [key: string]: {
      min: number;
      max: number;
      ustepsPerDeg: number;
      hasEndstop: boolean;
    };
  };
}

export interface EndstopState {
  J1: boolean;
  J2: boolean;
  J3: boolean;
  J4: boolean;
  J5: boolean;
  J6: boolean;
}

export interface CartesianPose {
  position: { x: number; y: number; z: number };
  orientation: { x: number; y: number; z: number }; // Euler angles (degrees)
}

export enum ConnectionStatus {
  DISCONNECTED = 'disconnected',
  CONNECTING = 'connecting',
  CONNECTED = 'connected',
  /** Link dropped unexpectedly; the manager is trying to reopen the port. */
  RECONNECTING = 'reconnecting',
  ERROR = 'error'
}

export enum RobotState {
  IDLE = 'idle',
  MOVING = 'moving',
  HOMING = 'homing',
  ERROR = 'error',
  ESTOPPED = 'estopped'
}
