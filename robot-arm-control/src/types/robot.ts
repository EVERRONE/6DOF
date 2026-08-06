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

export interface JointCalibration {
  scale: number;
  offset: number;
}

export interface FirmwareJointConfig {
  min: number;
  max: number;
  stepsPerDeg: number;
  invertDir: boolean;
  hasEndstop: boolean;
  homeTowardMin: boolean;
  homeLogicalDeg: number;
  postHomeOffsetDeg: number;
  urdfOffsetDeg: number;
  cal: JointCalibration;
}

export interface HomePoseConfig {
  enabled: boolean;
  speedDegS: number;
  applyAfterHAll: boolean;
  // J2..J5 values are also used by UI as URDF-default anchors for FK/IK/3D mapping.
  jointsDeg: number[];
}

export interface FirmwareCapabilities {
  trajectoryQueue: boolean;
  trajectoryMaxPoints: number;
  trajectoryPointFormat: 'hermite_v1' | string;
  motionKernelV2?: boolean;
  motionKernelDiag?: boolean;
  commandAckV1?: boolean;
  trajectoryErrorCodesV1?: boolean;
  trajectoryStatusV2?: boolean;
}

export interface FirmwareConfig {
  version: number;
  protocolVersion?: number;
  capabilities?: FirmwareCapabilities;
  joints: FirmwareJointConfig[];
  pulseWidthUs: number;
  dirSetupUs: number;
  endstopDebounceMs: number;
  homePose?: HomePoseConfig;
}

export interface EndstopState {
  J1: boolean;
  J2: boolean;
  J3: boolean;
  J4: boolean;
  J5: boolean;
  J6: boolean;
}

export interface HomedState {
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
  ERROR = 'error'
}

export enum RobotState {
  IDLE = 'idle',
  MOVING = 'moving',
  HOMING = 'homing',
  ERROR = 'error',
  ESTOPPED = 'estopped'
}
