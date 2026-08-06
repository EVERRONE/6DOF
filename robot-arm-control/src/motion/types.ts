// Motion planning type definitions for 6DOF robot arm

import { IKResult, Rotation3, Vector3 } from '../kinematics/types';
import { IKTrackingMode } from '../kinematics/types';

/**
 * A waypoint in the path (Cartesian space)
 */
export interface Waypoint {
  id: string;
  position: Vector3;
  orientation?: { roll: number; pitch: number; yaw: number };
  /** Joint angles snapshot at this waypoint (if taught) */
  jointAngles?: number[];
  /** Speed for moving TO this waypoint (mm/s) */
  speed: number;
  /** Label for UI display */
  label?: string;
}

/**
 * Velocity profile type
 */
export type ProfileType = 'trapezoidal' | 'triangular';

/**
 * Velocity profile parameters
 */
export interface VelocityProfileConfig {
  maxVelocity: number;    // degrees/s (joint space) or mm/s (Cartesian)
  maxAcceleration: number; // degrees/s² or mm/s²
}

/**
 * A single point along a trajectory (time-parameterized)
 */
export interface TrajectoryPoint {
  time: number;           // seconds from segment start
  jointAngles: number[];  // 6 joint angles in degrees
  velocity?: number[];    // joint velocities (deg/s)
}

/**
 * A trajectory segment between two waypoints
 */
export interface TrajectorySegment {
  startWaypoint: number;  // index
  endWaypoint: number;    // index
  points: TrajectoryPoint[];
  duration: number;       // total time in seconds
  distance: number;       // Cartesian distance in meters
}

export interface CartesianInterpolationResult {
  success: boolean;
  segment: TrajectorySegment;
  error?: string;
  failedAtTime?: number;
  timedOut?: boolean;
  timeoutAtSampleIndex?: number;
  timeoutAtTimeSec?: number;
  targetOrientation?: Rotation3;
  lastIK?: IKResult;
  trackingMode?: IKTrackingMode;
}

/**
 * A complete trajectory (multiple segments)
 */
export interface Trajectory {
  segments: TrajectorySegment[];
  totalDuration: number;  // seconds
  totalDistance: number;   // meters
  pointCount: number;
  waypoints: Waypoint[];
}

/**
 * Execution state of a trajectory
 */
export enum ExecutionState {
  IDLE = 'idle',
  PLANNING = 'planning',
  EXECUTING = 'executing',
  PAUSED = 'paused',
  COMPLETED = 'completed',
  ERROR = 'error'
}

/**
 * Progress info during execution
 */
export interface ExecutionProgress {
  state: ExecutionState;
  currentSegment: number;
  totalSegments: number;
  currentPointInSegment: number;
  totalPointsInSegment: number;
  overallProgress: number;     // 0-100
  elapsedTime: number;         // seconds
  estimatedTimeRemaining: number; // seconds
}

/**
 * Interpolation mode for path segments
 */
export type InterpolationMode = 'joint' | 'linear';

/**
 * Path planner configuration
 */
export interface PathPlannerConfig {
  interpolationMode: InterpolationMode;
  defaultSpeed: number;           // mm/s
  defaultAcceleration: number;    // mm/s²
  maxJointSpeed: number;          // deg/s
  maxJointAcceleration: number;   // deg/s²
  pointsPerSecond: number;        // trajectory sampling rate (Hz)
  loopCount: number;              // 0 = no loop, >0 = repeat N times
}

/**
 * Saved path file format
 */
export interface SavedPath {
  name: string;
  description?: string;
  waypoints: Waypoint[];
  config: PathPlannerConfig;
  createdAt: string;
  version: string;
  kinematicsFrame?: 'urdf_chain_v1' | 'legacy_dh_v1';
}
