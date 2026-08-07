// Motion planning type definitions for 6DOF robot arm

import { Vector3 } from '../kinematics/types';

/**
 * A waypoint in the path (Cartesian space)
 */
/** Plane of a figure, in base coordinates. */
export type ShapePlane = 'XY' | 'XZ' | 'YZ';

/**
 * A figure attached to a waypoint.
 *
 * The waypoint's position is the figure's centre, and the figure is expanded
 * into a path only at planning time. Holding it as one item is what lets the
 * list show one row, the 3D view draw one marker, and a delete remove the
 * circle rather than forty points of it - and it lets the planner sample along
 * the true arc rather than chord between points chosen earlier.
 */
export type WaypointShape = {
  kind: 'circle';
  /** Radius in metres. */
  radius: number;
  plane: ShapePlane;
  /** Where on the circle to start and finish, in degrees. */
  startAngleDeg?: number;
  clockwise?: boolean;
};

export interface Waypoint {
  id: string;
  position: Vector3;
  /** When set, this waypoint is a figure centred on `position`. */
  shape?: WaypointShape;
  orientation?: { roll: number; pitch: number; yaw: number };
  /** Joint angles snapshot at this waypoint (if taught) */
  jointAngles?: number[];
  /**
   * Feed rate for the move TO this waypoint.
   *
   * The unit follows the interpolation mode: mm/s in 'linear' mode, deg/s in
   * 'joint' mode. Either way the firmware clamps it to the per-joint limits, so
   * an over-ambitious value is safe - it just runs as fast as the arm allows.
   */
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
  /**
   * Samples along a linear segment where IK found no solution, so the previous
   * pose was held instead. Non-zero means part of the commanded straight line is
   * out of reach and the arm will cut the corner there.
   */
  unreachableSamples?: number;
  /**
   * Where consecutive samples jump in joint space, or null when the path is
   * continuous.
   *
   * Two samples 2 mm apart normally differ by a degree or so. Near a singularity
   * they can differ by tens of degrees while the tool barely moves, because the
   * arm is free to reconfigure in a direction the tool cannot see. The tool pose
   * is correct at both samples and wrong everywhere between them, and no amount
   * of finer Cartesian sampling fixes it - the discontinuity is in joint space,
   * not in the line.
   */
  discontinuity?: {
    /** Sample index the jump lands on. */
    index: number;
    /** How far along the segment, as a percentage. */
    atPercent: number;
    /** Zero-based joint that moved furthest. */
    axis: number;
    /** How far it moved, in degrees. */
    degrees: number;
  } | null;
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
  /**
   * Total samples across all segments where IK found no solution. Non-zero means
   * part of the planned path is out of reach, and the arm will not follow the
   * line there.
   */
  unreachableSamples: number;
  /** Waypoints that could not be resolved at all, and were left out of the plan. */
  skippedWaypoints: string[];
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

  /**
   * Hold the tool's orientation for the whole path.
   *
   * The orientation held is the one the arm starts the path in. Without it a
   * path solves for position alone and the wrist tips as the arm reaches -
   * fine for moving, useless for carrying a pen or a gripper.
   */
  holdToolOrientation: boolean;
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
}
