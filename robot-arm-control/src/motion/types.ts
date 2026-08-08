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
  /**
   * Joint angles the arm has to be in before this segment can start, when they
   * are not the ones it is in now.
   *
   * The wrist has to be in the right configuration to run some lines smoothly,
   * and from the parked pose it is not. Rather than discovering that halfway
   * along and flipping 44 degrees between two samples 2 mm apart, the segment is
   * solved from its far end - where the wrist is well conditioned - and the
   * configuration it needs at the near end is reported here.
   *
   * Free at the tool. At the singularity J4 and J6 turn about the same axis, so
   * counter-rotating them is exactly null-space motion: measured over the whole
   * 90 degree reconfiguration the tool moves 0.0 mm and tilts 0 degrees. It is a
   * wrist turning in place before the move starts, not a detour.
   */
  reconfiguration?: number[] | null;
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
  /**
   * The first place the joint path jumps, across the whole trajectory, or null.
   *
   * Segments have carried this since linear moves learned to detect it, but the
   * trajectory did not surface it and execution did not read it - so a path could
   * run straight through a 44 degree wrist flip that a single Cartesian move to
   * the same place refuses.
   */
  discontinuity?:
    | (NonNullable<TrajectorySegment['discontinuity']> & { segment: number })
    | null;
  /**
   * Joint angles the arm has to be turned to before the path can start, or null.
   *
   * Only the first segment's, because every later one begins where the previous
   * ended. A segment solved from its far end can want the wrist in a
   * configuration the arm is not in - 90 degrees away, from the parked pose -
   * and the first point of the path would otherwise be that jump, unannounced.
   */
  reconfiguration?: number[] | null;
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
  pointsPerSecond: number;        // trajectory sampling rate (Hz)
  loopCount: number;              // 0 = no loop, >0 = repeat N times

  /**
   * Global override on how fast the path runs, as a fraction of what each joint
   * can do. 1 is full speed.
   *
   * Replaces a pair of absolute caps - `maxJointSpeed: 60` and
   * `maxJointAcceleration: 120` - which were a second set of limits alongside
   * the per-joint ones in the model, and went stale the moment the arm was
   * tuned. They held J6 to 17% of its measured speed and J5 to 30%, silently,
   * because a single figure cannot describe six joints that differ by 4x.
   *
   * A scale cannot go stale: it is relative to whatever the joints can do today.
   * It is also what an operator actually wants at the panel - "run this at half
   * speed while I watch it" - which is why every industrial controller has one.
   */
  speedScale: number;

  /**
   * Radius of the arc that replaces a corner between two linear segments, in
   * metres. Zero runs the corners square.
   *
   * A path through a sharp corner has to stop there: carrying speed round it
   * would need a step change in a joint's velocity, so the firmware's junction
   * rule takes the speed to zero. Two linear segments meeting at a right angle
   * cost a full stop, every lap. Replacing the corner with an arc tangent to
   * both segments removes the discontinuity, and the arm runs through.
   *
   * The trade is that the path no longer passes through the waypoint - it cuts
   * the corner by up to the radius. Named for what it is, rather than
   * "zone" (ABB) or "CNT" (Fanuc), but it is the same idea.
   */
  blendRadius: number;

  /**
   * Hold the tool's orientation for the whole path.
   *
   * The orientation held is the one the arm starts the path in. Without it a
   * path solves for position alone and the wrist tips as the arm reaches -
   * fine for moving, useless for carrying a pen or a gripper.
   *
   * Orthogonal to per-waypoint orientations: this fills every waypoint with the
   * starting one, so the interpolation below has nothing to do. Waypoints that
   * carry their own orientations are blended between instead.
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
