// Trajectory Planner
// Plans and coordinates multi-segment trajectories through a series of waypoints

import {
  CartesianInterpolationResult,
  Waypoint,
  Trajectory,
  TrajectorySegment,
  TrajectoryPoint,
  PathPlannerConfig,
  SavedPath
} from './types';
import { PathInterpolator } from './PathInterpolator';
import { ForwardKinematics } from '../kinematics/ForwardKinematics';
import { IKEngineMode, IKSolveOptions, Rotation3, Vector3 } from '../kinematics/types';
import { logicalToUrdfAngles } from '../kinematics/angleMapping';

/**
 * Default planner configuration
 */
export const DEFAULT_PLANNER_CONFIG: PathPlannerConfig = {
  interpolationMode: 'joint',
  defaultSpeed: 50,           // 50 mm/s
  defaultAcceleration: 100,   // 100 mm/s²
  maxJointSpeed: 60,          // 60 deg/s
  maxJointAcceleration: 120,  // 120 deg/s²
  pointsPerSecond: 10,        // 10 Hz trajectory sampling
  loopCount: 0
};

/**
 * TrajectoryPlanner
 *
 * Orchestrates multi-waypoint trajectory planning:
 * 1. Resolves each waypoint to joint angles (via IK if needed)
 * 2. Plans trajectory segments between consecutive waypoints
 * 3. Coordinates timing across segments
 * 4. Returns a complete trajectory ready for execution
 */
export class TrajectoryPlanner {
  private interpolator: PathInterpolator;
  private config: PathPlannerConfig;
  private urdfOffsetsDeg: number[];

  constructor(config?: Partial<PathPlannerConfig>, urdfOffsetsDeg?: number[]) {
    this.config = { ...DEFAULT_PLANNER_CONFIG, ...config };
    this.urdfOffsetsDeg = urdfOffsetsDeg && urdfOffsetsDeg.length === 6
      ? [...urdfOffsetsDeg]
      : [0, 0, 0, 0, 0, 0];
    this.interpolator = new PathInterpolator(this.urdfOffsetsDeg);
  }

  /**
   * Plan a complete trajectory through a series of waypoints
   *
   * @param waypoints - Ordered list of waypoints to visit
   * @param startAngles - Current joint angles (degrees)
   * @returns Complete trajectory with all segments
   */
  planTrajectory(waypoints: Waypoint[], startAngles: number[]): Trajectory {
    if (waypoints.length === 0) {
      return this.createEmptyTrajectory(waypoints);
    }

    // Resolve all waypoints to joint angles
    const waypointAngles: number[][] = [];
    let currentAngles = [...startAngles];

    for (const waypoint of waypoints) {
      const resolved = this.interpolator.resolveWaypointAngles(waypoint, currentAngles);
      if (!resolved) {
        console.warn(`Failed to resolve waypoint: ${waypoint.label || waypoint.id}`);
        // Skip unreachable waypoints
        continue;
      }
      waypointAngles.push(resolved);
      currentAngles = resolved;
    }

    if (waypointAngles.length === 0) {
      return this.createEmptyTrajectory(waypoints);
    }

    // Plan segments between consecutive waypoints
    const segments: TrajectorySegment[] = [];
    let segStartAngles = [...startAngles];
    let timeOffset = 0;

    for (let i = 0; i < waypointAngles.length; i++) {
      const endAngles = waypointAngles[i];
      const waypoint = waypoints[i];
      const speed = waypoint.speed || this.config.defaultSpeed;

      let segment: TrajectorySegment;

      if (this.config.interpolationMode === 'linear') {
        // Cartesian straight-line interpolation
        const interpolation = this.interpolator.interpolateCartesianSpace(
          segStartAngles,
          waypoint.position,
          speed,
          this.config.defaultAcceleration,
          this.config.pointsPerSecond,
          waypoint.orientation,
          undefined,
          'hybrid_constrained_v2'
        );
        if (!interpolation.success) {
          console.warn(
            `Failed Cartesian interpolation for waypoint ${waypoint.label || waypoint.id}: ${interpolation.error}`
          );
          continue;
        }
        segment = interpolation.segment;
      } else {
        // Joint space interpolation
        segment = this.interpolator.interpolateJointSpace(
          segStartAngles,
          endAngles,
          this.config.maxJointSpeed,
          this.config.maxJointAcceleration,
          this.config.pointsPerSecond
        );
      }

      // Update segment indices
      segment.startWaypoint = i === 0 ? -1 : i - 1;  // -1 = start position
      segment.endWaypoint = i;

      // Offset times for global timeline
      const offset = timeOffset;
      segment.points = segment.points.map(p => ({
        ...p,
        time: p.time + offset
      }));

      segments.push(segment);
      timeOffset += segment.duration;
      segStartAngles = segment.points[segment.points.length - 1]?.jointAngles || endAngles;
    }

    // Compute totals
    const totalDuration = segments.reduce((sum, s) => sum + s.duration, 0);
    const totalDistance = segments.reduce((sum, s) => sum + s.distance, 0);
    const pointCount = segments.reduce((sum, s) => sum + s.points.length, 0);

    return {
      segments,
      totalDuration,
      totalDistance,
      pointCount,
      waypoints
    };
  }

  /**
   * Get all trajectory points flattened into a single array
   * with global timestamps
   */
  static flattenTrajectory(trajectory: Trajectory): TrajectoryPoint[] {
    const allPoints: TrajectoryPoint[] = [];

    for (const segment of trajectory.segments) {
      for (const point of segment.points) {
        // Avoid duplicate timestamps at segment boundaries
        if (allPoints.length > 0) {
          const lastTime = allPoints[allPoints.length - 1].time;
          if (Math.abs(point.time - lastTime) < 0.001) {
            continue;
          }
        }
        allPoints.push(point);
      }
    }

    return allPoints;
  }

  /**
   * Get the Cartesian positions along the trajectory for 3D visualization
   */
  static getTrajectoryPositions(trajectory: Trajectory, urdfOffsetsDeg?: number[]): Vector3[] {
    const positions: Vector3[] = [];
    const allPoints = TrajectoryPlanner.flattenTrajectory(trajectory);
    const offsets = urdfOffsetsDeg && urdfOffsetsDeg.length === 6
      ? urdfOffsetsDeg
      : [0, 0, 0, 0, 0, 0];

    // Sample every Nth point for performance
    const step = Math.max(1, Math.floor(allPoints.length / 200));

    for (let i = 0; i < allPoints.length; i += step) {
      const fk = ForwardKinematics.solve(logicalToUrdfAngles(allPoints[i].jointAngles, offsets));
      if (fk.success) {
        positions.push({ ...fk.endEffectorPose.position });
      }
    }

    // Always include last point
    if (allPoints.length > 0) {
      const lastFK = ForwardKinematics.solve(
        logicalToUrdfAngles(allPoints[allPoints.length - 1].jointAngles, offsets)
      );
      if (lastFK.success) {
        positions.push({ ...lastFK.endEffectorPose.position });
      }
    }

    return positions;
  }

  /**
   * Update planner configuration
   */
  updateConfig(config: Partial<PathPlannerConfig>): void {
    this.config = { ...this.config, ...config };
  }

  setUrdfOffsets(urdfOffsetsDeg: number[]): void {
    this.urdfOffsetsDeg = urdfOffsetsDeg && urdfOffsetsDeg.length === 6
      ? [...urdfOffsetsDeg]
      : [0, 0, 0, 0, 0, 0];
    this.interpolator.setUrdfOffsets(this.urdfOffsetsDeg);
  }

  setJointLimitsDeg(minDeg: number[], maxDeg: number[]): void {
    this.interpolator.setJointLimitsDeg(minDeg, maxDeg);
  }

  planPoseLockedCartesianMove(
    startAngles: number[],
    targetPosition: Vector3,
    targetOrientation?: Rotation3,
    options?: {
      speedMmS?: number;
      accelerationMmS2?: number;
      pointsPerSecond?: number;
      ikOptions?: Partial<IKSolveOptions>;
      ikEngineMode?: IKEngineMode;
      strictFinalGate?: boolean;
      deadlineMs?: number;
    }
  ): CartesianInterpolationResult {
    const speed = options?.speedMmS ?? 25;
    const acceleration = options?.accelerationMmS2 ?? 80;
    const pointsPerSecond = options?.pointsPerSecond ?? 40;
    const ikEngineMode = options?.ikEngineMode ?? 'hybrid_constrained_v2';

    return this.interpolator.interpolateCartesianSpace(
      startAngles,
      targetPosition,
      speed,
      acceleration,
      pointsPerSecond,
      targetOrientation,
      options?.ikOptions,
      ikEngineMode,
      options?.strictFinalGate ?? true,
      options?.deadlineMs
    );
  }

  /**
   * Get current configuration
   */
  getConfig(): PathPlannerConfig {
    return { ...this.config };
  }

  /**
   * Export waypoints and config as a saveable path
   */
  static exportPath(
    name: string,
    waypoints: Waypoint[],
    config: PathPlannerConfig,
    description?: string
  ): SavedPath {
    return {
      name,
      description,
      waypoints,
      config,
      createdAt: new Date().toISOString(),
      version: '1.0',
      kinematicsFrame: 'urdf_chain_v1'
    };
  }

  /**
   * Validate a saved path for import
   */
  static validateSavedPath(data: unknown): data is SavedPath {
    if (!data || typeof data !== 'object') return false;
    const obj = data as Record<string, unknown>;
    const frame = obj.kinematicsFrame;
    const frameValid = (
      frame === undefined ||
      frame === 'urdf_chain_v1' ||
      frame === 'legacy_dh_v1'
    );

    return (
      typeof obj.name === 'string' &&
      Array.isArray(obj.waypoints) &&
      obj.waypoints.length > 0 &&
      typeof obj.version === 'string' &&
      frameValid
    );
  }

  private createEmptyTrajectory(waypoints: Waypoint[]): Trajectory {
    return {
      segments: [],
      totalDuration: 0,
      totalDistance: 0,
      pointCount: 0,
      waypoints
    };
  }
}
