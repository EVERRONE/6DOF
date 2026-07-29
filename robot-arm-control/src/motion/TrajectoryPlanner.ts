// Trajectory Planner
// Plans and coordinates multi-segment trajectories through a series of waypoints

import {
  Waypoint,
  Trajectory,
  TrajectorySegment,
  TrajectoryPoint,
  PathPlannerConfig,
  SavedPath
} from './types';
import { PathInterpolator } from './PathInterpolator';
import { ForwardKinematics } from '../kinematics/ForwardKinematics';
import { Vector3 } from '../kinematics/types';

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

  constructor(config?: Partial<PathPlannerConfig>) {
    this.config = { ...DEFAULT_PLANNER_CONFIG, ...config };
    this.interpolator = new PathInterpolator();
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
    const resolvedWaypoints: Waypoint[] = [];
    const skippedWaypoints: string[] = [];
    let currentAngles = [...startAngles];

    for (const waypoint of waypoints) {
      const resolved = this.interpolator.resolveWaypointAngles(waypoint, currentAngles);
      if (!resolved) {
        // Report it rather than only logging: a silently dropped waypoint means
        // the arm runs a different path than the operator laid out.
        skippedWaypoints.push(waypoint.label || waypoint.id);
        continue;
      }
      waypointAngles.push(resolved);
      resolvedWaypoints.push(waypoint);
      currentAngles = resolved;
    }

    if (waypointAngles.length === 0) {
      return { ...this.createEmptyTrajectory(waypoints), skippedWaypoints };
    }

    // Plan segments between consecutive waypoints
    const segments: TrajectorySegment[] = [];
    let segStartAngles = [...startAngles];
    let timeOffset = 0;

    for (let i = 0; i < waypointAngles.length; i++) {
      const endAngles = waypointAngles[i];
      const waypoint = resolvedWaypoints[i];
      const speed = waypoint.speed || this.config.defaultSpeed;

      let segment: TrajectorySegment;

      if (this.config.interpolationMode === 'linear') {
        // Cartesian straight-line interpolation
        segment = this.interpolator.interpolateCartesianSpace(
          segStartAngles,
          waypoint.position,
          speed,
          this.config.defaultAcceleration,
          this.config.pointsPerSecond
        );
      } else {
        // Joint space interpolation.
        //
        // The waypoint's own feed rate is used here, falling back to the config
        // default. It used to pass config.maxJointSpeed unconditionally, so the
        // per-waypoint speed control did nothing at all in joint mode - which is
        // the default mode.
        segment = this.interpolator.interpolateJointSpace(
          segStartAngles,
          endAngles,
          Math.min(speed, this.config.maxJointSpeed),
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
      segStartAngles = endAngles;
    }

    // Compute totals
    const totalDuration = segments.reduce((sum, s) => sum + s.duration, 0);
    const totalDistance = segments.reduce((sum, s) => sum + s.distance, 0);
    const pointCount = segments.reduce((sum, s) => sum + s.points.length, 0);
    const unreachableSamples = segments.reduce(
      (sum, s) => sum + (s.unreachableSamples ?? 0),
      0
    );

    return {
      segments,
      totalDuration,
      totalDistance,
      pointCount,
      waypoints,
      unreachableSamples,
      skippedWaypoints
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
  static getTrajectoryPositions(trajectory: Trajectory): Vector3[] {
    const positions: Vector3[] = [];
    const allPoints = TrajectoryPlanner.flattenTrajectory(trajectory);

    // Sample every Nth point for performance
    const step = Math.max(1, Math.floor(allPoints.length / 200));

    for (let i = 0; i < allPoints.length; i += step) {
      const fk = ForwardKinematics.solve(allPoints[i].jointAngles);
      if (fk.success) {
        positions.push({ ...fk.endEffectorPose.position });
      }
    }

    // Always include last point
    if (allPoints.length > 0) {
      const lastFK = ForwardKinematics.solve(allPoints[allPoints.length - 1].jointAngles);
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
      version: '1.0'
    };
  }

  /**
   * Validate a saved path for import
   */
  static validateSavedPath(data: unknown): data is SavedPath {
    if (!data || typeof data !== 'object') return false;
    const obj = data as Record<string, unknown>;
    return (
      typeof obj.name === 'string' &&
      Array.isArray(obj.waypoints) &&
      obj.waypoints.length > 0 &&
      typeof obj.version === 'string'
    );
  }

  private createEmptyTrajectory(waypoints: Waypoint[]): Trajectory {
    return {
      segments: [],
      totalDuration: 0,
      totalDistance: 0,
      pointCount: 0,
      waypoints,
      unreachableSamples: 0,
      skippedWaypoints: []
    };
  }
}
