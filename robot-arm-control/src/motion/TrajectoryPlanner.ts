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
import { pointOnCircle } from './Shapes';
import { blendPolyline } from './CornerBlend';
import { ForwardKinematics } from '../kinematics/ForwardKinematics';
import { Rotation3, Vector3 } from '../kinematics/types';
import { matrixToQuat, matrixToRpy, quatToMatrix, rpyToMatrix, slerp } from '../kinematics/linalg';
import {
  WorkObject,
  frameById,
  pointToBase,
  rotationToBase
} from '../kinematics/workObject';

/**
 * A waypoint's position and orientation in base coordinates.
 *
 * Everything downstream of planning works in base, so this is the one place work
 * objects are resolved. Doing it per consumer instead would mean every stage had
 * to remember, and the one that forgot would drive the arm to the right numbers
 * in the wrong frame.
 */
export function resolveWaypoint(waypoint: Waypoint, objects: WorkObject[]): Waypoint {
  if (!waypoint.frame) return waypoint;

  const frame = frameById(objects, waypoint.frame);
  return {
    ...waypoint,
    position: pointToBase(frame, waypoint.position),
    orientation: waypoint.orientation
      ? rotationToBase(frame, waypoint.orientation)
      : undefined
  };
}

/** Slerp between two attitudes, tolerating either being absent. */
function blendRotation(
  a: Rotation3 | undefined,
  b: Rotation3 | undefined,
  t: number
): Rotation3 | undefined {
  if (!a) return b;
  if (!b) return a;
  return matrixToRpy(
    quatToMatrix(slerp(matrixToQuat(rpyToMatrix(a)), matrixToQuat(rpyToMatrix(b)), t))
  );
}

/**
 * Default planner configuration
 */
export const DEFAULT_PLANNER_CONFIG: PathPlannerConfig = {
  interpolationMode: 'joint',
  defaultSpeed: 50,           // 50 mm/s
  defaultAcceleration: 100,   // 100 mm/s²
  pointsPerSecond: 10,        // 10 Hz trajectory sampling
  loopCount: 0,
  // Full speed, meaning whatever each joint's own measured limit is. The pair of
  // absolute caps this replaces - 60 deg/s and 120 deg/s^2 - were a second set
  // of limits alongside the model's, and went stale when the arm was tuned: they
  // held J6 to 17% of its measured speed and J5 to 30%.
  speedScale: 1,
  // Square corners by default, because cutting one changes where the arm goes
  // and that should be asked for rather than assumed.
  blendRadius: 0,
  holdToolOrientation: false
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
  planTrajectory(
    waypoints: Waypoint[],
    startAngles: number[],
    /**
     * Work objects the waypoints may name. Resolved once here, at the top, so
     * that everything below this line is in base coordinates and no later stage
     * has to know work objects exist.
     */
    workObjects: WorkObject[] = []
  ): Trajectory {
    if (waypoints.length === 0) {
      return this.createEmptyTrajectory(waypoints);
    }

    // The original list is kept for the returned trajectory, because that is
    // what the UI edits and what gets saved; only the planning below uses the
    // resolved copies.
    const taught = waypoints;
    waypoints = waypoints.map(w => resolveWaypoint(w, workObjects));

    // Resolve all waypoints to joint angles
    const waypointAngles: number[][] = [];
    const resolvedWaypoints: Waypoint[] = [];
    const skippedWaypoints: string[] = [];
    let currentAngles = [...startAngles];

    // The orientation to hold across the whole path, when asked for: the one
    // the arm starts in. Taken once, not per waypoint, so residuals cannot
    // accumulate into the tool drifting round over a long path.
    const heldOrientation = this.config.holdToolOrientation
      ? ForwardKinematics.solve(startAngles).endEffectorPose.rotation
      : undefined;

    for (const waypoint of waypoints) {
      // A figure is entered at its start point, not at its centre - the centre
      // is where it is described from, and is usually not on the path at all.
      const entry = waypoint.shape
        ? { ...waypoint, position: pointOnCircle(waypoint.shape, waypoint.position, 0) }
        : waypoint;
      const effective = heldOrientation
        ? { ...entry, orientation: heldOrientation }
        : entry;

      const resolved = this.interpolator.resolveWaypointAngles(effective, currentAngles);
      if (!resolved) {
        // Report it rather than only logging: a silently dropped waypoint means
        // the arm runs a different path than the operator laid out.
        skippedWaypoints.push(waypoint.label || waypoint.id);
        continue;
      }
      waypointAngles.push(resolved);
      resolvedWaypoints.push(effective);
      currentAngles = resolved;
    }

    if (waypointAngles.length === 0) {
      return { ...this.createEmptyTrajectory(taught), skippedWaypoints };
    }

    // A linear path whose corners are to be rounded is planned as a whole rather
    // than waypoint by waypoint: a blend arc straddles a waypoint, taking a bite
    // out of the legs either side, so neither leg can be planned without knowing
    // about the other.
    if (
      this.config.interpolationMode === 'linear' &&
      this.config.blendRadius > 1e-9 &&
      resolvedWaypoints.length >= 2 &&
      !resolvedWaypoints.some(w => w.shape)
    ) {
      return this.planBlended(taught, resolvedWaypoints, startAngles, heldOrientation);
    }

    // Plan segments between consecutive waypoints
    const segments: TrajectorySegment[] = [];
    let segStartAngles = [...startAngles];
    let timeOffset = 0;

    // The attitude the previous waypoint left the tool in, so a segment turns
    // from it rather than starting in the one it is heading for. Seeded from the
    // pose the arm is actually in.
    let carriedOrientation: Rotation3 | undefined =
      heldOrientation ??
      (resolvedWaypoints.some(w => w.orientation)
        ? ForwardKinematics.solve(startAngles).endEffectorPose.rotation
        : undefined);

    for (let i = 0; i < waypointAngles.length; i++) {
      const endAngles = waypointAngles[i];
      const waypoint = resolvedWaypoints[i];
      const speed = waypoint.speed || this.config.defaultSpeed;

      let segment: TrajectorySegment;

      if (this.config.interpolationMode === 'linear') {
        // Cartesian straight-line interpolation, turning the tool from the
        // attitude the previous waypoint left it in to this one's. Held constant
        // when they agree, which is what holdToolOrientation arranges.
        segment = this.interpolator.interpolateCartesianSpace(
          segStartAngles,
          waypoint.position,
          speed * this.config.speedScale,
          this.config.defaultAcceleration * this.config.speedScale,
          this.config.pointsPerSecond,
          carriedOrientation ?? waypoint.orientation,
          waypoint.orientation
        );
        carriedOrientation = waypoint.orientation ?? carriedOrientation;
      } else if (waypoint.shape) {
        // Getting to a figure is an ordinary move; the figure itself is not.
        // Handled below, after this approach segment.
        segment = this.interpolator.interpolateJointSpace(
          segStartAngles,
          endAngles,
          Infinity,
          Infinity,
          this.config.pointsPerSecond,
          this.config.speedScale
        );
      } else {
        // Joint space interpolation.
        //
        // The waypoint's own feed rate is a further cap on top of the per-joint
        // limits, not a replacement for them. It used to pass config.maxJointSpeed
        // unconditionally, so the per-waypoint control did nothing at all in
        // joint mode - which is the default mode - and config.maxJointSpeed then
        // capped every joint at one figure that could not describe six of them.
        segment = this.interpolator.interpolateJointSpace(
          segStartAngles,
          endAngles,
          speed,
          Infinity,
          this.config.pointsPerSecond,
          this.config.speedScale
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

      // Then the figure itself, sampled along the true arc. This is the whole
      // point of holding a shape as one waypoint: the density is chosen here,
      // by the motion, rather than fixed when the circle was described.
      if (waypoint.shape) {
        const centre = waypoints[i].position;
        const arc = this.interpolator.interpolateArc(
          waypointAngles[i],
          waypoint.shape,
          centre,
          speed,
          this.config.defaultAcceleration,
          this.config.pointsPerSecond,
          waypoint.orientation
        );

        arc.startWaypoint = i;
        arc.endWaypoint = i;

        const arcOffset = timeOffset;
        arc.points = arc.points.map(p => ({ ...p, time: p.time + arcOffset }));

        segments.push(arc);
        timeOffset += arc.duration;

        if (arc.points.length > 0) {
          segStartAngles = [...arc.points[arc.points.length - 1].jointAngles];
          continue;
        }
      }
      segStartAngles = endAngles;
    }

    return this.summarise(segments, taught, skippedWaypoints);
  }

  /**
   * Plan a linear path whose corners are replaced by tangent arcs.
   *
   * Kept apart from the waypoint-by-waypoint path above because the unit of
   * planning is different. There, one segment runs from one waypoint to the
   * next. Here the polyline is blended first and the resulting pieces - shortened
   * legs and the arcs between them - are what get sampled, so a segment no longer
   * corresponds to a waypoint at all. The arm never reaches the corner waypoints;
   * it passes within `cutBy` of them.
   *
   * Orientation is carried along the whole path by distance rather than per
   * segment: a blend arc straddles a waypoint, so asking "which waypoint's
   * attitude does this piece end in" has no answer. Each waypoint is placed at
   * its own distance along the blended path, and the attitude at any point is
   * the slerp between the two it lies between.
   */
  private planBlended(
    waypoints: Waypoint[],
    resolved: Waypoint[],
    startAngles: number[],
    heldOrientation?: Rotation3
  ): Trajectory {
    const startPos = ForwardKinematics.position(startAngles);
    const corners = [startPos, ...resolved.map(w => w.position)];
    const pieces = blendPolyline(corners, this.config.blendRadius);

    if (pieces.length === 0) {
      return this.createEmptyTrajectory(waypoints);
    }

    // Where each corner sits along the blended path. A blended corner is placed
    // at the middle of the arc that replaced it, which is the closest the path
    // comes to it.
    const total = pieces.reduce((sum, p) => sum + p.length, 0);
    const cornerDistance = new Array(corners.length).fill(0);
    let walked = 0;
    for (const piece of pieces) {
      if (piece.kind === 'arc') {
        cornerDistance[piece.cornerIndex] = walked + piece.length / 2;
      }
      walked += piece.length;
    }
    cornerDistance[0] = 0;
    cornerDistance[corners.length - 1] = total;
    // Any corner that was not blended - too straight, too tight, or squeezed out
    // by its neighbours - still needs a place on the line for the orientation
    // schedule. Between the two either side of it is the honest answer.
    for (let i = 1; i < corners.length - 1; i++) {
      if (cornerDistance[i] === 0) {
        cornerDistance[i] = (cornerDistance[i - 1] + cornerDistance[i + 1] || total) / 2;
      }
    }

    const attitudes: Array<Rotation3 | undefined> = [
      heldOrientation ??
        (resolved.some(w => w.orientation)
          ? ForwardKinematics.solve(startAngles).endEffectorPose.rotation
          : undefined),
      ...resolved.map(w => heldOrientation ?? w.orientation)
    ];

    /** The attitude wanted at a given distance along the blended path. */
    const attitudeAt = (d: number): Rotation3 | undefined => {
      if (!attitudes.some(Boolean)) return undefined;
      for (let i = 1; i < cornerDistance.length; i++) {
        if (d > cornerDistance[i] && i < cornerDistance.length - 1) continue;
        const span = cornerDistance[i] - cornerDistance[i - 1];
        const t = span > 1e-9 ? (d - cornerDistance[i - 1]) / span : 1;
        return blendRotation(attitudes[i - 1], attitudes[i], t);
      }
      return attitudes[attitudes.length - 1];
    };

    const segments: TrajectorySegment[] = [];
    let segStartAngles = [...startAngles];
    let timeOffset = 0;
    let travelled = 0;
    let cornerCursor = 1;

    for (const piece of pieces) {
      const speed = (resolved[Math.min(cornerCursor - 1, resolved.length - 1)]?.speed ||
        this.config.defaultSpeed) * this.config.speedScale;

      const segment = this.interpolator.interpolatePiece(
        segStartAngles,
        piece,
        speed,
        this.config.defaultAcceleration * this.config.speedScale,
        this.config.pointsPerSecond,
        attitudeAt(travelled),
        attitudeAt(travelled + piece.length)
      );

      if (piece.kind === 'arc') cornerCursor = Math.min(resolved.length, piece.cornerIndex);
      segment.startWaypoint = Math.max(0, cornerCursor - 1);
      segment.endWaypoint = Math.min(resolved.length - 1, cornerCursor);

      const offset = timeOffset;
      segment.points = segment.points.map(p => ({ ...p, time: p.time + offset }));

      segments.push(segment);
      timeOffset += segment.duration;
      travelled += piece.length;

      if (segment.points.length > 0) {
        segStartAngles = [...segment.points[segment.points.length - 1].jointAngles];
      }
    }

    return this.summarise(segments, waypoints, []);
  }

  /** Roll a list of planned segments up into a trajectory. */
  private summarise(
    segments: TrajectorySegment[],
    waypoints: Waypoint[],
    skippedWaypoints: string[]
  ): Trajectory {
    // The first jump anywhere in the path. Segments have carried this since
    // linear moves learned to detect it, and nothing read it: a path could run
    // through a 44 degree wrist flip that a single move to the same place
    // refuses.
    let discontinuity: Trajectory['discontinuity'] = null;
    for (let i = 0; i < segments.length && !discontinuity; i++) {
      const jump = segments[i].discontinuity;
      if (jump) discontinuity = { ...jump, segment: i };
    }

    return {
      segments,
      totalDuration: segments.reduce((sum, s) => sum + s.duration, 0),
      totalDistance: segments.reduce((sum, s) => sum + s.distance, 0),
      pointCount: segments.reduce((sum, s) => sum + s.points.length, 0),
      waypoints,
      unreachableSamples: segments.reduce((sum, s) => sum + (s.unreachableSamples ?? 0), 0),
      skippedWaypoints,
      discontinuity,
      // Only the first segment's. Every later segment starts from the pose the
      // previous one ended in, so its solve is already seeded there and cannot
      // want a different configuration; the first is the only one solved against
      // a pose the arm is actually in.
      reconfiguration: segments[0]?.reconfiguration ?? null
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
   * For each point `flattenTrajectory` produces, the waypoint the arm has just
   * arrived at, or null while it is still travelling.
   *
   * Kept beside flattenTrajectory and built by the same walk, because the two
   * have to agree on which points exist. Reconstructing the mapping from segment
   * lengths does not: the seam between two segments is dropped as a duplicate
   * timestamp, so every index after the first seam is off by one, and the
   * arrival lands on the wrong point - or on none.
   *
   * When the seam itself is a segment's last point, the marker moves back onto
   * the entry that survived rather than being lost with it.
   */
  static waypointArrivals(trajectory: Trajectory): Array<Waypoint | null> {
    const arrivals: Array<Waypoint | null> = [];
    let lastTime: number | null = null;

    for (const segment of trajectory.segments) {
      const endsHere = trajectory.waypoints[segment.endWaypoint] ?? null;

      for (let k = 0; k < segment.points.length; k++) {
        const point = segment.points[k];
        const isLast = k === segment.points.length - 1;

        if (lastTime !== null && Math.abs(point.time - lastTime) < 0.001) {
          if (isLast && arrivals.length > 0) arrivals[arrivals.length - 1] = endsHere;
          continue;
        }

        lastTime = point.time;
        arrivals.push(isLast ? endsHere : null);
      }
    }

    return arrivals;
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
