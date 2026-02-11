// Path Interpolator
// Generates trajectory points between waypoints using joint or Cartesian interpolation

import { TrajectoryPoint, TrajectorySegment, Waypoint } from './types';
import { VelocityProfile, computeSynchronizedDuration, createSynchronizedProfiles } from './VelocityProfile';
import { ForwardKinematics } from '../kinematics/ForwardKinematics';
import { InverseKinematics } from '../kinematics/InverseKinematics';
import { Vector3 } from '../kinematics/types';

/**
 * PathInterpolator
 *
 * Generates trajectory points between two waypoints using either:
 * - Joint space interpolation (smoother in joint space, curved in Cartesian)
 * - Linear Cartesian interpolation (straight line in workspace, requires IK per point)
 */
export class PathInterpolator {
  private ikSolver: InverseKinematics;

  constructor() {
    this.ikSolver = new InverseKinematics({
      maxIterations: 100,
      tolerance: 0.001,
      dampingFactor: 0.01
    });
  }

  /**
   * Interpolate between two sets of joint angles in joint space
   * Uses synchronized trapezoidal velocity profiles per joint
   */
  interpolateJointSpace(
    startAngles: number[],
    endAngles: number[],
    maxJointSpeed: number,
    maxJointAccel: number,
    pointsPerSecond: number
  ): TrajectorySegment {
    // Compute joint distances
    const jointDistances = endAngles.map((end, i) => end - startAngles[i]);

    // Find synchronized duration (all joints start/stop together)
    const duration = computeSynchronizedDuration(jointDistances, maxJointSpeed, maxJointAccel);

    if (duration < 1e-6) {
      // No movement needed
      return {
        startWaypoint: 0,
        endWaypoint: 0,
        points: [{
          time: 0,
          jointAngles: [...startAngles]
        }],
        duration: 0,
        distance: 0
      };
    }

    // Create synchronized profiles for each joint
    const profiles = createSynchronizedProfiles(jointDistances, duration, maxJointAccel);

    // Sample trajectory points
    const dt = 1.0 / pointsPerSecond;
    const points: TrajectoryPoint[] = [];

    for (let t = 0; t <= duration; t += dt) {
      const angles = startAngles.map((start, i) => {
        return start + profiles[i].getPosition(t);
      });

      const velocities = profiles.map(p => p.getVelocity(t));

      points.push({
        time: t,
        jointAngles: angles,
        velocity: velocities
      });
    }

    // Ensure final point is exact
    const lastPoint = points[points.length - 1];
    if (lastPoint && Math.abs(lastPoint.time - duration) > dt * 0.5) {
      points.push({
        time: duration,
        jointAngles: [...endAngles],
        velocity: Array(6).fill(0)
      });
    } else if (lastPoint) {
      // Snap the last point to exact end angles
      lastPoint.jointAngles = [...endAngles];
      lastPoint.velocity = Array(6).fill(0);
    }

    // Compute Cartesian distance between start and end
    const fkStart = ForwardKinematics.solve(startAngles);
    const fkEnd = ForwardKinematics.solve(endAngles);
    let distance = 0;
    if (fkStart.success && fkEnd.success) {
      const dx = fkEnd.endEffectorPose.position.x - fkStart.endEffectorPose.position.x;
      const dy = fkEnd.endEffectorPose.position.y - fkStart.endEffectorPose.position.y;
      const dz = fkEnd.endEffectorPose.position.z - fkStart.endEffectorPose.position.z;
      distance = Math.sqrt(dx * dx + dy * dy + dz * dz);
    }

    return {
      startWaypoint: 0,
      endWaypoint: 0,
      points,
      duration,
      distance
    };
  }

  /**
   * Interpolate between two positions in Cartesian space (straight line)
   * Uses IK at each sample point to convert to joint angles
   */
  interpolateCartesianSpace(
    startAngles: number[],
    endPosition: Vector3,
    speed: number,         // mm/s
    acceleration: number,  // mm/s²
    pointsPerSecond: number
  ): TrajectorySegment {
    // Get start position from FK
    const fkStart = ForwardKinematics.solve(startAngles);
    if (!fkStart.success) {
      return this.createErrorSegment(startAngles);
    }

    const startPos = fkStart.endEffectorPose.position;

    // Compute Cartesian distance (meters)
    const dx = endPosition.x - startPos.x;
    const dy = endPosition.y - startPos.y;
    const dz = endPosition.z - startPos.z;
    const distance = Math.sqrt(dx * dx + dy * dy + dz * dz);

    if (distance < 1e-6) {
      return {
        startWaypoint: 0,
        endWaypoint: 0,
        points: [{ time: 0, jointAngles: [...startAngles] }],
        duration: 0,
        distance: 0
      };
    }

    // Convert speed from mm/s to m/s for profile computation
    const speedMs = speed / 1000;
    const accelMs = acceleration / 1000;

    // Create velocity profile for the Cartesian path
    const profile = new VelocityProfile(distance, speedMs, accelMs);
    const duration = profile.getDuration();

    // Sample along the straight line
    const dt = 1.0 / pointsPerSecond;
    const points: TrajectoryPoint[] = [];
    let currentAngles = [...startAngles];

    for (let t = 0; t <= duration; t += dt) {
      const s = profile.getProgress(t); // 0 to 1

      // Interpolate Cartesian position
      const pos: Vector3 = {
        x: startPos.x + dx * s,
        y: startPos.y + dy * s,
        z: startPos.z + dz * s
      };

      // Solve IK for this position
      const ikResult = this.ikSolver.solvePosition(pos, currentAngles);

      if (ikResult.success) {
        currentAngles = ikResult.jointAngles;
        points.push({
          time: t,
          jointAngles: [...ikResult.jointAngles]
        });
      } else {
        // IK failed - use last known good angles
        points.push({
          time: t,
          jointAngles: [...currentAngles]
        });
      }
    }

    // Ensure final point
    const lastPoint = points[points.length - 1];
    if (lastPoint && Math.abs(lastPoint.time - duration) > dt * 0.5) {
      const ikFinal = this.ikSolver.solvePosition(endPosition, currentAngles);
      points.push({
        time: duration,
        jointAngles: ikFinal.success ? [...ikFinal.jointAngles] : [...currentAngles],
        velocity: Array(6).fill(0)
      });
    }

    return {
      startWaypoint: 0,
      endWaypoint: 0,
      points,
      duration,
      distance
    };
  }

  /**
   * Resolve a waypoint to joint angles.
   * If the waypoint already has jointAngles, use those.
   * Otherwise, solve IK from the waypoint position.
   */
  resolveWaypointAngles(waypoint: Waypoint, currentAngles: number[]): number[] | null {
    if (waypoint.jointAngles) {
      return [...waypoint.jointAngles];
    }

    const ikResult = this.ikSolver.solvePosition(waypoint.position, currentAngles);
    if (ikResult.success) {
      return ikResult.jointAngles;
    }

    return null;
  }

  private createErrorSegment(startAngles: number[]): TrajectorySegment {
    return {
      startWaypoint: 0,
      endWaypoint: 0,
      points: [{ time: 0, jointAngles: [...startAngles] }],
      duration: 0,
      distance: 0
    };
  }
}
