// Path Interpolator
//
// Turns a pair of waypoints into a series of intermediate joint poses for the
// firmware to run. It produces *geometry*, not timing: the firmware owns the
// velocity profile, so what matters here is that the points lie on the intended
// path. Times are still attached, but only for progress display and duration
// estimates.

import { TrajectoryPoint, TrajectorySegment, Waypoint } from './types';
import { VelocityProfile } from './VelocityProfile';
import { ForwardKinematics } from '../kinematics/ForwardKinematics';
import { InverseKinematics } from '../kinematics/InverseKinematics';
import {
  JOINT_MAX_ACCEL_DEG_S2,
  JOINT_MAX_SPEED_DEG_S,
  NUM_JOINTS
} from '../kinematics/robotModel';
import { Rotation3, Vector3 } from '../kinematics/types';

/**
 * Longest straight chord allowed between two samples of a Cartesian move, in
 * metres.
 *
 * A straight line in Cartesian space is a curve in joint space, so the arm bows
 * off the line between samples. Bounding the chord bounds that deviation, which
 * is the whole point of choosing linear mode over joint mode.
 */
const MAX_CARTESIAN_CHORD_M = 0.002; // 2 mm

/**
 * Hard ceiling on samples for one linear segment.
 *
 * The reachable workspace is about 440 mm across, so at the chord limit a
 * legitimate move needs at most a few hundred samples. This only bites on a
 * nonsensical target, where it stops the sample count scaling with the distance
 * to somewhere the arm can never go.
 */
const MAX_CARTESIAN_SAMPLES = 1000;

/**
 * Consecutive IK failures after which sampling stops.
 *
 * Once the line has left the reachable set it rarely comes back, and a failing
 * solve is the expensive case - it tries every seed before giving up. Without
 * this bound a mistyped coordinate cost about a minute of solid computation.
 */
const MAX_CONSECUTIVE_IK_FAILURES = 8;

/**
 * PathInterpolator
 *
 * Two modes:
 *
 *  - Joint space: a straight line in joint space. Every joint covers the same
 *    fraction of its travel at the same moment, which is what makes the motion
 *    predictable. Curved in Cartesian space.
 *
 *  - Linear: a straight line in Cartesian space, sampled closely enough to hold
 *    the arm on it, with IK at every sample.
 */
export class PathInterpolator {
  private ikSolver: InverseKinematics;

  constructor() {
    this.ikSolver = new InverseKinematics();
  }

  /**
   * Interpolate between two joint poses along a straight line in joint space.
   *
   * A single normalised profile s(t) in [0, 1] drives every joint, so joint i is
   * always at `start[i] + distance[i] * s`. That is what keeps the path straight.
   *
   * The previous version gave each joint its own trapezoid with a shared
   * duration but its own peak velocity. Those profiles have different shapes, so
   * the joints did not progress in step and the path bowed: on a move of 40
   * degrees of J2 against 5 degrees of J3, J3 was 11% ahead of J2 part way
   * through. The intermediate points then were not collinear, which also cost
   * speed at every one of them, because the firmware slows for a corner.
   *
   * @param maxJointSpeed requested speed cap, deg/s, further limited per joint
   * @param maxJointAccel requested acceleration cap, deg/s^2, likewise
   * @param pointsPerSecond sampling density. In this mode it affects only
   *        progress granularity and the 3D preview - the samples are collinear,
   *        so spacing cannot change the path.
   */
  interpolateJointSpace(
    startAngles: number[],
    endAngles: number[],
    maxJointSpeed: number,
    maxJointAccel: number,
    pointsPerSecond: number
  ): TrajectorySegment {
    const jointDistances = endAngles.map((end, i) => end - startAngles[i]);
    const largest = Math.max(...jointDistances.map(Math.abs));

    if (largest < 1e-9) {
      return emptySegment(startAngles);
    }

    // Convert the joint limits into limits on the path parameter s. Joint i moves
    // at |distance[i]| * sDot, so the binding joint is whichever gives the
    // smallest allowance.
    let sMaxVel = Infinity;
    let sMaxAccel = Infinity;

    for (let i = 0; i < NUM_JOINTS; i++) {
      const travel = Math.abs(jointDistances[i]);
      if (travel < 1e-9) continue;

      const speedCap = Math.min(JOINT_MAX_SPEED_DEG_S[i], maxJointSpeed);
      const accelCap = Math.min(JOINT_MAX_ACCEL_DEG_S2[i], maxJointAccel);

      sMaxVel = Math.min(sMaxVel, speedCap / travel);
      sMaxAccel = Math.min(sMaxAccel, accelCap / travel);
    }

    if (!Number.isFinite(sMaxVel) || sMaxVel <= 0 || !Number.isFinite(sMaxAccel) || sMaxAccel <= 0) {
      return emptySegment(startAngles);
    }

    const profile = new VelocityProfile(1, sMaxVel, sMaxAccel);
    const duration = profile.getDuration();

    const dt = 1.0 / Math.max(1, pointsPerSecond);
    const points: TrajectoryPoint[] = [];

    for (let t = 0; t <= duration; t += dt) {
      const s = profile.getPosition(t);
      const sDot = profile.getVelocity(t);

      points.push({
        time: t,
        jointAngles: jointDistances.map((d, i) => startAngles[i] + d * s),
        velocity: jointDistances.map(d => d * sDot)
      });
    }

    appendExactEnd(points, duration, endAngles, dt);

    return {
      startWaypoint: 0,
      endWaypoint: 0,
      points,
      duration,
      distance: cartesianDistance(startAngles, endAngles)
    };
  }

  /**
   * Interpolate along a straight line in Cartesian space, solving IK at each
   * sample.
   *
   * Samples are spaced by arc length rather than by time, so the spacing - and
   * therefore how far the arm may bow off the line - is uniform along the whole
   * move. Sampling uniformly in time instead puts the samples closest together
   * at the ends, where the arm is slow, and furthest apart at cruise, which is
   * exactly where the deviation would be largest.
   *
   * @param speed        requested Cartesian speed, mm/s
   * @param acceleration requested Cartesian acceleration, mm/s^2
   * @param pointsPerSecond lower bound on sampling density; the chord limit
   *        raises it when needed
   */
  interpolateCartesianSpace(
    startAngles: number[],
    endPosition: Vector3,
    speed: number,
    acceleration: number,
    pointsPerSecond: number,
    /**
     * Tool orientation to hold along the whole line, or undefined to leave it
     * free. Free, the wrist tips as the arm reaches - several degrees over a
     * 20 mm move - which is fine for getting somewhere and useless for carrying
     * a pen. Held, the solve is exactly determined and costs reach.
     */
    orientation?: Rotation3
  ): TrajectorySegment {
    const startPos = ForwardKinematics.position(startAngles);

    const delta = {
      x: endPosition.x - startPos.x,
      y: endPosition.y - startPos.y,
      z: endPosition.z - startPos.z
    };
    const distance = Math.sqrt(delta.x ** 2 + delta.y ** 2 + delta.z ** 2);

    if (distance < 1e-9) {
      return emptySegment(startAngles);
    }

    // mm/s and mm/s^2 from the caller, metres internally.
    const profile = new VelocityProfile(distance, speed / 1000, acceleration / 1000);
    const duration = profile.getDuration();

    // Check the far end before sampling anything. A failing IK solve is the
    // expensive one - it exhausts every seed - so discovering an unreachable
    // target after several hundred of them is the worst possible order.
    const endCheck = this.solveAt(endPosition, orientation, startAngles);
    if (!endCheck.success) {
      return { ...emptySegment(startAngles), distance, unreachableSamples: 1 };
    }

    // Enough samples to hold the chord below the limit, and never fewer than the
    // caller's density.
    const chordSamples = Math.ceil(distance / MAX_CARTESIAN_CHORD_M);
    const timeSamples = Math.ceil(duration * Math.max(1, pointsPerSecond));
    const steps = Math.min(
      MAX_CARTESIAN_SAMPLES,
      Math.max(1, chordSamples, timeSamples)
    );

    const points: TrajectoryPoint[] = [];
    let currentAngles = [...startAngles];
    let failures = 0;
    let consecutiveFailures = 0;

    for (let k = 0; k <= steps; k++) {
      const s = k / steps;

      const target: Vector3 = {
        x: startPos.x + delta.x * s,
        y: startPos.y + delta.y * s,
        z: startPos.z + delta.z * s
      };

      const ik = this.solveAt(target, orientation, currentAngles);

      if (ik.success) {
        currentAngles = ik.jointAngles;
        consecutiveFailures = 0;
      } else {
        failures++;
        consecutiveFailures++;
      }

      points.push({
        time: timeAtProgress(profile, s),
        jointAngles: [...currentAngles]
      });

      // The line has left the reachable set. Stop here rather than repeat the
      // last pose for the remainder: a short path the caller is told about beats
      // a full-length one that quietly stands still.
      if (consecutiveFailures >= MAX_CONSECUTIVE_IK_FAILURES) break;
    }

    if (points.length > 0) {
      points[points.length - 1].velocity = Array(NUM_JOINTS).fill(0);
    }

    return {
      startWaypoint: 0,
      endWaypoint: 0,
      points,
      duration,
      distance,
      unreachableSamples: failures
    };
  }

  /**
   * Resolve a waypoint to joint angles: use the taught pose when it has one,
   * otherwise solve IK from the current pose.
   */
  resolveWaypointAngles(waypoint: Waypoint, currentAngles: number[]): number[] | null {
    if (waypoint.jointAngles) {
      return [...waypoint.jointAngles];
    }

    // A waypoint that carries an orientation wants it held. The field existed
    // on the type from the start and was never read, so every Cartesian
    // waypoint was solved for position alone.
    const ik = this.solveAt(waypoint.position, waypoint.orientation, currentAngles);
    return ik.success ? ik.jointAngles : null;
  }

  /** Position-only or full-pose solve, depending on whether one was asked for. */
  private solveAt(position: Vector3, orientation: Rotation3 | undefined, seed: number[]) {
    return orientation
      ? this.ikSolver.solvePose({ position, rotation: orientation }, seed)
      : this.ikSolver.solvePosition(position, seed);
  }
}

// ---------------------------------------------------------------------------
// Helpers
// ---------------------------------------------------------------------------

function emptySegment(startAngles: number[]): TrajectorySegment {
  return {
    startWaypoint: 0,
    endWaypoint: 0,
    points: [{ time: 0, jointAngles: [...startAngles], velocity: Array(NUM_JOINTS).fill(0) }],
    duration: 0,
    distance: 0
  };
}

/**
 * Make sure the segment ends exactly on the target, whether the sampling loop
 * happened to land on the duration or stopped just short of it.
 */
function appendExactEnd(
  points: TrajectoryPoint[],
  duration: number,
  endAngles: number[],
  dt: number
): void {
  const last = points[points.length - 1];

  if (!last || Math.abs(last.time - duration) > dt * 0.5) {
    points.push({
      time: duration,
      jointAngles: [...endAngles],
      velocity: Array(NUM_JOINTS).fill(0)
    });
    return;
  }

  last.jointAngles = [...endAngles];
  last.velocity = Array(NUM_JOINTS).fill(0);
}

/** Straight-line TCP distance between two joint poses, in metres. */
function cartesianDistance(a: number[], b: number[]): number {
  const pa = ForwardKinematics.position(a);
  const pb = ForwardKinematics.position(b);
  return Math.sqrt((pb.x - pa.x) ** 2 + (pb.y - pa.y) ** 2 + (pb.z - pa.z) ** 2);
}

/**
 * Time at which a profile has covered fraction `s` of its distance.
 *
 * getProgress is monotone, so a bisection inverts it. Used to attach plausible
 * timestamps to arc-length samples.
 */
function timeAtProgress(profile: VelocityProfile, s: number): number {
  const duration = profile.getDuration();
  if (s <= 0) return 0;
  if (s >= 1) return duration;

  let low = 0;
  let high = duration;
  for (let k = 0; k < 40; k++) {
    const mid = (low + high) / 2;
    if (profile.getProgress(mid) < s) {
      low = mid;
    } else {
      high = mid;
    }
  }
  return (low + high) / 2;
}
