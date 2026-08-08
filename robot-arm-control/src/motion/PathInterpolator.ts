// Path Interpolator
//
// Turns a pair of waypoints into a series of intermediate joint poses for the
// firmware to run. It produces *geometry*, not timing: the firmware owns the
// velocity profile, so what matters here is that the points lie on the intended
// path. Times are still attached, but only for progress display and duration
// estimates.

import { TrajectoryPoint, TrajectorySegment, Waypoint, WaypointShape } from './types';
import { pointOnCircle } from './Shapes';
import { PathPiece, pointOnPiece } from './CornerBlend';
import { VelocityProfile } from './VelocityProfile';
import { ForwardKinematics } from '../kinematics/ForwardKinematics';
import { InverseKinematics } from '../kinematics/InverseKinematics';
import {
  JOINT_MAX_ACCEL_DEG_S2,
  JOINT_MAX_SPEED_DEG_S,
  NUM_JOINTS
} from '../kinematics/robotModel';
import { Rotation3, Vector3 } from '../kinematics/types';
import {
  angleBetweenQuat,
  matrixToQuat,
  quatToMatrix,
  rpyToMatrix,
  slerp
} from '../kinematics/linalg';

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
    pointsPerSecond: number,
    /**
     * Fraction of each joint's own limit to use. The caller's caps above are an
     * additional ceiling, not a replacement - pass Infinity for them to let the
     * per-joint limits govern alone, which is what the planner does now that it
     * no longer carries a second, staler copy of them.
     */
    scale = 1
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

      const bounded = Math.max(0.01, Math.min(1, scale));
      const speedCap = Math.min(JOINT_MAX_SPEED_DEG_S[i] * bounded, maxJointSpeed);
      const accelCap = Math.min(JOINT_MAX_ACCEL_DEG_S2[i] * bounded, maxJointAccel);

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
    orientation?: Rotation3,
    /**
     * Orientation to arrive in, when it differs from the one being left.
     *
     * Given, the tool is turned from `orientation` to this one along the segment
     * by slerp, so the attitude changes at a constant rate over the move.
     * Omitted, `orientation` is simply held.
     *
     * Without this a path could only hold one attitude or none: two waypoints
     * wanting different ones produced a segment that held the second from its
     * very first sample, so the whole change happened in one step at the
     * boundary. Measured on waypoints 46.0 degrees apart, the held orientation
     * moved 46.1 degrees between two consecutive points.
     */
    endOrientation?: Rotation3
  ): TrajectorySegment {
    const from = ForwardKinematics.position(startAngles);
    const length = Math.hypot(
      endPosition.x - from.x,
      endPosition.y - from.y,
      endPosition.z - from.z
    );

    if (length < 1e-9) {
      return emptySegment(startAngles);
    }

    return this.interpolatePiece(
      startAngles,
      { kind: 'line', from, to: endPosition, length },
      speed,
      acceleration,
      pointsPerSecond,
      orientation,
      endOrientation
    );
  }

  /**
   * Sample one piece of a Cartesian path - a straight leg or a blend arc.
   *
   * The one place a Cartesian path is turned into joint poses, so a blend arc
   * gets exactly the treatment a straight line does: the same chord limit, the
   * same seeded continuity from sample to sample, the same discontinuity check
   * afterwards. A rounded corner that was sampled more coarsely than the legs
   * either side of it would put the deviation precisely where the arm is
   * changing direction fastest.
   */
  interpolatePiece(
    startAngles: number[],
    piece: PathPiece,
    speed: number,
    acceleration: number,
    pointsPerSecond: number,
    orientation?: Rotation3,
    endOrientation?: Rotation3
  ): TrajectorySegment {
    const distance = piece.length;
    if (distance < 1e-9) return emptySegment(startAngles);

    // mm/s and mm/s^2 from the caller, metres internally.
    const profile = new VelocityProfile(distance, speed / 1000, acceleration / 1000);
    const duration = profile.getDuration();

    // The attitude to be in at each point along the way. Held in quaternions
    // because that is the only form an orientation can be interpolated in.
    const turn = orientation ? makeTurn(orientation, endOrientation) : null;

    // Check the far end before sampling anything. A failing IK solve is the
    // expensive one - it exhausts every seed - so discovering an unreachable
    // target after several hundred of them is the worst possible order.
    const endCheck = this.solveAtMatrix(piece.to, turn ? turn(1) : null, startAngles);
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

    const forward = this.walk(piece, turn, profile, steps, startAngles, 'forward');

    if (!forward.discontinuity) {
      return { ...forward, duration, distance };
    }

    // The forward pass jumps. Try again from the far end.
    //
    // The jump is not the solver being capricious: running some lines with the
    // tool held genuinely needs the wrist in a different configuration, and from
    // the parked pose it is in the wrong one. Solving forwards, the first sample
    // sits at the singularity where the split between J4 and J6 is arbitrary, so
    // the solver takes the identity split and then has to migrate 90 degrees -
    // which it does all at once, in whichever single step is cheapest.
    //
    // The far end is away from the singularity and well conditioned. Walked
    // backwards from there, every sample stays near its predecessor and the
    // reconfiguration never has to happen at all: measured on a 100 mm line from
    // the parked pose, 44.1 degrees becomes 0.7.
    const backward = this.walk(
      piece,
      turn,
      profile,
      steps,
      forward.points[forward.points.length - 1].jointAngles,
      'backward'
    );

    if (!backward.discontinuity && backward.points.length === forward.points.length) {
      const entry = backward.points[0].jointAngles;
      const swing = Math.max(...entry.map((v, i) => Math.abs(v - startAngles[i])));

      return {
        ...backward,
        duration,
        distance,
        reconfiguration: swing > RECONFIGURE_THRESHOLD_DEG ? [...entry] : null
      };
    }

    // Backward did not help either, which is what happens on a short move: its
    // far end is only millimetres away, so it is just as badly conditioned as
    // the near one. A 5 mm jog away from the singularity cannot be rescued by
    // starting from the other end of 5 mm.
    //
    // But the forward pass has already found the configuration the line wants -
    // it is the one on the far side of the jump. Solving the *start* point from
    // there gives the same tool pose in that configuration, and walking forward
    // from it never has to reconfigure at all. This is the general form of what
    // the backward pass does by luck on a long line.
    const jump = forward.discontinuity;
    const after = forward.points[Math.min(jump.index, forward.points.length - 1)].jointAngles;
    const entrySolve = this.solveAtMatrix(pointOnPiece(piece, 0), turn ? turn(0) : null, after);

    if (entrySolve.success) {
      const hoisted = this.walk(piece, turn, profile, steps, entrySolve.jointAngles, 'forward');

      if (!hoisted.discontinuity && hoisted.points.length === forward.points.length) {
        const entry = hoisted.points[0].jointAngles;
        const swing = Math.max(...entry.map((v, i) => Math.abs(v - startAngles[i])));

        return {
          ...hoisted,
          duration,
          distance,
          reconfiguration: swing > RECONFIGURE_THRESHOLD_DEG ? [...entry] : null
        };
      }
    }

    // Nothing worked - keep the forward pass and its discontinuity, so the
    // caller is told rather than handed a worse path silently.
    return { ...forward, duration, distance };

  }

  /**
   * Sample a piece end to end, seeding each solve from the one before.
   *
   * Backwards, the samples are generated from the far end and reversed, so the
   * seeding runs the other way while the geometry and the timing come out
   * identical.
   */
  private walk(
    piece: PathPiece,
    turn: ((s: number) => number[][]) | null,
    profile: VelocityProfile,
    steps: number,
    seedAngles: number[],
    direction: 'forward' | 'backward'
  ): TrajectorySegment {
    const points: TrajectoryPoint[] = [];
    let currentAngles = [...seedAngles];
    let failures = 0;
    let consecutiveFailures = 0;

    for (let k = 0; k <= steps; k++) {
      const s = direction === 'forward' ? k / steps : 1 - k / steps;
      const target = pointOnPiece(piece, s);

      const ik = this.solveAtMatrix(target, turn ? turn(s) : null, currentAngles);

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

    if (direction === 'backward') points.reverse();

    if (points.length > 0) {
      points[points.length - 1].velocity = Array(NUM_JOINTS).fill(0);
    }

    return {
      startWaypoint: 0,
      endWaypoint: 0,
      points,
      duration: profile.getDuration(),
      distance: piece.length,
      unreachableSamples: failures,
      discontinuity: findDiscontinuity(points, piece.length)
    };
  }

  /**
   * Sample along a figure rather than between points chosen earlier.
   *
   * This is why a shape is one waypoint: the path can be sampled at whatever
   * density the motion needs, on the true arc. Expanding the figure into fixed
   * waypoints at creation time locks in a polygon, and every later stage can
   * only chord between its corners.
   */
  interpolateArc(
    startAngles: number[],
    shape: WaypointShape,
    centre: Vector3,
    speed: number,
    acceleration: number,
    pointsPerSecond: number,
    orientation?: Rotation3
  ): TrajectorySegment {
    const length = 2 * Math.PI * shape.radius;
    if (length < 1e-9) return emptySegment(startAngles);

    const profile = new VelocityProfile(length, speed / 1000, acceleration / 1000);
    const duration = profile.getDuration();

    // Same chord rule as anywhere else on a curve, so an arc is no less round
    // than a straight line is straight.
    const chordSamples = Math.ceil(length / MAX_CARTESIAN_CHORD_M);
    const timeSamples = Math.ceil(duration * Math.max(1, pointsPerSecond));
    const steps = Math.min(
      MAX_CARTESIAN_SAMPLES,
      Math.max(8, chordSamples, timeSamples)
    );

    const points: TrajectoryPoint[] = [];
    let currentAngles = [...startAngles];
    let failures = 0;
    let consecutiveFailures = 0;

    for (let k = 0; k <= steps; k++) {
      const s = k / steps;
      const target = pointOnCircle(shape, centre, s);
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
      distance: length,
      unreachableSamples: failures,
      discontinuity: findDiscontinuity(points, length)
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

  /** As solveAt, for an orientation already reduced to a rotation matrix. */
  private solveAtMatrix(position: Vector3, rotation: number[][] | null, seed: number[]) {
    return rotation
      ? this.ikSolver.solvePoseMatrix(position, rotation, seed)
      : this.ikSolver.solvePosition(position, seed);
  }
}

/**
 * The attitude to be in at each fraction of a segment.
 *
 * With one orientation this is a constant. With two it is a slerp between them,
 * so the tool turns at a steady rate over the move instead of the whole change
 * landing in a single step at the boundary. Built once per segment rather than
 * per sample: the conversion to quaternions is the only part that is not free,
 * and a segment can run to a thousand samples.
 */
function makeTurn(from: Rotation3, to?: Rotation3): (s: number) => number[][] {
  const a = matrixToQuat(rpyToMatrix(from));

  if (!to) {
    const held = quatToMatrix(a);
    return () => held;
  }

  const b = matrixToQuat(rpyToMatrix(to));

  // Nothing to turn through. Worth catching, because slerp between two nearly
  // identical quaternions is the case its own fallback exists for, and skipping
  // it entirely is both faster and exact.
  if (angleBetweenQuat(a, b) < 1e-9) {
    const held = quatToMatrix(a);
    return () => held;
  }

  return (s: number) => quatToMatrix(slerp(a, b, Math.max(0, Math.min(1, s))));
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

/**
 * How far a single step must stand out from the rest of the path before it
 * counts as a discontinuity rather than a fast bit.
 *
 * Relative rather than absolute, which is how MoveIt's Cartesian interpolator
 * treats the same problem. A path where every step is large is simply a fast
 * path, and the firmware clamps it to the joint limits; a path where one step is
 * forty times its neighbours is the arm reconfiguring through a singularity.
 */
const JUMP_FACTOR = 10;

/**
 * Absolute floor, so a path made of very small steps cannot trip the relative
 * test on rounding noise. Below this the firmware's own interpolation across the
 * step cannot take the tool far from where it should be anyway.
 */
const JUMP_FLOOR_DEG = 5;

/**
 * Degrees of joint travel per millimetre of tool travel, above which a step is a
 * reconfiguration whatever the rest of the path looks like.
 *
 * The relative test above cannot stand alone. It compares a step against the
 * path's median step, which is meaningful over a hundred samples and meaningless
 * over eight: a short jog near a singularity has *every* step large, so the
 * outlier test finds no outlier and a 43 degree swap over 5 mm passes as normal.
 *
 * This one is a physical ratio and does not care how long the path is or how
 * finely it was sampled. A well-conditioned path costs well under a degree per
 * millimetre; ten is already the arm reconfiguring rather than travelling.
 */
const JUMP_DEG_PER_MM = 10;

/**
 * How far the wrist has to be from the configuration a segment needs before the
 * caller is told to turn it there first.
 *
 * Below this the difference is ordinary solver residual, and prepending a move
 * for it would cost a stop at the start of every path.
 */
const RECONFIGURE_THRESHOLD_DEG = 5;

/**
 * Find where the joint path jumps, if it does.
 *
 * The tool pose is right at every sample by construction - IK put it there. What
 * this catches is a pair of samples the tool can barely tell apart while the arm
 * between them is somewhere else entirely. Sampling the line more finely does
 * not help: the jump is in joint space, and halving the Cartesian step just puts
 * the same reconfiguration into a smaller gap.
 */
function findDiscontinuity(
  points: TrajectoryPoint[],
  /** Tool travel the whole piece covers, in metres. Used for the °/mm test. */
  distance: number
): TrajectorySegment['discontinuity'] {
  if (points.length < 3) return null;

  const steps: number[] = [];
  const worstAxis: number[] = [];

  for (let i = 1; i < points.length; i++) {
    let step = 0;
    let axis = 0;
    for (let j = 0; j < NUM_JOINTS; j++) {
      const d = Math.abs(points[i].jointAngles[j] - points[i - 1].jointAngles[j]);
      if (d > step) {
        step = d;
        axis = j;
      }
    }
    steps.push(step);
    worstAxis.push(axis);
  }

  const sorted = [...steps].sort((a, b) => a - b);
  const median = sorted[Math.floor(sorted.length / 2)];

  // How far the tool moves between two samples, in millimetres.
  const perStepMm = (distance * 1000) / steps.length;

  let worst = -1;
  for (let i = 0; i < steps.length; i++) {
    if (steps[i] < JUMP_FLOOR_DEG) continue;

    const outlier = steps[i] >= JUMP_FACTOR * median;
    const steep = perStepMm > 1e-9 && steps[i] / perStepMm >= JUMP_DEG_PER_MM;
    if (!outlier && !steep) continue;

    if (worst < 0 || steps[i] > steps[worst]) worst = i;
  }

  if (worst < 0) return null;

  return {
    index: worst + 1,
    atPercent: Math.round(((worst + 1) / (points.length - 1)) * 100),
    axis: worstAxis[worst],
    degrees: steps[worst]
  };
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
