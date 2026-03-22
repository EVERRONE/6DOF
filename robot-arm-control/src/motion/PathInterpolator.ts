// Path Interpolator
// Generates trajectory points between waypoints using joint or Cartesian interpolation

import {
  CartesianInterpolationResult,
  TrajectoryPoint,
  TrajectorySegment,
  Waypoint
} from './types';
import {
  computeSynchronizedDuration,
  createSynchronizedProfiles
} from './VelocityProfile';
import { ForwardKinematics } from '../kinematics/ForwardKinematics';
import { HybridIKSolver } from '../kinematics/HybridIKSolver';
import { InverseKinematics } from '../kinematics/InverseKinematics';
import { IKEngineMode, IKResult, IKSolveOptions, Pose, Rotation3, Vector3 } from '../kinematics/types';
import { logicalToUrdfAngles, urdfToLogicalAngles } from '../kinematics/angleMapping';

/**
 * PathInterpolator
 *
 * Generates trajectory points between two waypoints using either:
 * - Joint space interpolation (smoother in joint space, curved in Cartesian)
 * - Linear Cartesian interpolation (straight line in workspace, requires IK per point)
 */
export class PathInterpolator {
  private hybridIKSolver: HybridIKSolver;
  private legacyIKSolver: InverseKinematics;
  private urdfOffsetsDeg: number[];

  constructor(urdfOffsetsDeg?: number[]) {
    this.hybridIKSolver = new HybridIKSolver();
    this.legacyIKSolver = new InverseKinematics();
    this.urdfOffsetsDeg = urdfOffsetsDeg && urdfOffsetsDeg.length === 6
      ? [...urdfOffsetsDeg]
      : [0, 0, 0, 0, 0, 0];
  }

  setUrdfOffsets(urdfOffsetsDeg: number[]): void {
    this.urdfOffsetsDeg = urdfOffsetsDeg && urdfOffsetsDeg.length === 6
      ? [...urdfOffsetsDeg]
      : [0, 0, 0, 0, 0, 0];
  }

  setJointLimitsDeg(minDeg: number[], maxDeg: number[]): void {
    this.hybridIKSolver.setJointLimitsDeg(minDeg, maxDeg);
    this.legacyIKSolver.setJointLimitsDeg(minDeg, maxDeg);
  }

  private logicalToUrdf(angles: number[]): number[] {
    return logicalToUrdfAngles(angles, this.urdfOffsetsDeg);
  }

  private urdfToLogical(angles: number[]): number[] {
    return urdfToLogicalAngles(angles, this.urdfOffsetsDeg);
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
    const jointDistances = endAngles.map((end, i) => end - startAngles[i]);
    const duration = computeSynchronizedDuration(jointDistances, maxJointSpeed, maxJointAccel);

    if (duration < 1e-6) {
      return {
        startWaypoint: 0,
        endWaypoint: 0,
        points: [{ time: 0, jointAngles: [...startAngles] }],
        duration: 0,
        distance: 0
      };
    }

    const profiles = createSynchronizedProfiles(jointDistances, duration, maxJointAccel);
    const dt = 1.0 / pointsPerSecond;
    const points: TrajectoryPoint[] = [];

    for (let t = 0; t <= duration; t += dt) {
      const angles = startAngles.map((start, i) => start + profiles[i].getPosition(t));
      const velocities = profiles.map((p) => p.getVelocity(t));

      points.push({
        time: t,
        jointAngles: angles,
        velocity: velocities
      });
    }

    const lastPoint = points[points.length - 1];
    if (lastPoint && Math.abs(lastPoint.time - duration) > dt * 0.5) {
      points.push({
        time: duration,
        jointAngles: [...endAngles],
        velocity: Array(6).fill(0)
      });
    } else if (lastPoint) {
      lastPoint.jointAngles = [...endAngles];
      lastPoint.velocity = Array(6).fill(0);
    }

    const fkStart = ForwardKinematics.solve(this.logicalToUrdf(startAngles));
    const fkEnd = ForwardKinematics.solve(this.logicalToUrdf(endAngles));
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
   * Interpolate in Cartesian space with optional pose lock.
   * When targetOrientation is provided, every IK sample enforces it.
   */
  interpolateCartesianSpace(
    startAngles: number[],
    endPosition: Vector3,
    speed: number,          // mm/s
    acceleration: number,   // mm/s^2
    pointsPerSecond: number,
    targetOrientation?: Rotation3,
    solveOptions?: Partial<IKSolveOptions>,
    ikEngineMode: IKEngineMode = 'hybrid_constrained_v2',
    strictFinalGate: boolean = true,
    deadlineMs?: number
  ): CartesianInterpolationResult {
    const fkStart = ForwardKinematics.solve(this.logicalToUrdf(startAngles));
    if (!fkStart.success) {
      return {
        success: false,
        segment: this.createErrorSegment(startAngles),
        error: 'Forward kinematics failed at Cartesian interpolation start'
      };
    }

    const startPos = fkStart.endEffectorPose.position;
    const lockedOrientation = targetOrientation || fkStart.endEffectorPose.rotation;
    const ikOptions: Partial<IKSolveOptions> = {
      orientationWeight: targetOrientation ? 0.25 : 0.0,
      tolerancePositionM: 0.0015,
      toleranceOrientationRad: targetOrientation ? 0.035 : Number.POSITIVE_INFINITY,
      toleranceWeighted: 0.01,
      maxStepDeg: 3.0,
      intent: 'tracking_local',
      mode: targetOrientation ? 'pose_lock' : 'position_only',
      computeDiagnostics: false,
      trackingMaxIterations: 24,
      trackingMode: 'iterative_pose',
      ...solveOptions
    };
    const trackingMode = ikOptions.trackingMode ?? 'iterative_pose';

    const dx = endPosition.x - startPos.x;
    const dy = endPosition.y - startPos.y;
    const dz = endPosition.z - startPos.z;
    const distance = Math.sqrt(dx * dx + dy * dy + dz * dz);

    if (distance < 1e-6) {
      return {
        success: true,
        segment: {
          startWaypoint: 0,
          endWaypoint: 0,
          points: [{ time: 0, jointAngles: [...startAngles] }],
          duration: 0,
          distance: 0
        },
        targetOrientation: lockedOrientation
      };
    }

    const speedMs = Math.max(0.001, speed / 1000);
    const accelMs = Math.max(0.001, acceleration / 1000);
    const duration = this.computeMinimumJerkDuration(distance, speedMs, accelMs);
    const baseSampleCount = Math.max(2, Math.ceil(duration * pointsPerSecond));
    const maxAdaptiveRounds = 4;
    const maxAdaptiveSamples = Math.max(baseSampleCount * 8, 64);
    const maxJointDeltaDeg = Math.max(2.0, (ikOptions.maxStepDeg ?? 3.0) * 1.15);
    const residualGrowthThreshold = Math.max(0.004, (ikOptions.toleranceWeighted ?? 0.01) * 2.5);
    let normalizedSamples = Array.from({ length: baseSampleCount + 1 }, (_, i) => i / baseSampleCount);

    type SolvedSample = {
      u: number;
      t: number;
      jointAnglesUrdf: number[];
      weightedResidual: number;
    };

    type SampleSolveResult = {
      success: boolean;
      points: TrajectoryPoint[];
      solvedSamples: SolvedSample[];
      lastIK: IKResult | null;
      failedIndex?: number;
      failedAtTime?: number;
      failureError?: string;
    };

    const solveAtSamples = (uSamples: number[]): SampleSolveResult => {
      const points: TrajectoryPoint[] = [];
      const solvedSamples: SolvedSample[] = [];
      let currentAnglesUrdf = this.logicalToUrdf(startAngles);
      let previousPosition = startPos;
      let lastIK: IKResult | null = null;

      for (let sampleIndex = 0; sampleIndex < uSamples.length; sampleIndex++) {
        if (deadlineMs !== undefined && Date.now() > deadlineMs) {
          const timeoutAtU = uSamples[Math.max(0, sampleIndex - 1)] ?? 0;
          const timeoutAtTimeSec = duration * timeoutAtU;
          return {
            success: false,
            points,
            solvedSamples,
            lastIK: lastIK || {
              jointAngles: [...currentAnglesUrdf],
              success: false,
              error: `Stage2 interpolation deadline exceeded at sample ${sampleIndex}`,
              errorCode: 'STAGE2_TIMEOUT',
              stage: 'interpolation',
              sampleIndex,
              limitsSource: 'firmware',
              angleFrame: 'urdf',
              failureCategory: 'stage2_timeout'
            },
            failedIndex: sampleIndex,
            failedAtTime: timeoutAtTimeSec,
            failureError: `Stage2 interpolation deadline exceeded at sample ${sampleIndex}`
          };
        }
        const u = uSamples[sampleIndex];
        const t = duration * u;
        const s = this.minimumJerkProgress(u);
        const pos: Vector3 = {
          x: startPos.x + dx * s,
          y: startPos.y + dy * s,
          z: startPos.z + dz * s
        };

        if (sampleIndex === 0) {
          points.push({
            time: t,
            jointAngles: [...startAngles]
          });
          solvedSamples.push({
            u,
            t,
            jointAnglesUrdf: [...currentAnglesUrdf],
            weightedResidual: 0
          });
          previousPosition = pos;
          continue;
        }

        const poseTarget: Pose = { position: pos, rotation: lockedOrientation };
        let ikSolveResult: IKResult;
        if (trackingMode === 'resolved_rate') {
          const prevU = uSamples[sampleIndex - 1] ?? 0;
          const dt = Math.max(1e-3, duration * Math.max(u - prevU, 1 / baseSampleCount));
          const resolvedRateResult = this.legacyIKSolver.stepResolvedRate(
            poseTarget,
            currentAnglesUrdf,
            dt,
            {
              ...ikOptions,
              intent: 'resolved_rate',
              trackingMode: 'resolved_rate'
            }
          );
          if (resolvedRateResult.success) {
            ikSolveResult = resolvedRateResult;
          } else {
            const recoveryOptions: Partial<IKSolveOptions> = {
              ...ikOptions,
              intent: 'tracking_local',
              trackingMode: 'iterative_pose',
              trackingMaxIterations: Math.max(24, ikOptions.trackingMaxIterations ?? 24)
            };
            const iterativeRecovery = this.solveSampleWithRecovery(
              poseTarget,
              currentAnglesUrdf,
              previousPosition,
              lockedOrientation,
              recoveryOptions,
              ikEngineMode
            ).result;
            if (iterativeRecovery.success) {
              ikSolveResult = iterativeRecovery;
            } else {
              ikSolveResult = {
                ...iterativeRecovery,
                error:
                  iterativeRecovery.error ||
                  resolvedRateResult.error ||
                  'Resolved-rate and iterative recovery both failed during Cartesian interpolation'
              };
            }
          }
        } else {
          ikSolveResult = this.solveSampleWithRecovery(
            poseTarget,
            currentAnglesUrdf,
            previousPosition,
            lockedOrientation,
            ikOptions,
            ikEngineMode
          ).result;
        }

        const taggedIK: IKResult = {
          ...ikSolveResult,
          stage: ikSolveResult.stage || 'interpolation',
          sampleIndex,
          errorCode: ikSolveResult.errorCode || (ikSolveResult.success ? undefined : 'IK_INTERPOLATION_SAMPLE_FAILED'),
          angleFrame: ikSolveResult.angleFrame || 'urdf',
          limitsSource: ikSolveResult.limitsSource || 'firmware'
        };
        lastIK = taggedIK;

        if (!taggedIK.success) {
          return {
            success: false,
            points,
            solvedSamples,
            lastIK,
            failedIndex: sampleIndex,
            failedAtTime: t,
            failureError: taggedIK.error || 'Pose IK failed during Cartesian interpolation'
          };
        }

        currentAnglesUrdf = taggedIK.jointAngles;
        points.push({
          time: t,
          jointAngles: this.urdfToLogical(taggedIK.jointAngles)
        });
        solvedSamples.push({
          u,
          t,
          jointAnglesUrdf: [...taggedIK.jointAngles],
          weightedResidual: taggedIK.quality?.weightedResidual ?? 0
        });
        previousPosition = pos;
      }

      return {
        success: true,
        points,
        solvedSamples,
        lastIK
      };
    };

    let points: TrajectoryPoint[] = [];
    let lastIK: IKResult | null = null;
    let failedAtTime: number | undefined;
    let failureError: string | undefined;

    for (let round = 0; round <= maxAdaptiveRounds; round++) {
      if (deadlineMs !== undefined && Date.now() > deadlineMs) {
        failureError = `Stage2 interpolation deadline exceeded before refinement round ${round}`;
        failedAtTime = duration;
        lastIK = lastIK || {
          jointAngles: [...this.logicalToUrdf(startAngles)],
          success: false,
          error: failureError,
          errorCode: 'STAGE2_TIMEOUT',
          stage: 'interpolation',
          sampleIndex: normalizedSamples.length - 1,
          limitsSource: 'firmware',
          angleFrame: 'urdf',
          failureCategory: 'stage2_timeout'
        };
        break;
      }
      const solved = solveAtSamples(normalizedSamples);
      points = solved.points;
      lastIK = solved.lastIK;
      failedAtTime = solved.failedAtTime;
      failureError = solved.failureError;

      if (!solved.success) {
        const failedIndex = solved.failedIndex ?? -1;
        if (failedIndex <= 0 || round >= maxAdaptiveRounds || normalizedSamples.length >= maxAdaptiveSamples) {
          break;
        }
        const uPrev = normalizedSamples[failedIndex - 1];
        const uCurr = normalizedSamples[failedIndex];
        const mid = (uPrev + uCurr) * 0.5;
        if (
          !Number.isFinite(mid) ||
          (uCurr - uPrev) <= 1e-5 ||
          normalizedSamples.some((value) => Math.abs(value - mid) < 1e-8)
        ) {
          break;
        }
        normalizedSamples = [...normalizedSamples, mid].sort((a, b) => a - b);
        continue;
      }

      const pendingMidpoints: number[] = [];
      for (let i = 1; i < solved.solvedSamples.length; i++) {
        const prev = solved.solvedSamples[i - 1];
        const curr = solved.solvedSamples[i];
        let maxDelta = 0;
        for (let j = 0; j < 6; j++) {
          const delta = Math.abs((curr.jointAnglesUrdf[j] ?? 0) - (prev.jointAnglesUrdf[j] ?? 0));
          if (delta > maxDelta) maxDelta = delta;
        }
        const residualGrowth = curr.weightedResidual - prev.weightedResidual;
        if (maxDelta <= maxJointDeltaDeg && residualGrowth <= residualGrowthThreshold) {
          continue;
        }
        const mid = (prev.u + curr.u) * 0.5;
        if (
          (curr.u - prev.u) > 1e-5 &&
          !normalizedSamples.some((value) => Math.abs(value - mid) < 1e-8)
        ) {
          pendingMidpoints.push(mid);
        }
      }

      if (
        pendingMidpoints.length === 0 ||
        round >= maxAdaptiveRounds ||
        normalizedSamples.length >= maxAdaptiveSamples
      ) {
        failureError = undefined;
        failedAtTime = undefined;
        break;
      }

      let added = 0;
      for (const mid of pendingMidpoints) {
        if (normalizedSamples.length + added >= maxAdaptiveSamples) break;
        normalizedSamples.push(mid);
        added += 1;
      }
      if (added === 0) {
        failureError = undefined;
        failedAtTime = undefined;
        break;
      }
      normalizedSamples = Array.from(new Set(normalizedSamples)).sort((a, b) => a - b);
    }

    this.populateJointVelocities(points);

    const segment: TrajectorySegment = {
      startWaypoint: 0,
      endWaypoint: 0,
      points: points.length > 0 ? points : [{ time: 0, jointAngles: [...startAngles] }],
      duration: failureError ? (failedAtTime ?? duration) : duration,
      distance
    };

    if (failureError) {
      const timedOut = /deadline exceeded|stage2 interpolation deadline/i.test(failureError) || Boolean(lastIK?.failureCategory === 'stage2_timeout');
      return {
        success: false,
        segment,
        error: failureError,
        failedAtTime,
        timedOut,
        timeoutAtSampleIndex: lastIK?.sampleIndex,
        timeoutAtTimeSec: failedAtTime,
        targetOrientation: lockedOrientation,
        lastIK: lastIK || undefined,
        trackingMode
      };
    }

    if (
      strictFinalGate &&
      lastIK &&
      !this.isStrictlyConverged(lastIK, ikOptions, Boolean(targetOrientation))
    ) {
      const terminalRefinement = this.refineTerminalSample(
        points,
        startAngles,
        endPosition,
        lockedOrientation,
        ikOptions,
        ikEngineMode,
        Boolean(targetOrientation),
        deadlineMs
      );
      if (terminalRefinement.success && terminalRefinement.lastIK) {
        lastIK = {
          ...terminalRefinement.lastIK,
          errorCode: terminalRefinement.lastIK.errorCode || 'IK_TERMINAL_REFINEMENT_CONVERGED',
          notes: [...(terminalRefinement.lastIK.notes || []), 'terminal_refinement_pass']
        };
        points[points.length - 1].jointAngles = this.urdfToLogical(lastIK.jointAngles);
        this.populateJointVelocities(points);
      } else {
        return {
          success: false,
          segment,
          error: 'Final Cartesian sample did not meet strict residual gate',
          failedAtTime: duration,
          targetOrientation: lockedOrientation,
          lastIK,
          trackingMode
        };
      }
    }

    return {
      success: true,
      segment,
      targetOrientation: lockedOrientation,
      lastIK: lastIK || undefined,
      trackingMode
    };
  }

  /**
   * Resolve a waypoint to joint angles.
   * If the waypoint already has jointAngles, use those.
   * Otherwise, solve IK from waypoint position or pose.
   */
  resolveWaypointAngles(waypoint: Waypoint, currentAngles: number[]): number[] | null {
    if (waypoint.jointAngles) {
      return [...waypoint.jointAngles];
    }

    const initialUrdf = this.logicalToUrdf(currentAngles);
    const ikResult = waypoint.orientation
      ? this.hybridIKSolver.solvePoseWeighted(
          {
            position: waypoint.position,
            rotation: waypoint.orientation
          },
          initialUrdf
        )
      : this.hybridIKSolver.solvePosition(waypoint.position, initialUrdf);

    if (ikResult.success) {
      return this.urdfToLogical(ikResult.jointAngles);
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

  private isStrictlyConverged(
    ikResult: IKResult,
    ikOptions: Partial<IKSolveOptions>,
    hasOrientationConstraint: boolean
  ): boolean {
    if (!ikResult.success || !ikResult.quality) return false;
    const posTol = ikOptions.tolerancePositionM ?? 0.0015;
    const oriTol = hasOrientationConstraint
      ? (ikOptions.toleranceOrientationRad ?? 0.035)
      : Number.POSITIVE_INFINITY;
    const weightedTol = ikOptions.toleranceWeighted ?? 0.01;
    return (
      ikResult.quality.positionResidualM <= posTol &&
      ikResult.quality.orientationResidualRad <= oriTol &&
      ikResult.quality.weightedResidual <= weightedTol
    );
  }

  private refineTerminalSample(
    points: TrajectoryPoint[],
    startAngles: number[],
    targetPosition: Vector3,
    lockedOrientation: Rotation3,
    ikOptions: Partial<IKSolveOptions>,
    ikEngineMode: IKEngineMode,
    hasOrientationConstraint: boolean,
    deadlineMs?: number
  ): { success: boolean; lastIK?: IKResult } {
    if (points.length === 0) {
      return { success: false };
    }
    if (deadlineMs !== undefined && Date.now() > deadlineMs) {
      return { success: false };
    }

    let previousPosition = targetPosition;
    if (points.length > 1) {
      const prevLogical = points[points.length - 2].jointAngles;
      const prevFk = ForwardKinematics.solve(this.logicalToUrdf(prevLogical));
      if (prevFk.success) {
        previousPosition = prevFk.endEffectorPose.position;
      }
    }

    const seedCandidatesUrdf: number[][] = [];
    const pushSeed = (seed: number[] | undefined): void => {
      if (!seed || seed.length !== 6) return;
      if (!seed.every((value) => Number.isFinite(value))) return;
      for (const existing of seedCandidatesUrdf) {
        let same = true;
        for (let i = 0; i < 6; i++) {
          if (Math.abs(existing[i] - seed[i]) > 1e-8) {
            same = false;
            break;
          }
        }
        if (same) return;
      }
      seedCandidatesUrdf.push([...seed]);
    };

    pushSeed(this.logicalToUrdf(points[points.length - 1].jointAngles));
    if (points.length > 1) {
      pushSeed(this.logicalToUrdf(points[points.length - 2].jointAngles));
    }
    pushSeed(this.logicalToUrdf(startAngles));

    const baseTrackingIterations = ikOptions.trackingMaxIterations ?? 24;
    const refinementOptions: Partial<IKSolveOptions>[] = [
      {
        ...ikOptions,
        intent: 'tracking_local',
        trackingMode: 'iterative_pose',
        trackingMaxIterations: Math.max(baseTrackingIterations + 10, 32),
        maxSeeds: Math.max(2, ikOptions.maxSeeds ?? 1),
        maxStages: Math.max(2, ikOptions.maxStages ?? 1),
        maxStepDeg: Math.min(6.0, Math.max(ikOptions.maxStepDeg ?? 4.0, 4.5)),
        postureWeight: Math.max(0, (ikOptions.postureWeight ?? 0.0000002) * 0.4),
        minDamping: Math.max(0.0001, (ikOptions.minDamping ?? 0.0005) * 0.8)
      },
      {
        ...ikOptions,
        intent: 'tracking_local',
        trackingMode: 'iterative_pose',
        trackingMaxIterations: Math.max(baseTrackingIterations + 16, 40),
        maxSeeds: Math.max(3, ikOptions.maxSeeds ?? 1),
        maxStages: Math.max(3, ikOptions.maxStages ?? 1),
        maxStepDeg: Math.min(6.5, Math.max(ikOptions.maxStepDeg ?? 4.0, 5.0)),
        postureWeight: 0
      }
    ];

    const targetPose: Pose = {
      position: targetPosition,
      rotation: lockedOrientation
    };

    let best: IKResult | null = null;
    for (const options of refinementOptions) {
      if (deadlineMs !== undefined && Date.now() > deadlineMs) {
        break;
      }
      for (const seedUrdf of seedCandidatesUrdf) {
        if (deadlineMs !== undefined && Date.now() > deadlineMs) {
          break;
        }
        const candidate = this.solveSampleWithRecovery(
          targetPose,
          seedUrdf,
          previousPosition,
          lockedOrientation,
          options,
          ikEngineMode
        ).result;
        if (!candidate.success) continue;
        if (!this.isStrictlyConverged(candidate, ikOptions, hasOrientationConstraint)) continue;
        if (!best) {
          best = candidate;
          continue;
        }
        const bestResidual = best.quality?.weightedResidual ?? Number.POSITIVE_INFINITY;
        const candidateResidual = candidate.quality?.weightedResidual ?? Number.POSITIVE_INFINITY;
        if (candidateResidual < bestResidual) {
          best = candidate;
        }
      }
    }

    if (!best) {
      return { success: false };
    }
    return { success: true, lastIK: best };
  }

  private solveSampleWithRecovery(
    poseTarget: Pose,
    initialAnglesUrdf: number[],
    previousPosition: Vector3,
    lockedOrientation: Rotation3,
    ikOptions: Partial<IKSolveOptions>,
    ikEngineMode: IKEngineMode
  ): {
    result: IKResult;
  } {
    let result = this.solveWithEngine(ikEngineMode, poseTarget, initialAnglesUrdf, ikOptions);
    if (result.success) return { result };

    const fractions = ikOptions.intent === 'tracking_local' ? [0.5] : [0.5, 0.75];
    let seedAngles = [...initialAnglesUrdf];
    for (const fraction of fractions) {
      const midPose: Pose = {
        position: {
          x: previousPosition.x + (poseTarget.position.x - previousPosition.x) * fraction,
          y: previousPosition.y + (poseTarget.position.y - previousPosition.y) * fraction,
          z: previousPosition.z + (poseTarget.position.z - previousPosition.z) * fraction
        },
        rotation: lockedOrientation
      };

      const mid = this.solveWithEngine(ikEngineMode, midPose, seedAngles, ikOptions);
      if (!mid.success) {
        result = mid;
        continue;
      }

      seedAngles = mid.jointAngles;
      const retry = this.solveWithEngine(ikEngineMode, poseTarget, seedAngles, ikOptions);
      result = retry;
      if (retry.success) return { result: retry };
    }

    return { result };
  }

  private solveWithEngine(
    ikEngineMode: IKEngineMode,
    poseTarget: Pose,
    seed: number[],
    ikOptions: Partial<IKSolveOptions>
  ): IKResult {
    if (ikEngineMode === 'legacy_dls_v1') {
      return this.legacyIKSolver.solvePoseWeighted(poseTarget, seed, ikOptions);
    }
    return this.hybridIKSolver.solvePoseWeighted(poseTarget, seed, ikOptions);
  }

  private computeMinimumJerkDuration(distanceM: number, speedMs: number, accelMs: number): number {
    // For quintic minimum-jerk profile s(t)=10u^3-15u^4+6u^5:
    // max(ds/du) ~= 1.875, max(d2s/du2) ~= 5.8
    const tVel = (1.875 * distanceM) / speedMs;
    const tAcc = Math.sqrt((5.8 * distanceM) / accelMs);
    return Math.max(0.25, tVel, tAcc);
  }

  private minimumJerkProgress(u: number): number {
    const uc = Math.max(0, Math.min(1, u));
    const u2 = uc * uc;
    const u3 = u2 * uc;
    const u4 = u3 * uc;
    const u5 = u4 * uc;
    return 10 * u3 - 15 * u4 + 6 * u5;
  }

  // Matches firmware STREAM_MAX_SPEED_DEG_S = 120.0 in config.h.
  // Velocities above this cause firmware trajectory rejection or audible cracking.
  private static readonly MAX_JOINT_VEL_DEG_S = 120.0;

  private populateJointVelocities(points: TrajectoryPoint[]): void {
    if (points.length === 0) return;
    if (points.length === 1) {
      points[0].velocity = Array(6).fill(0);
      return;
    }

    const maxVel = PathInterpolator.MAX_JOINT_VEL_DEG_S;

    for (let i = 0; i < points.length; i++) {
      const velocity = Array(6).fill(0);
      for (let j = 0; j < 6; j++) {
        let raw: number;
        if (i === 0) {
          const dt = Math.max(1e-6, points[i + 1].time - points[i].time);
          raw = (points[i + 1].jointAngles[j] - points[i].jointAngles[j]) / dt;
        } else if (i === points.length - 1) {
          const dt = Math.max(1e-6, points[i].time - points[i - 1].time);
          raw = (points[i].jointAngles[j] - points[i - 1].jointAngles[j]) / dt;
        } else {
          const dt = Math.max(1e-6, points[i + 1].time - points[i - 1].time);
          raw = (points[i + 1].jointAngles[j] - points[i - 1].jointAngles[j]) / dt;
        }
        velocity[j] = Math.max(-maxVel, Math.min(maxVel, raw));
      }
      points[i].velocity = velocity;
    }

    // Cartesian direct move starts/stops at rest by default.
    points[0].velocity = Array(6).fill(0);
    points[points.length - 1].velocity = Array(6).fill(0);
  }
}
