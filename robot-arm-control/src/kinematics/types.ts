// Kinematics type definitions for 6DOF robot arm

/**
 * 3D Vector (position or direction)
 */
export interface Vector3 {
  x: number;
  y: number;
  z: number;
}

/**
 * 3D Rotation (Euler angles in radians)
 */
export interface Rotation3 {
  roll: number;   // Rotation around X axis
  pitch: number;  // Rotation around Y axis
  yaw: number;    // Rotation around Z axis
}

/**
 * Unit quaternion (w + xi + yj + zk)
 */
export interface Quaternion {
  w: number;
  x: number;
  y: number;
  z: number;
}

/**
 * 6DOF Pose (position + orientation)
 */
export interface Pose {
  position: Vector3;
  // Euler rotation is retained for UI I/O compatibility.
  rotation: Rotation3;
}

/**
 * 6DOF Pose represented with quaternion orientation.
 */
export interface PoseQuat {
  position: Vector3;
  orientation: Quaternion;
}

/**
 * Denavit-Hartenberg Parameters (Modified DH Convention)
 *
 * Modified DH parameters (Craig convention):
 * - alpha: Twist angle between Z(i-1) and Z(i) about X(i-1) [radians]
 * - a: Link length from Z(i-1) to Z(i) along X(i-1) [meters]
 * - d: Link offset from X(i-1) to X(i) along Z(i) [meters]
 * - theta: Joint angle between X(i-1) and X(i) about Z(i) [radians]
 */
export interface DHParameter {
  alpha: number;  // Twist angle (rad)
  a: number;      // Link length (m)
  d: number;      // Link offset (m)
  theta: number;  // Joint angle (rad) - variable for revolute joints
}

/**
 * 4x4 Homogeneous Transformation Matrix
 */
export type Matrix4x4 = number[][];

/**
 * URDF Joint definition
 */
export interface URDFJoint {
  name: string;
  type: 'revolute' | 'continuous' | 'fixed';
  parent: string;
  child: string;
  origin: {
    xyz: Vector3;
    rpy: Rotation3;
  };
  axis: Vector3;
  limit?: {
    lower: number;
    upper: number;
    effort: number;
    velocity: number;
  };
}

/**
 * Forward Kinematics Result
 */
export interface FKResult {
  endEffectorPose: Pose;
  jointTransforms: Matrix4x4[];  // Transform for each joint
  success: boolean;
  error?: string;
}

/**
 * Inverse Kinematics Result
 */
export interface IKResult {
  jointAngles: number[];  // Solution in degrees
  success: boolean;
  error?: string;
  errorCode?: string;
  iterations?: number;
  residualError?: number;
  quality?: IKQualityMetrics;
  branchId?: IKBranchId;
  solverPath?: IKSolverPath;
  stage?: IKSolveStage;
  sampleIndex?: number;
  limitsSource?: IKLimitsSource;
  angleFrame?: IKAngleFrame;
  singularityFlags?: string[];
  limitMarginDeg?: number;
  boundaryDistanceM?: number | null;
  collisionFree?: boolean;
  failureCategory?:
    | 'fk_failure'
    | 'max_iterations'
    | 'stage2_timeout'
    | 'singularity'
    | 'joint_limit'
    | 'orientation_infeasible'
    | 'queue_upload_failed'
    | 'invalid_target'
    | 'collision'
    | 'boundary_clamped'
    | 'branch_discontinuity';
  notes?: string[];
}

export type IKAngleFrame = 'logical' | 'urdf';
export type IKLimitsSource = 'firmware' | 'urdf';
export type IKSolveStage =
  | 'endpoint'
  | 'stage1_fast'
  | 'stage2_refine'
  | 'interpolation'
  | 'execution'
  | 'unknown';

export interface IKSolveContext {
  angleFrame: IKAngleFrame;
  limitsSource: IKLimitsSource;
  previousSolutionUrdfDeg: number[];
  preferredBranch?: IKBranchId;
}

export interface IKSampleDiagnostic {
  stage: IKSolveStage;
  sampleIndex: number;
  tSec: number;
  residualPosM: number;
  residualOriRad: number;
  weightedResidual: number;
  branchId?: string;
  flags: string[];
  note?: string;
}

/**
 * IK Solver Configuration
 */
export interface IKConfig {
  maxIterations: number;
  tolerance: number;          // Position tolerance in meters
  dampingFactor: number;      // Damping for Jacobian pseudo-inverse
  jointLimits: {
    min: number[];
    max: number[];
  };
}

/**
 * Optional tuning for weighted full-pose IK solve.
 */
export interface IKSolveOptions {
  positionWeight?: number;
  orientationWeight?: number;
  intent?: IKSolveIntent;
  mode?: CartesianMode;
  maxStepDeg?: number;
  maxJointVelocityDegS?: number;
  maxJointAccelerationDegS2?: number;
  trackingMaxIterations?: number;
  maxSeeds?: number;
  maxStages?: number;
  // Dimensionless damping range used on a normalized J^T*J scale.
  minDamping?: number;
  maxDamping?: number;
  dampingGrowth?: number;
  dampingShrink?: number;
  singularityThreshold?: number;
  // Dimensionless posture regularization weight on the normalized solve scale.
  postureWeight?: number;
  tolerancePositionM?: number;
  toleranceOrientationRad?: number;
  toleranceWeighted?: number;
  computeDiagnostics?: boolean;
  diagnosticStride?: number;
  logDiagnostics?: boolean;
  activeConstraints?: string[];
  preferredBranch?: IKBranchId | null;
  previousSolutionDeg?: number[] | null;
  branchLockEnabled?: boolean;
  boundaryDistanceM?: number | null;
  collisionCheckEnabled?: boolean;
  trackingMode?: IKTrackingMode;
}

/**
 * Detailed quality information for IK convergence diagnostics.
 */
export interface IKQualityMetrics {
  positionResidualM: number;
  orientationResidualRad: number;
  weightedResidual: number;
  minSingularValue: number;
  conditionNumber: number;
  activeConstraints: string[];
  jointParticipation: number[];
  iterations: number;
  damping: number;
  seedIndex?: number;
  branchId?: string;
  retryStage?: string;
  linearityErrorMm?: number;
  jointLimitMarginDeg?: number;
}

/**
 * Jacobian Matrix (6 x n) for velocity kinematics
 * Maps joint velocities to end-effector velocities
 */
export type JacobianMatrix = number[][];

export type CartesianMode = 'pose_lock' | 'position_only';
export type IKEngineMode = 'legacy_dls_v1' | 'hybrid_constrained_v2';
export type IKSolveProfile = 'smooth' | 'balanced' | 'precision';
export type IKSolveIntent = 'endpoint_global' | 'tracking_local' | 'resolved_rate';
export type IKTrackingMode = 'iterative_pose' | 'resolved_rate';
export type IKSolverPath = 'analytic' | 'analytic_refined' | 'numeric_fallback';
export type IKBranchId =
  | 'SL_EU_WF'
  | 'SL_EU_WN'
  | 'SL_ED_WF'
  | 'SL_ED_WN'
  | 'SR_EU_WF'
  | 'SR_EU_WN'
  | 'SR_ED_WF'
  | 'SR_ED_WN';

export interface IKRequest {
  targetPosition: Vector3;
  targetOrientationQuat: Quaternion;
  mode: CartesianMode;
  intent: IKSolveIntent;
  preferredBranch?: IKBranchId;
  previousSolutionDeg?: number[];
}

export interface IKPlanningBudget {
  stage1BudgetMs: number;
  stage2MaxMs: number;
  trackingMaxIterations: number;
}
