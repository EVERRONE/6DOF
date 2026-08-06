import React, { useEffect, useMemo, useState } from 'react';
import { useRobotStore } from '../store/robotStore';
import { Vector3 } from '../kinematics/types';
import { ConnectionStatus } from '../types/robot';

// Soft UI bounds — much larger than physical reach to avoid false rejections.
// Reachability is determined by the IK solver using firmware joint limits.
const WORKSPACE_LIMITS = {
  x: { min: -0.6, max: 0.6 },
  y: { min: -0.6, max: 0.6 },
  z: { min: -0.3, max: 0.7 }
};

/**
 * Cartesian Control Panel Component
 *
 * Allows user to control robot end-effector in Cartesian coordinates (X, Y, Z)
 * Uses inverse kinematics to convert XYZ position to joint angles
 */
export const CartesianControlPanel: React.FC = () => {
  const {
    currentPosition,
    targetPosition,
    setTargetPosition,
    cancelCartesianPlanning,
    moveToPosition,
    motorsEnabled,
    connectionStatus,
    ikStatus,
    ikDiagnostics,
    homedJoints,
    isCartesianReady,
    kinematicsFrameReady,
    ikEngineMode,
    ikSolveProfile,
    ikPrimaryMode,
    branchLockEnabled,
    resolvedRateEnabled,
    collisionCheckEnabled,
    setIKEngineMode,
    setIKSolveProfile,
    setIKPrimaryMode,
    setBranchLockEnabled,
    setResolvedRateEnabled,
    setCollisionCheckEnabled,
    reachabilityAtlasReady,
    motionKernelStatus,
    requestMotionKernelStatus,
    cartesianMode,
    setCartesianMode,
    jogCartesian,
    queueProgress,
    planningState,
    planningLatencyMs,
    planningNotes,
    firmwareConfig
  } = useRobotStore();

  // Local state for input fields (mm, user-owned values)
  const [inputX, setInputX] = useState<string>('0');
  const [inputY, setInputY] = useState<string>('0');
  const [inputZ, setInputZ] = useState<string>('0');
  const [hasUserEditedTarget, setHasUserEditedTarget] = useState(false);
  const [hasSeededFromCurrentFrame, setHasSeededFromCurrentFrame] = useState(false);
  const [jogStepMm, setJogStepMm] = useState(5);
  const [jogStepDeg, setJogStepDeg] = useState(5);
  const [jogFrame, setJogFrame] = useState<'world' | 'tool'>('world');

  const isConnected = connectionStatus === ConnectionStatus.CONNECTED;
  const orientationResidualDeg = ikStatus?.quality
    ? (ikStatus.quality.orientationResidualRad * 180) / Math.PI
    : null;
  const conditionNumber = ikStatus?.quality?.conditionNumber ?? null;
  const minSingularValue = ikStatus?.quality?.minSingularValue ?? null;
  const weakJoints = ikStatus?.quality?.jointParticipation
    ? (() => {
        const participation = ikStatus.quality.jointParticipation;
        const maxValue = participation.reduce((acc, value) => Math.max(acc, value), 0);
        const threshold = maxValue * 0.1;
        return participation
          .map((value, index) => ({ value, index }))
          .filter((entry) => entry.value <= threshold)
          .map((entry) => `J${entry.index + 1}`);
      })()
    : [];

  useEffect(() => {
    if (!isConnected || !firmwareConfig?.capabilities?.motionKernelDiag) return;
    requestMotionKernelStatus().catch((error) => {
      console.warn('Failed to query motion kernel status:', error);
    });
  }, [isConnected, firmwareConfig, requestMotionKernelStatus]);

  // Reset init state on disconnect and clear preview marker.
  useEffect(() => {
    if (!isConnected) {
      setHasSeededFromCurrentFrame(false);
      setHasUserEditedTarget(false);
      setTargetPosition(null);
    }
  }, [isConnected, setTargetPosition]);

  // Allow re-seeding after homing or other flows that intentionally clear the marker.
  useEffect(() => {
    if (isConnected && kinematicsFrameReady && !hasUserEditedTarget && targetPosition === null) {
      setHasSeededFromCurrentFrame(false);
    }
  }, [isConnected, kinematicsFrameReady, hasUserEditedTarget, targetPosition]);

  // Initialize the input fields once after connect from the first valid pose.
  useEffect(() => {
    if (
      !isConnected ||
      !kinematicsFrameReady ||
      hasSeededFromCurrentFrame ||
      hasUserEditedTarget ||
      !currentPosition
    ) {
      return;
    }

    setInputX((currentPosition.x * 1000).toFixed(1));
    setInputY((currentPosition.y * 1000).toFixed(1));
    setInputZ((currentPosition.z * 1000).toFixed(1));
    setTargetPosition({ ...currentPosition });
    setHasSeededFromCurrentFrame(true);
  }, [
    isConnected,
    kinematicsFrameReady,
    hasSeededFromCurrentFrame,
    hasUserEditedTarget,
    currentPosition,
    setTargetPosition
  ]);

  const parsedTarget = useMemo((): { position: Vector3 | null; error: string | null } => {
    const xMm = Number.parseFloat(inputX);
    const yMm = Number.parseFloat(inputY);
    const zMm = Number.parseFloat(inputZ);

    if (!Number.isFinite(xMm) || !Number.isFinite(yMm) || !Number.isFinite(zMm)) {
      return { position: null, error: 'Enter numeric X, Y, and Z values.' };
    }

    const x = xMm / 1000;
    const y = yMm / 1000;
    const z = zMm / 1000;

    if (
      x < WORKSPACE_LIMITS.x.min || x > WORKSPACE_LIMITS.x.max ||
      y < WORKSPACE_LIMITS.y.min || y > WORKSPACE_LIMITS.y.max ||
      z < WORKSPACE_LIMITS.z.min || z > WORKSPACE_LIMITS.z.max
    ) {
      return {
        position: null,
        error: `Target out of workspace: X ${WORKSPACE_LIMITS.x.min * 1000}..${WORKSPACE_LIMITS.x.max * 1000} mm, ` +
          `Y ${WORKSPACE_LIMITS.y.min * 1000}..${WORKSPACE_LIMITS.y.max * 1000} mm, ` +
          `Z ${WORKSPACE_LIMITS.z.min * 1000}..${WORKSPACE_LIMITS.z.max * 1000} mm.`
      };
    }

    return { position: { x, y, z }, error: null };
  }, [inputX, inputY, inputZ]);

  // Keep target marker in sync with validated user input only while the user is editing.
  useEffect(() => {
    if (!isConnected || !kinematicsFrameReady) {
      setTargetPosition(null);
      return;
    }

    if (!hasUserEditedTarget) {
      return;
    }

    if (!parsedTarget.position) {
      setTargetPosition(null);
      return;
    }

    setTargetPosition(parsedTarget.position);
  }, [isConnected, kinematicsFrameReady, hasUserEditedTarget, parsedTarget.position, setTargetPosition]);

  const homedForCartesian =
    homedJoints.J2 && homedJoints.J3 && homedJoints.J4 && homedJoints.J5;
  const cartesianReady = isCartesianReady();
  const queueCapable = Boolean(firmwareConfig?.capabilities?.trajectoryQueue);
  const planningBusy = planningState === 'stage1_fast' || planningState === 'stage2_refine';
  const canMove = cartesianReady && queueCapable && parsedTarget.position !== null && !planningBusy;

  const markUserEditing = (): void => {
    if (planningBusy) {
      cancelCartesianPlanning('Planning cancelled: target edited');
    }
    setHasUserEditedTarget(true);
  };

  const disabledReason = (() => {
    if (!isConnected) return 'Connect to robot';
    if (!kinematicsFrameReady) return 'Waiting for Cartesian frame sync';
    if (!motorsEnabled) return 'Enable motors';
    if (!homedForCartesian) return 'Home J2..J5 first';
    if (!firmwareConfig?.capabilities?.trajectoryQueue) return 'Update firmware: trajectory queue (TQ) required';
    if (planningBusy) return 'Planning in progress...';
    if (parsedTarget.error) return parsedTarget.error;
    return null;
  })();

  const handleMoveToPosition = async () => {
    if (!parsedTarget.position) return;
    setHasUserEditedTarget(false);
    await moveToPosition(parsedTarget.position);
  };

  const canJog = cartesianReady && queueCapable && !planningBusy;

  const handleJog = async (axis: 'x' | 'y' | 'z' | 'rx' | 'ry' | 'rz', sign: 1 | -1) => {
    const stepM = jogStepMm / 1000;
    const stepRad = (jogStepDeg * Math.PI) / 180;
    const jogDelta: { dx?: number; dy?: number; dz?: number; rx?: number; ry?: number; rz?: number; frame: 'world' | 'tool' } = { frame: jogFrame };
    if (axis === 'x')  jogDelta.dx = sign * stepM;
    if (axis === 'y')  jogDelta.dy = sign * stepM;
    if (axis === 'z')  jogDelta.dz = sign * stepM;
    if (axis === 'rx') jogDelta.rx = sign * stepRad;
    if (axis === 'ry') jogDelta.ry = sign * stepRad;
    if (axis === 'rz') jogDelta.rz = sign * stepRad;
    await jogCartesian(jogDelta);
  };

  const handleSetCurrentAsTarget = () => {
    if (!currentPosition || !kinematicsFrameReady) return;
    setInputX((currentPosition.x * 1000).toFixed(1));
    setInputY((currentPosition.y * 1000).toFixed(1));
    setInputZ((currentPosition.z * 1000).toFixed(1));
    setTargetPosition({ ...currentPosition });
    setHasUserEditedTarget(false);
    setHasSeededFromCurrentFrame(true);
  };

  return (
    <div className="p-4 bg-white border-b">
      <h2 className="text-xl font-bold mb-4">Cartesian Control</h2>

      {/* Current Position Display */}
      <div className="mb-4 p-3 bg-gray-50 rounded">
        <h3 className="text-sm font-semibold mb-2 text-gray-700">Current Position</h3>
        <div className="grid grid-cols-3 gap-2 text-sm">
          <div>
            <span className="text-gray-600">X:</span>
            <span className="ml-2 font-mono">
              {currentPosition ? (currentPosition.x * 1000).toFixed(1) : '---'}
            </span>
            <span className="text-gray-500 ml-1">mm</span>
          </div>
          <div>
            <span className="text-gray-600">Y:</span>
            <span className="ml-2 font-mono">
              {currentPosition ? (currentPosition.y * 1000).toFixed(1) : '---'}
            </span>
            <span className="text-gray-500 ml-1">mm</span>
          </div>
          <div>
            <span className="text-gray-600">Z:</span>
            <span className="ml-2 font-mono">
              {currentPosition ? (currentPosition.z * 1000).toFixed(1) : '---'}
            </span>
            <span className="text-gray-500 ml-1">mm</span>
          </div>
        </div>
      </div>

      {/* Target Position Inputs */}
      <div className="mb-4">
        <h3 className="text-sm font-semibold mb-2 text-gray-700">Target Position (mm)</h3>
        <div className="grid grid-cols-3 gap-3">
          <div>
            <label className="block text-xs text-gray-600 mb-1">X</label>
            <input
              type="number"
              value={inputX}
              onChange={(e) => {
                setInputX(e.target.value);
                markUserEditing();
              }}
              className="w-full px-2 py-1 border rounded text-sm font-mono"
              step="1"
              disabled={!isConnected}
            />
            <span className="text-xs text-gray-400">
              {WORKSPACE_LIMITS.x.min * 1000} to {WORKSPACE_LIMITS.x.max * 1000}
            </span>
          </div>

          <div>
            <label className="block text-xs text-gray-600 mb-1">Y</label>
            <input
              type="number"
              value={inputY}
              onChange={(e) => {
                setInputY(e.target.value);
                markUserEditing();
              }}
              className="w-full px-2 py-1 border rounded text-sm font-mono"
              step="1"
              disabled={!isConnected}
            />
            <span className="text-xs text-gray-400">
              {WORKSPACE_LIMITS.y.min * 1000} to {WORKSPACE_LIMITS.y.max * 1000}
            </span>
          </div>

          <div>
            <label className="block text-xs text-gray-600 mb-1">Z</label>
            <input
              type="number"
              value={inputZ}
              onChange={(e) => {
                setInputZ(e.target.value);
                markUserEditing();
              }}
              className="w-full px-2 py-1 border rounded text-sm font-mono"
              step="1"
              disabled={!isConnected}
            />
            <span className="text-xs text-gray-400">
              {WORKSPACE_LIMITS.z.min * 1000} to {WORKSPACE_LIMITS.z.max * 1000}
            </span>
          </div>
        </div>
      </div>

      <div className="mb-3 text-xs text-gray-600">
        Homed for Cartesian:
        <span className={`ml-2 ${homedJoints.J2 ? 'text-green-700' : 'text-red-600'}`}>J2 {homedJoints.J2 ? 'OK' : 'NO'}</span>
        <span className={`ml-2 ${homedJoints.J3 ? 'text-green-700' : 'text-red-600'}`}>J3 {homedJoints.J3 ? 'OK' : 'NO'}</span>
        <span className={`ml-2 ${homedJoints.J4 ? 'text-green-700' : 'text-red-600'}`}>J4 {homedJoints.J4 ? 'OK' : 'NO'}</span>
        <span className={`ml-2 ${homedJoints.J5 ? 'text-green-700' : 'text-red-600'}`}>J5 {homedJoints.J5 ? 'OK' : 'NO'}</span>
      </div>

      <div className="mb-3 p-2 rounded text-xs bg-blue-50 text-blue-800">
        J6 is orientation-only for the current TCP definition (no direct XYZ leverage in position-only mode).
      </div>

      <div className="mb-3 p-2 rounded border bg-gray-50">
        <div className="text-xs font-semibold text-gray-700 mb-2">Cartesian Mode</div>
        <div className="flex gap-2">
          <button
            onClick={() => setCartesianMode('pose_lock')}
            className={`px-3 py-1 rounded text-xs font-semibold ${
              cartesianMode === 'pose_lock'
                ? 'bg-blue-600 text-white'
                : 'bg-white text-gray-700 border'
            }`}
          >
            Pose Lock
          </button>
          <button
            onClick={() => setCartesianMode('position_only')}
            className={`px-3 py-1 rounded text-xs font-semibold ${
              cartesianMode === 'position_only'
                ? 'bg-blue-600 text-white'
                : 'bg-white text-gray-700 border'
            }`}
          >
            Position Only
          </button>
        </div>
        <div className="mt-2 text-xs text-gray-600">
          {cartesianMode === 'pose_lock'
            ? 'Pose lock: keep current tool orientation and reject infeasible targets.'
            : 'Position only: prioritize XYZ target; orientation may change.'}
        </div>
      </div>

      <div className="mb-3 p-2 rounded border bg-gray-50">
        <div className="text-xs font-semibold text-gray-700 mb-2">IK Engine</div>
        <div className="flex flex-wrap gap-2">
          <button
            onClick={() => setIKEngineMode('hybrid_constrained_v2')}
            className={`px-3 py-1 rounded text-xs font-semibold ${
              ikEngineMode === 'hybrid_constrained_v2'
                ? 'bg-green-600 text-white'
                : 'bg-white text-gray-700 border'
            }`}
          >
            Hybrid V2
          </button>
          <button
            onClick={() => setIKEngineMode('legacy_dls_v1')}
            className={`px-3 py-1 rounded text-xs font-semibold ${
              ikEngineMode === 'legacy_dls_v1'
                ? 'bg-green-600 text-white'
                : 'bg-white text-gray-700 border'
            }`}
          >
            Legacy DLS
          </button>
        </div>

        <div className="mt-2 text-xs font-semibold text-gray-700">Solve Profile</div>
        <div className="flex flex-wrap gap-2 mt-1">
          {(['smooth', 'balanced', 'precision'] as const).map((profile) => (
            <button
              key={profile}
              onClick={() => setIKSolveProfile(profile)}
              className={`px-3 py-1 rounded text-xs font-semibold ${
                ikSolveProfile === profile
                  ? 'bg-indigo-600 text-white'
                  : 'bg-white text-gray-700 border'
              }`}
            >
              {profile}
            </button>
          ))}
        </div>

        <div className="mt-2 text-xs text-gray-600">
          Atlas: {reachabilityAtlasReady ? 'ready' : 'not ready'} |
          Kernel: {firmwareConfig?.capabilities?.motionKernelV2 ? 'v2' : 'legacy/degraded'}
          {firmwareConfig?.capabilities?.motionKernelDiag
            ? (
              <> | Motion kernel jitter: {motionKernelStatus.tickJitterUs} us |
                Queue underrun: {motionKernelStatus.queueUnderrun} |
                Step overrun: {motionKernelStatus.stepOverrun}
              </>
            )
            : ' | Motion kernel diagnostics unavailable'}
        </div>

        <div className="mt-3 text-xs font-semibold text-gray-700">IK Runtime</div>
        <div className="mt-1 flex flex-wrap gap-2">
          <button
            onClick={() => setIKPrimaryMode('analytic_first')}
            className={`px-3 py-1 rounded text-xs font-semibold ${
              ikPrimaryMode === 'analytic_first'
                ? 'bg-emerald-600 text-white'
                : 'bg-white text-gray-700 border'
            }`}
          >
            Analytic Primary
          </button>
          <button
            onClick={() => setIKPrimaryMode('numeric_fallback')}
            className={`px-3 py-1 rounded text-xs font-semibold ${
              ikPrimaryMode === 'numeric_fallback'
                ? 'bg-emerald-600 text-white'
                : 'bg-white text-gray-700 border'
            }`}
          >
            Numeric Primary
          </button>
        </div>
        <div className="mt-2 grid grid-cols-1 gap-1 text-xs text-gray-700">
          <label className="inline-flex items-center gap-2">
            <input
              type="checkbox"
              checked={branchLockEnabled}
              onChange={(e) => setBranchLockEnabled(e.target.checked)}
            />
            Branch continuity lock
          </label>
          <label className="inline-flex items-center gap-2">
            <input
              type="checkbox"
              checked={resolvedRateEnabled}
              onChange={(e) => setResolvedRateEnabled(e.target.checked)}
            />
            Resolved-rate tracking for Cartesian interpolation
          </label>
          <label className="inline-flex items-center gap-2">
            <input
              type="checkbox"
              checked={collisionCheckEnabled}
              onChange={(e) => setCollisionCheckEnabled(e.target.checked)}
            />
            Collision checks (capsule model)
          </label>
        </div>
      </div>

      {disabledReason && (
        <div className={`mb-3 p-2 rounded text-sm ${
          parsedTarget.error ? 'bg-red-50 text-red-700' : 'bg-yellow-50 text-yellow-800'
        }`}>
          Move disabled: {disabledReason}
        </div>
      )}

      {planningState !== 'idle' && (
        <div className={`mb-3 p-2 rounded text-sm ${
          planningState === 'failed' ? 'bg-red-50 text-red-700' :
          planningState === 'ready' ? 'bg-green-50 text-green-700' :
          'bg-blue-50 text-blue-700'
        }`}>
          <div className="font-semibold">
            Planner state: {planningState} ({planningLatencyMs.toFixed(0)} ms)
          </div>
          {planningNotes.length > 0 && (
            <div className="mt-1 text-xs">
              {planningNotes.join(' | ')}
            </div>
          )}
        </div>
      )}

      {/* IK Status */}
      {ikStatus && (
        <div className={`mb-3 p-2 rounded text-sm ${
          ikStatus.success ? 'bg-green-50 text-green-700' : 'bg-red-50 text-red-700'
        }`}>
          {ikStatus.success ? (
            <div>
              IK solution found ({ikStatus.iterations} iterations,
              pos error: {ikStatus.residualError !== undefined ? (ikStatus.residualError * 1000).toFixed(2) : '---'}mm,
              ori error: {orientationResidualDeg !== null ? orientationResidualDeg.toFixed(2) : '---'}deg)
              {ikStatus.error && (
                <span className="ml-2 text-yellow-700">[{ikStatus.error}]</span>
              )}
              {ikStatus.notes && ikStatus.notes.length > 0 && (
                <div className="mt-1 text-xs text-gray-700">
                  {ikStatus.notes.join(' | ')}
                </div>
              )}
              {ikDiagnostics && (
                <div className="mt-1 text-xs text-gray-700">
                  attempts: {ikDiagnostics.attempts} | stage: {ikDiagnostics.bestStage} |
                  residual: {Number.isFinite(ikDiagnostics.bestResidualMm)
                    ? ikDiagnostics.bestResidualMm.toFixed(2)
                    : '---'}mm |
                  branch: {ikDiagnostics.branchId} | seed: {ikDiagnostics.seedIndex}
                </div>
              )}
              <div className="mt-1 text-xs text-gray-700">
                cond: {conditionNumber !== null ? conditionNumber.toFixed(1) : '---'} |
                sigmaMin: {minSingularValue !== null ? minSingularValue.toExponential(2) : '---'} |
                weak joints: {weakJoints.length > 0 ? weakJoints.join(', ') : 'none'}
              </div>
            </div>
          ) : (
            <div>
              IK failed: {ikStatus.error}
              {ikStatus.failureCategory && (
                <span className="ml-2 text-xs uppercase tracking-wide">[{ikStatus.failureCategory}]</span>
              )}
              {ikStatus.notes && ikStatus.notes.length > 0 && (
                <div className="mt-1 text-xs text-red-800">{ikStatus.notes.join(' | ')}</div>
              )}
              {ikDiagnostics && (
                <div className="mt-1 text-xs text-red-800">
                  attempts: {ikDiagnostics.attempts} | stage: {ikDiagnostics.bestStage} |
                  residual: {Number.isFinite(ikDiagnostics.bestResidualMm)
                    ? ikDiagnostics.bestResidualMm.toFixed(2)
                    : '---'}mm |
                  branch: {ikDiagnostics.branchId} | seed: {ikDiagnostics.seedIndex}
                </div>
              )}
            </div>
          )}
        </div>
      )}

      {/* Action Buttons */}
      <div className="flex gap-2">
        <button
          onClick={handleMoveToPosition}
          disabled={!canMove}
          className="flex-1 bg-blue-600 text-white px-4 py-2 rounded font-semibold hover:bg-blue-700 disabled:bg-gray-300 disabled:cursor-not-allowed"
        >
          Move to Position
        </button>

        <button
          onClick={handleSetCurrentAsTarget}
          disabled={!isConnected}
          className="px-4 py-2 bg-gray-200 text-gray-700 rounded hover:bg-gray-300 disabled:bg-gray-100 disabled:cursor-not-allowed"
          title="Set current position as target"
        >
          Current -&gt; Target
        </button>
      </div>

      {/* Cartesian Jog */}
      <div className="mt-4 p-3 bg-gray-50 rounded border">
        <h3 className="text-sm font-semibold mb-2 text-gray-700">Cartesian Jog</h3>

        {/* Step size selectors */}
        <div className="flex flex-wrap items-center gap-2 mb-2 text-xs">
          <span className="text-gray-600">Pos:</span>
          {[1, 5, 10, 50].map(s => (
            <button
              key={s}
              onClick={() => setJogStepMm(s)}
              className={`px-2 py-0.5 rounded ${jogStepMm === s ? 'bg-blue-600 text-white' : 'bg-white border text-gray-700'}`}
            >
              {s} mm
            </button>
          ))}
          <span className="text-gray-600 ml-2">Rot:</span>
          {[1, 5, 15].map(s => (
            <button
              key={s}
              onClick={() => setJogStepDeg(s)}
              className={`px-2 py-0.5 rounded ${jogStepDeg === s ? 'bg-blue-600 text-white' : 'bg-white border text-gray-700'}`}
            >
              {s}°
            </button>
          ))}
        </div>

        {/* Frame selector */}
        <div className="flex gap-2 mb-3">
          <button
            onClick={() => setJogFrame('world')}
            className={`px-3 py-1 rounded text-xs font-semibold ${jogFrame === 'world' ? 'bg-indigo-600 text-white' : 'bg-white text-gray-700 border'}`}
          >
            World Frame
          </button>
          <button
            onClick={() => setJogFrame('tool')}
            className={`px-3 py-1 rounded text-xs font-semibold ${jogFrame === 'tool' ? 'bg-indigo-600 text-white' : 'bg-white text-gray-700 border'}`}
          >
            Tool Frame
          </button>
        </div>

        {/* Position jog buttons */}
        <div className="text-xs text-gray-500 mb-1 font-semibold">Position</div>
        <div className="grid grid-cols-3 gap-2 mb-3">
          {(['x', 'y', 'z'] as const).map(axis => (
            <div key={axis} className="flex gap-1">
              <button
                disabled={!canJog}
                onClick={() => handleJog(axis, -1)}
                className="flex-1 py-1.5 bg-gray-200 rounded text-xs font-mono hover:bg-gray-300 disabled:opacity-40 disabled:cursor-not-allowed"
              >
                −{axis.toUpperCase()}
              </button>
              <button
                disabled={!canJog}
                onClick={() => handleJog(axis, 1)}
                className="flex-1 py-1.5 bg-gray-200 rounded text-xs font-mono hover:bg-gray-300 disabled:opacity-40 disabled:cursor-not-allowed"
              >
                +{axis.toUpperCase()}
              </button>
            </div>
          ))}
        </div>

        {/* Rotation jog buttons */}
        <div className="text-xs text-gray-500 mb-1 font-semibold">Rotation</div>
        <div className="grid grid-cols-3 gap-2">
          {(['rx', 'ry', 'rz'] as const).map(axis => (
            <div key={axis} className="flex gap-1">
              <button
                disabled={!canJog}
                onClick={() => handleJog(axis, -1)}
                className="flex-1 py-1.5 bg-amber-100 rounded text-xs font-mono hover:bg-amber-200 disabled:opacity-40 disabled:cursor-not-allowed"
              >
                −{axis.toUpperCase()}
              </button>
              <button
                disabled={!canJog}
                onClick={() => handleJog(axis, 1)}
                className="flex-1 py-1.5 bg-amber-100 rounded text-xs font-mono hover:bg-amber-200 disabled:opacity-40 disabled:cursor-not-allowed"
              >
                +{axis.toUpperCase()}
              </button>
            </div>
          ))}
        </div>
      </div>

      {/* Help Text */}
      <div className="mt-3 text-xs text-gray-500">
        <p>
          <strong>Note:</strong> Inverse kinematics may not always find a solution,
          especially near workspace boundaries or singularities.
        </p>
        {queueProgress.count > 0 && (
          <p className="mt-1">
            Queue progress: point {queueProgress.pointIndex} / {queueProgress.count} (
            {queueProgress.elapsedMs} ms)
          </p>
        )}
      </div>
    </div>
  );
};
