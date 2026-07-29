import React, { useState, useEffect, useMemo } from 'react';
import { useRobotStore } from '../store/robotStore';
import { Vector3 } from '../kinematics/types';
import { computeWorkspaceBounds } from '../kinematics/InverseKinematics';

/**
 * Cartesian Control Panel Component
 *
 * Allows user to control robot end-effector in Cartesian coordinates (X, Y, Z)
 * Uses inverse kinematics to convert XYZ position to joint angles
 */
export const CartesianControlPanel: React.FC = () => {
  const {
    currentPosition,
    setTargetPosition,
    moveToPosition,
    motorsEnabled,
    connectionStatus,
    ikStatus
  } = useRobotStore();

  // Local state for input fields
  const [inputX, setInputX] = useState<string>('0');
  const [inputY, setInputY] = useState<string>('0');
  const [inputZ, setInputZ] = useState<string>('0');

  // Workspace limits, derived from the actual kinematic model and the
  // mechanical joint limits rather than guessed. This is the axis-aligned outer
  // bound of the reachable set, so a point inside the box is not guaranteed to
  // be reachable - the IK result is the authority on that.
  const WORKSPACE_LIMITS = useMemo(() => {
    const bounds = computeWorkspaceBounds(9);
    return {
      x: { min: bounds.min.x, max: bounds.max.x },
      y: { min: bounds.min.y, max: bounds.max.y },
      z: { min: bounds.min.z, max: bounds.max.z }
    };
  }, []);

  // Update input fields when current position changes
  useEffect(() => {
    if (currentPosition) {
      setInputX((currentPosition.x * 1000).toFixed(1)); // Convert to mm
      setInputY((currentPosition.y * 1000).toFixed(1));
      setInputZ((currentPosition.z * 1000).toFixed(1));
    }
  }, [currentPosition]);

  const handleMoveToPosition = async () => {
    const x = parseFloat(inputX) / 1000; // Convert mm to m
    const y = parseFloat(inputY) / 1000;
    const z = parseFloat(inputZ) / 1000;

    // Validate input
    if (isNaN(x) || isNaN(y) || isNaN(z)) {
      alert('Invalid input. Please enter numeric values.');
      return;
    }

    // Check workspace limits
    if (
      x < WORKSPACE_LIMITS.x.min || x > WORKSPACE_LIMITS.x.max ||
      y < WORKSPACE_LIMITS.y.min || y > WORKSPACE_LIMITS.y.max ||
      z < WORKSPACE_LIMITS.z.min || z > WORKSPACE_LIMITS.z.max
    ) {
      const range = (axis: { min: number; max: number }) =>
        `${(axis.min * 1000).toFixed(0)} to ${(axis.max * 1000).toFixed(0)} mm`;
      alert(
        `Position outside the reachable workspace!\n` +
        `X: ${range(WORKSPACE_LIMITS.x)}\n` +
        `Y: ${range(WORKSPACE_LIMITS.y)}\n` +
        `Z: ${range(WORKSPACE_LIMITS.z)}`
      );
      return;
    }

    const position: Vector3 = { x, y, z };

    // Publish the target so the 3D viewer can draw its target marker. Without
    // this the marker existed but was never given a position, so it stayed
    // permanently hidden.
    setTargetPosition(position);

    await moveToPosition(position);
  };

  const handleSetCurrentAsTarget = () => {
    if (currentPosition) {
      setInputX((currentPosition.x * 1000).toFixed(1));
      setInputY((currentPosition.y * 1000).toFixed(1));
      setInputZ((currentPosition.z * 1000).toFixed(1));
    }
  };

  const isConnected = connectionStatus === 'connected';

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
          {/* X Input */}
          <div>
            <label className="block text-xs text-gray-600 mb-1">X</label>
            <input
              type="number"
              value={inputX}
              onChange={(e) => setInputX(e.target.value)}
              className="w-full px-2 py-1 border rounded text-sm font-mono"
              step="1"
              disabled={!isConnected}
            />
            <span className="text-xs text-gray-400">
              {(WORKSPACE_LIMITS.x.min * 1000).toFixed(0)} to {(WORKSPACE_LIMITS.x.max * 1000).toFixed(0)}
            </span>
          </div>

          {/* Y Input */}
          <div>
            <label className="block text-xs text-gray-600 mb-1">Y</label>
            <input
              type="number"
              value={inputY}
              onChange={(e) => setInputY(e.target.value)}
              className="w-full px-2 py-1 border rounded text-sm font-mono"
              step="1"
              disabled={!isConnected}
            />
            <span className="text-xs text-gray-400">
              {(WORKSPACE_LIMITS.y.min * 1000).toFixed(0)} to {(WORKSPACE_LIMITS.y.max * 1000).toFixed(0)}
            </span>
          </div>

          {/* Z Input */}
          <div>
            <label className="block text-xs text-gray-600 mb-1">Z</label>
            <input
              type="number"
              value={inputZ}
              onChange={(e) => setInputZ(e.target.value)}
              className="w-full px-2 py-1 border rounded text-sm font-mono"
              step="1"
              disabled={!isConnected}
            />
            <span className="text-xs text-gray-400">
              {(WORKSPACE_LIMITS.z.min * 1000).toFixed(0)} to {(WORKSPACE_LIMITS.z.max * 1000).toFixed(0)}
            </span>
          </div>
        </div>
      </div>

      {/* IK Status */}
      {ikStatus && (
        <div className={`mb-3 p-2 rounded text-sm ${
          ikStatus.success ? 'bg-green-50 text-green-700' : 'bg-red-50 text-red-700'
        }`}>
          {ikStatus.success ? (
            <div>
              ✓ IK Solution found ({ikStatus.iterations} iterations, error: {(ikStatus.residualError! * 1000).toFixed(2)}mm)
            </div>
          ) : (
            <div>✗ {ikStatus.error}</div>
          )}
        </div>
      )}

      {/* Action Buttons */}
      <div className="flex gap-2">
        <button
          onClick={handleMoveToPosition}
          disabled={!isConnected || !motorsEnabled}
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
          Current → Target
        </button>
      </div>

      {/* Help Text */}
      <div className="mt-3 text-xs text-gray-500">
        <p>
          <strong>Note:</strong> the ranges above are the outer bounds of the
          reachable workspace, so a point inside them can still be unreachable -
          the arm is tightly limited on J1 and J2. If IK reports a residual, the
          target is out of reach from the current arm configuration.
        </p>
      </div>
    </div>
  );
};
