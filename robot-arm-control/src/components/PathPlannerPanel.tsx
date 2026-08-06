import React, { useState, useRef, useCallback } from 'react';
import { useRobotStore } from '../store/robotStore';
import { ExecutionState, Waypoint } from '../motion/types';
import { ShapePlane } from '../motion/Shapes';

/**
 * PathPlannerPanel
 *
 * UI for waypoint management, trajectory planning, and execution control.
 * Features:
 * - Waypoint list with teach, add, remove, reorder
 * - Speed and interpolation settings
 * - Plan preview with duration/distance info
 * - Execute/Pause/Resume/Stop controls
 * - Progress bar during execution
 * - Path save/load (JSON files)
 */
export const PathPlannerPanel: React.FC = () => {
  const {
    waypoints,
    trajectory,
    executionState,
    executionProgress,
    plannerConfig,
    connectionStatus,
    teachCurrentPosition,
    removeWaypoint,
    reorderWaypoints,
    updateWaypoint,
    clearWaypoints,
    planTrajectory,
    executeTrajectory,
    pauseExecution,
    resumeExecution,
    cancelExecution,
    updatePlannerConfig,
    exportPath,
    importPath,
    firmwareStatus,
    addCircle,
    toolLocked
  } = useRobotStore();

  const [pathName, setPathName] = useState('');
  const [circleRadius, setCircleRadius] = useState('30');
  const [circlePlane, setCirclePlane] = useState<ShapePlane>('XY');
  const [circleClockwise, setCircleClockwise] = useState(false);
  const [loopCount, setLoopCount] = useState(0);
  const fileInputRef = useRef<HTMLInputElement>(null);

  const isExecuting = executionState === ExecutionState.EXECUTING;
  const isPaused = executionState === ExecutionState.PAUSED;
  const isIdle = executionState === ExecutionState.IDLE || executionState === ExecutionState.COMPLETED;
  const isConnected = connectionStatus === 'connected';

  // Why the arm cannot be asked to run a path right now, in the order that
  // matters. A path is a list of absolute joint angles, so an untrusted datum
  // makes every one of them mean something else - saying so beats a greyed-out
  // button with no explanation.
  const blockedReason = !isConnected
    ? 'Not connected'
    : waypoints.length === 0
      ? 'Teach at least one waypoint'
      : firmwareStatus && !firmwareStatus.positionTrusted
        ? 'Home the arm first — its position is not trusted, so the path would run from a datum that does not exist'
        : firmwareStatus && !firmwareStatus.enabled
          ? 'Motors are off'
          : null;

  // Teach current position as waypoint
  const handleTeach = useCallback(() => {
    teachCurrentPosition();
  }, [teachCurrentPosition]);

  // Move waypoint up in the list
  const handleMoveUp = useCallback((index: number) => {
    if (index > 0) {
      reorderWaypoints(index, index - 1);
    }
  }, [reorderWaypoints]);

  // Move waypoint down in the list
  const handleMoveDown = useCallback((index: number) => {
    if (index < waypoints.length - 1) {
      reorderWaypoints(index, index + 1);
    }
  }, [reorderWaypoints, waypoints.length]);

  // Plan trajectory
  const handlePlan = useCallback(() => {
    planTrajectory();
  }, [planTrajectory]);

  // Execute trajectory
  const handleExecute = useCallback(async () => {
    if (!trajectory) {
      planTrajectory();
      // Wait a tick for planning to complete
      setTimeout(() => {
        executeTrajectory();
      }, 50);
    } else {
      await executeTrajectory();
    }
  }, [trajectory, planTrajectory, executeTrajectory]);

  // Save path to file
  const handleSave = useCallback(() => {
    const name = pathName.trim() || `Path_${new Date().toISOString().slice(0, 10)}`;
    const savedPath = exportPath(name);

    const json = JSON.stringify(savedPath, null, 2);
    const blob = new Blob([json], { type: 'application/json' });
    const url = URL.createObjectURL(blob);

    const a = document.createElement('a');
    a.href = url;
    a.download = `${name.replace(/\s+/g, '_')}.json`;
    a.click();

    URL.revokeObjectURL(url);
  }, [pathName, exportPath]);

  // Load path from file
  const handleLoad = useCallback((event: React.ChangeEvent<HTMLInputElement>) => {
    const file = event.target.files?.[0];
    if (!file) return;

    const reader = new FileReader();
    reader.onload = (e) => {
      try {
        const data = JSON.parse(e.target?.result as string);
        const success = importPath(data);
        if (!success) {
          alert('Invalid path file format');
        }
      } catch {
        alert('Failed to parse path file');
      }
    };
    reader.readAsText(file);

    // Reset input so same file can be loaded again
    event.target.value = '';
  }, [importPath]);

  // Update loop count
  const handleLoopChange = useCallback((count: number) => {
    setLoopCount(count);
    updatePlannerConfig({ loopCount: count });
  }, [updatePlannerConfig]);

  return (
    <div className="p-4 border-b">
      <h2 className="text-lg font-bold mb-3">Path Planner</h2>

      {/* Waypoint List */}
      <div className="mb-4">
        <div className="flex items-center justify-between mb-2">
          <h3 className="text-sm font-semibold text-gray-700">
            Waypoints ({waypoints.length})
          </h3>
          <div className="flex gap-1">
            <button
              onClick={handleTeach}
              className="px-2 py-1 text-xs bg-blue-500 text-white rounded hover:bg-blue-600"
              title="Capture current robot position as waypoint"
            >
              + Teach
            </button>
            {waypoints.length > 0 && (
              <button
                onClick={clearWaypoints}
                disabled={isExecuting}
                className="px-2 py-1 text-xs bg-red-100 text-red-600 rounded hover:bg-red-200 disabled:opacity-50"
              >
                Clear All
              </button>
            )}
          </div>
        </div>

        {/* Figures. Described rather than taught: teaching a circle by hand
            means dozens of points, each slightly wrong. The generator checks
            the arm can reach the whole figure before adding anything, and says
            so in the console when it cannot. */}
        <div className="mb-3 p-2 bg-gray-50 rounded">
          <div className="flex items-center gap-2 mb-2">
            <span className="text-xs font-semibold text-gray-700">Circle</span>
            <span className="text-xs text-gray-500">
              centred where the tool is now
              {toolLocked ? ', tool held' : ''}
            </span>
          </div>
          <div className="flex items-end gap-2 flex-wrap">
            <label className="text-xs text-gray-600">
              Radius
              <div className="flex items-center gap-1">
                <input
                  type="number"
                  min="1"
                  step="1"
                  value={circleRadius}
                  onChange={e => setCircleRadius(e.target.value)}
                  disabled={isExecuting}
                  className="w-16 px-1 py-0.5 border rounded text-right font-mono text-sm disabled:bg-gray-100"
                />
                <span className="text-gray-500">mm</span>
              </div>
            </label>

            <label className="text-xs text-gray-600">
              Plane
              <select
                value={circlePlane}
                onChange={e => setCirclePlane(e.target.value as ShapePlane)}
                disabled={isExecuting}
                className="block px-1 py-0.5 border rounded text-sm disabled:bg-gray-100"
              >
                <option value="XY">XY (flat)</option>
                <option value="XZ">XZ (upright)</option>
                <option value="YZ">YZ (upright)</option>
              </select>
            </label>

            <label className="flex items-center gap-1 text-xs text-gray-600 pb-1">
              <input
                type="checkbox"
                checked={circleClockwise}
                onChange={e => setCircleClockwise(e.target.checked)}
                disabled={isExecuting}
              />
              Clockwise
            </label>

            <button
              onClick={() =>
                addCircle({
                  radius: Number(circleRadius) / 1000,
                  plane: circlePlane,
                  clockwise: circleClockwise
                })
              }
              disabled={isExecuting || !isConnected}
              className="px-2 py-1 text-xs bg-purple-600 text-white rounded hover:bg-purple-700 disabled:opacity-50"
            >
              + Add circle
            </button>
          </div>
        </div>

        {/* Waypoint items */}
        <div className="space-y-1 max-h-48 overflow-y-auto">
          {waypoints.length === 0 ? (
            <div className="text-sm text-gray-400 italic py-2">
              No waypoints. Click "Teach" to add the current position.
            </div>
          ) : (
            waypoints.map((wp, index) => (
              <WaypointItem
                key={wp.id}
                waypoint={wp}
                index={index}
                total={waypoints.length}
                onRemove={() => removeWaypoint(wp.id)}
                onMoveUp={() => handleMoveUp(index)}
                onMoveDown={() => handleMoveDown(index)}
                onSpeedChange={(speed) => updateWaypoint(wp.id, { speed })}
                onLabelChange={(label) => updateWaypoint(wp.id, { label })}
                disabled={isExecuting}
              />
            ))
          )}
        </div>
      </div>

      {/* Settings */}
      <div className="mb-4 border-t pt-3">
        <h3 className="text-sm font-semibold text-gray-700 mb-2">Settings</h3>

        <div className="grid grid-cols-2 gap-2">
          {/* Interpolation mode */}
          <div>
            <label className="text-xs text-gray-500">Interpolation</label>
            <select
              value={plannerConfig.interpolationMode}
              onChange={(e) => updatePlannerConfig({
                interpolationMode: e.target.value as 'joint' | 'linear'
              })}
              disabled={isExecuting}
              className="w-full text-sm border rounded px-2 py-1"
            >
              <option value="joint">Joint Space</option>
              <option value="linear">Cartesian Linear</option>
            </select>
          </div>

          {/* Default speed */}
          <div>
            <label className="text-xs text-gray-500">Default Speed</label>
            <div className="flex items-center gap-1">
              <input
                type="number"
                value={plannerConfig.defaultSpeed}
                onChange={(e) => updatePlannerConfig({
                  defaultSpeed: Math.max(1, parseInt(e.target.value) || 50)
                })}
                disabled={isExecuting}
                className="w-full text-sm border rounded px-2 py-1"
                min={1}
                max={200}
              />
              <span className="text-xs text-gray-400 whitespace-nowrap">mm/s</span>
            </div>
          </div>

          {/* Max joint speed */}
          <div>
            <label className="text-xs text-gray-500">Joint Speed</label>
            <div className="flex items-center gap-1">
              <input
                type="number"
                value={plannerConfig.maxJointSpeed}
                onChange={(e) => updatePlannerConfig({
                  maxJointSpeed: Math.max(1, parseInt(e.target.value) || 60)
                })}
                disabled={isExecuting}
                className="w-full text-sm border rounded px-2 py-1"
                min={1}
                max={180}
              />
              <span className="text-xs text-gray-400 whitespace-nowrap">deg/s</span>
            </div>
          </div>

          {/* Loop count */}
          <div>
            <label className="text-xs text-gray-500">Loop Count</label>
            <div className="flex items-center gap-1">
              <input
                type="number"
                value={loopCount}
                onChange={(e) => handleLoopChange(Math.max(0, parseInt(e.target.value) || 0))}
                disabled={isExecuting}
                className="w-full text-sm border rounded px-2 py-1"
                min={0}
                max={1000}
              />
              <span className="text-xs text-gray-400 whitespace-nowrap">
                {loopCount === 0 ? '(1x)' : `(${loopCount}x)`}
              </span>
            </div>
          </div>
        </div>
      </div>

      {/* Plan & Trajectory Info */}
      <div className="mb-4 border-t pt-3">
        <div className="flex items-center gap-2 mb-2">
          <button
            onClick={handlePlan}
            disabled={waypoints.length === 0 || isExecuting}
            className="px-3 py-1 text-sm bg-gray-700 text-white rounded hover:bg-gray-800 disabled:opacity-50"
          >
            Plan Path
          </button>

          {trajectory && (
            <div className="text-xs text-gray-600">
              <span>{trajectory.segments.length} segments</span>
              <span className="mx-1">|</span>
              <span>{trajectory.totalDuration.toFixed(1)}s</span>
              <span className="mx-1">|</span>
              <span>{(trajectory.totalDistance * 1000).toFixed(0)}mm</span>
              <span className="mx-1">|</span>
              <span>{trajectory.pointCount} pts</span>
            </div>
          )}
        </div>

        {/* A dropped waypoint or an out-of-reach stretch changes the path the arm
            will actually run, so it has to be said out loud rather than logged. */}
        {trajectory && trajectory.skippedWaypoints.length > 0 && (
          <div className="mt-2 p-2 rounded text-xs bg-red-50 text-red-700">
            Left out of the plan, no IK solution:{' '}
            <span className="font-semibold">
              {trajectory.skippedWaypoints.join(', ')}
            </span>
            . The arm will move straight past those positions.
          </div>
        )}

        {trajectory && trajectory.unreachableSamples > 0 && (
          <div className="mt-2 p-2 rounded text-xs bg-amber-50 text-amber-800">
            {trajectory.unreachableSamples} point
            {trajectory.unreachableSamples === 1 ? '' : 's'} along the straight
            line could not be reached. The tool will cut the corner there instead
            of following the line.
          </div>
        )}
      </div>

      {/* Execution Controls */}
      <div className="mb-4 border-t pt-3">
        <h3 className="text-sm font-semibold text-gray-700 mb-2">Execution</h3>

        {isIdle && blockedReason && (
          <p className="mb-2 text-xs text-amber-700">{blockedReason}</p>
        )}

        <div className="flex gap-2 mb-2">
          {isIdle && (
            <button
              onClick={handleExecute}
              disabled={blockedReason !== null}
              title={blockedReason ?? 'Run the planned path'}
              className="flex-1 px-3 py-2 text-sm bg-green-500 text-white rounded hover:bg-green-600 disabled:opacity-50 font-medium"
            >
              Execute
            </button>
          )}

          {isExecuting && (
            <button
              onClick={pauseExecution}
              className="flex-1 px-3 py-2 text-sm bg-yellow-500 text-white rounded hover:bg-yellow-600 font-medium"
            >
              Pause
            </button>
          )}

          {isPaused && (
            <button
              onClick={resumeExecution}
              className="flex-1 px-3 py-2 text-sm bg-green-500 text-white rounded hover:bg-green-600 font-medium"
            >
              Resume
            </button>
          )}

          {(isExecuting || isPaused) && (
            <button
              onClick={cancelExecution}
              className="px-3 py-2 text-sm bg-red-500 text-white rounded hover:bg-red-600 font-medium"
            >
              Stop
            </button>
          )}
        </div>

        {/* Progress bar */}
        {(isExecuting || isPaused) && (
          <div className="mb-2">
            <div className="w-full bg-gray-200 rounded-full h-2.5">
              <div
                className={`h-2.5 rounded-full transition-all ${
                  isPaused ? 'bg-yellow-500' : 'bg-green-500'
                }`}
                style={{ width: `${executionProgress.overallProgress}%` }}
              />
            </div>
            <div className="flex justify-between text-xs text-gray-500 mt-1">
              <span>{executionProgress.overallProgress.toFixed(0)}%</span>
              <span>
                Seg {executionProgress.currentSegment + 1}/{executionProgress.totalSegments}
              </span>
              <span>
                {executionProgress.elapsedTime.toFixed(1)}s /
                {(executionProgress.elapsedTime + executionProgress.estimatedTimeRemaining).toFixed(1)}s
              </span>
            </div>
          </div>
        )}

        {executionState === ExecutionState.COMPLETED && (
          <div className="text-sm text-green-600 font-medium">
            Path execution completed successfully.
          </div>
        )}

        {executionState === ExecutionState.ERROR && (
          <div className="text-sm text-red-600 font-medium">
            Execution error. Check console for details.
          </div>
        )}

        {!isConnected && (
          <div className="text-xs text-orange-500 mt-1">
            Connect to robot to execute paths.
          </div>
        )}
      </div>

      {/* Save/Load */}
      <div className="border-t pt-3">
        <h3 className="text-sm font-semibold text-gray-700 mb-2">Save / Load</h3>

        <div className="flex gap-2 mb-2">
          <input
            type="text"
            value={pathName}
            onChange={(e) => setPathName(e.target.value)}
            placeholder="Path name..."
            className="flex-1 text-sm border rounded px-2 py-1"
          />
          <button
            onClick={handleSave}
            disabled={waypoints.length === 0}
            className="px-3 py-1 text-sm bg-blue-100 text-blue-600 rounded hover:bg-blue-200 disabled:opacity-50"
          >
            Save
          </button>
        </div>

        <div className="flex gap-2">
          <button
            onClick={() => fileInputRef.current?.click()}
            disabled={isExecuting}
            className="px-3 py-1 text-sm bg-gray-100 text-gray-600 rounded hover:bg-gray-200 disabled:opacity-50"
          >
            Load Path
          </button>
          <input
            ref={fileInputRef}
            type="file"
            accept=".json"
            onChange={handleLoad}
            className="hidden"
          />
        </div>
      </div>
    </div>
  );
};

/**
 * Individual waypoint row in the list
 */
interface WaypointItemProps {
  waypoint: Waypoint;
  index: number;
  total: number;
  onRemove: () => void;
  onMoveUp: () => void;
  onMoveDown: () => void;
  onSpeedChange: (speed: number) => void;
  onLabelChange: (label: string) => void;
  disabled: boolean;
}

const WaypointItem: React.FC<WaypointItemProps> = ({
  waypoint,
  index,
  total,
  onRemove,
  onMoveUp,
  onMoveDown,
  onSpeedChange,
  onLabelChange,
  disabled
}) => {
  const [isEditing, setIsEditing] = useState(false);

  return (
    <div className="flex items-center gap-1 p-1.5 bg-gray-50 rounded text-xs group">
      {/* Index badge */}
      <span className="w-5 h-5 flex items-center justify-center bg-blue-500 text-white rounded-full text-xs font-bold flex-shrink-0">
        {index + 1}
      </span>

      {/* Position info */}
      <div className="flex-1 min-w-0">
        {isEditing ? (
          <input
            type="text"
            value={waypoint.label || ''}
            onChange={(e) => onLabelChange(e.target.value)}
            onBlur={() => setIsEditing(false)}
            onKeyDown={(e) => e.key === 'Enter' && setIsEditing(false)}
            className="w-full text-xs border rounded px-1"
            autoFocus
          />
        ) : (
          <div
            className="truncate cursor-pointer"
            onClick={() => !disabled && setIsEditing(true)}
            title={`X:${(waypoint.position.x * 1000).toFixed(1)} Y:${(waypoint.position.y * 1000).toFixed(1)} Z:${(waypoint.position.z * 1000).toFixed(1)}`}
          >
            <span className="font-medium">{waypoint.label || `WP ${index + 1}`}</span>
            <span className="text-gray-400 ml-1">
              ({(waypoint.position.x * 1000).toFixed(0)},
              {(waypoint.position.y * 1000).toFixed(0)},
              {(waypoint.position.z * 1000).toFixed(0)})
            </span>
          </div>
        )}
      </div>

      {/* Speed */}
      <input
        type="number"
        value={waypoint.speed}
        onChange={(e) => onSpeedChange(Math.max(1, parseInt(e.target.value) || 50))}
        disabled={disabled}
        className="w-12 text-xs border rounded px-1 py-0.5 text-center"
        min={1}
        max={200}
        title="Feed rate for the move to this waypoint: mm/s in linear mode, deg/s in joint mode"
      />

      {/* Reorder buttons */}
      <button
        onClick={onMoveUp}
        disabled={disabled || index === 0}
        className="p-0.5 text-gray-400 hover:text-gray-600 disabled:opacity-30"
        title="Move up"
      >
        &#9650;
      </button>
      <button
        onClick={onMoveDown}
        disabled={disabled || index === total - 1}
        className="p-0.5 text-gray-400 hover:text-gray-600 disabled:opacity-30"
        title="Move down"
      >
        &#9660;
      </button>

      {/* Remove button */}
      <button
        onClick={onRemove}
        disabled={disabled}
        className="p-0.5 text-red-400 hover:text-red-600 disabled:opacity-30"
        title="Remove waypoint"
      >
        &#10005;
      </button>
    </div>
  );
};
