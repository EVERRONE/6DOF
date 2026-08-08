import React, { useState } from 'react';
import { useRobotStore } from '../store/robotStore';
import { BASE_FRAME } from '../kinematics/workObject';
import { CartesianJogPanel } from './CartesianJogPanel';

const DEG = 180 / Math.PI;

/**
 * Work objects: the frames taught points are measured in.
 *
 * Without one, every point in a program describes where a thing was on the day
 * it was taught. Nudge the fixture and the whole program is wrong, with nothing
 * to do but teach it again.
 *
 * With one, the points describe the fixture and the frame describes where the
 * fixture is. Move it, re-teach three points, and the program follows.
 *
 * Three points rather than six typed numbers because touching a fixture with the
 * tool is something an operator can do accurately, and measuring its rotation
 * against the robot's base with a rule is not.
 */
export const WorkObjectPanel: React.FC = () => {
  const {
    workObjects,
    activeWorkObject,
    setActiveWorkObject,
    addWorkObject,
    renameWorkObject,
    removeWorkObject,
    currentPosition,
    waypoints,
    toolFrame,
    teachingWorkObject,
    touchedPoints,
    beginTeaching,
    touchPoint,
    undoTouch,
    cancelTeaching,
    finishTeaching
  } = useRobotStore();

  const [newName, setNewName] = useState('');

  const active = workObjects.find(o => o.id === activeWorkObject) ?? null;

  // Held in the store, not here. Teaching a frame means touching a point, then
  // moving the arm, and moving the arm used to mean leaving this tab - which
  // unmounted the panel and threw away every point touched so far.
  const teaching = teachingWorkObject;
  const touched = touchedPoints;

  const toolIsBare =
    toolFrame.xyz.x === 0 && toolFrame.xyz.y === 0 && toolFrame.xyz.z === 0;

  const STEP = [
    'the origin — the corner or datum everything is measured from',
    'a point along the +X direction — anywhere on that axis',
    'a point on the +Y side — anywhere off the X line, in the plane'
  ];

  return (
    <div className="p-4 bg-white border-b">
      <h2 className="text-lg font-bold mb-1">Work objects</h2>
      <p className="text-xs text-gray-600 mb-3">
        A named frame that points are taught in. Move the fixture, re-teach its
        three points, and every waypoint in that frame follows — no re-teaching.
      </p>

      {/* Which frame new points go into */}
      <div className="mb-3">
        <label className="block text-xs text-gray-600 mb-1">Teaching into</label>
        <select
          value={activeWorkObject ?? BASE_FRAME.id}
          onChange={e =>
            setActiveWorkObject(e.target.value === BASE_FRAME.id ? null : e.target.value)
          }
          className="w-full px-2 py-1 border rounded text-sm"
        >
          <option value={BASE_FRAME.id}>{BASE_FRAME.name} — no frame</option>
          {workObjects.map(o => (
            <option key={o.id} value={o.id}>{o.name}</option>
          ))}
        </select>
        {active && (
          <p className="mt-1 text-xs text-gray-500 font-mono">
            {(active.origin.x * 1000).toFixed(1)}, {(active.origin.y * 1000).toFixed(1)},{' '}
            {(active.origin.z * 1000).toFixed(1)} mm ·{' '}
            {(active.rpy.roll * DEG).toFixed(1)}, {(active.rpy.pitch * DEG).toFixed(1)},{' '}
            {(active.rpy.yaw * DEG).toFixed(1)}°
          </p>
        )}
      </div>

      {/* The list */}
      {workObjects.length > 0 && (
        <div className="mb-3 space-y-1">
          {workObjects.map(o => {
            const held = waypoints.filter(w => w.frame === o.id).length;
            const untaught =
              o.origin.x === 0 && o.origin.y === 0 && o.origin.z === 0 &&
              o.rpy.roll === 0 && o.rpy.pitch === 0 && o.rpy.yaw === 0;

            return (
              <div key={o.id} className="flex items-center gap-2 text-xs">
                <input
                  value={o.name}
                  onChange={e => renameWorkObject(o.id, e.target.value)}
                  className="flex-1 px-2 py-1 border rounded"
                />
                <span className="w-16 text-gray-500 text-right">
                  {held} point{held === 1 ? '' : 's'}
                </span>
                {untaught && <span className="text-amber-700">untaught</span>}
                <button
                  onClick={() => beginTeaching(o.id)}
                  className="px-2 py-1 border rounded hover:bg-gray-50"
                >
                  Teach
                </button>
                <button
                  onClick={() => removeWorkObject(o.id)}
                  className="px-2 py-1 border rounded text-red-700 hover:bg-red-50"
                  title={
                    held > 0
                      ? `${held} waypoints name this frame — they will fall back to the robot base`
                      : 'Delete'
                  }
                >
                  ✕
                </button>
              </div>
            );
          })}
        </div>
      )}

      {/* Add */}
      <div className="flex gap-2 mb-3">
        <input
          value={newName}
          onChange={e => setNewName(e.target.value)}
          onKeyDown={e => {
            if (e.key === 'Enter' && newName.trim()) {
              addWorkObject(newName);
              setNewName('');
            }
          }}
          placeholder="New work object name"
          className="flex-1 px-2 py-1 border rounded text-sm"
        />
        <button
          onClick={() => {
            addWorkObject(newName);
            setNewName('');
          }}
          className="px-3 py-1 bg-blue-600 text-white rounded text-sm hover:bg-blue-700"
        >
          Add
        </button>
      </div>

      {/* The three-point flow */}
      {teaching && (
        <div className="p-3 border rounded bg-blue-50">
          <h3 className="text-sm font-semibold mb-1">
            Teaching {workObjects.find(o => o.id === teaching)?.name}
          </h3>

          {toolIsBare && (
            <p className="text-xs text-amber-700 mb-2">
              The tool frame is still zero, so these points are the flange face and
              not the tool tip. The frame will be off by however far the real tip
              sticks out.
            </p>
          )}

          <p className="text-xs text-gray-700 mb-2">
            Move the tool tip onto {STEP[Math.min(touched.length, 2)]}, then touch.
          </p>

          <ol className="text-xs mb-2 space-y-0.5">
            {STEP.map((label, i) => (
              <li key={i} className={touched[i] ? 'text-gray-700' : 'text-gray-400'}>
                {i + 1}. {touched[i]
                  ? <span className="font-mono">
                      {(touched[i].x * 1000).toFixed(1)}, {(touched[i].y * 1000).toFixed(1)},{' '}
                      {(touched[i].z * 1000).toFixed(1)} mm
                    </span>
                  : label}
              </li>
            ))}
          </ol>

          <div className="flex gap-2">
            <button
              onClick={touchPoint}
              disabled={!currentPosition || touched.length >= 3}
              className="flex-1 px-3 py-1 bg-blue-600 text-white rounded text-sm hover:bg-blue-700 disabled:bg-gray-300"
            >
              Touch point {Math.min(touched.length + 1, 3)}
            </button>
            <button
              onClick={finishTeaching}
              disabled={touched.length < 3}
              className="px-3 py-1 bg-green-700 text-white rounded text-sm hover:bg-green-800 disabled:bg-gray-300"
            >
              Set frame
            </button>
            <button
              onClick={cancelTeaching}
              className="px-3 py-1 border rounded text-sm hover:bg-gray-50"
            >
              Cancel
            </button>
          </div>

          {touched.length > 0 && (
            <button
              onClick={undoTouch}
              className="mt-1 text-xs text-gray-600 underline"
            >
              Undo last touch
            </button>
          )}

          {/* The arm has to be moved between touches, and having to change tabs
              to do it is what made this workflow awkward in the first place. */}
          <div className="mt-3 -mx-3 -mb-3 border-t bg-white rounded-b">
            <CartesianJogPanel />
          </div>
        </div>
      )}
    </div>
  );
};
