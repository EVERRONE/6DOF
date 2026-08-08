import React, { useState } from 'react';
import { useRobotStore } from '../store/robotStore';
import { DEFAULT_TOOL_FRAME } from '../kinematics/robotModel';
import { CartesianJogPanel } from './CartesianJogPanel';

const RAD = Math.PI / 180;

/**
 * Where the tool is, relative to the flange.
 *
 * Two measurements, answering different questions.
 *
 * The **offset** says where the tip is. Until it is set, "the position" is the
 * flange origin, so every wrist rotation swings the real tip through an arc
 * nothing in the software can see: a 100 mm tool turned 10 degrees moves its tip
 * 17 mm while the reported position does not change at all.
 *
 * The **rotation** says which way the tool points. Until it is set, the tool's
 * axes are assumed to be the flange's, so "hold the tool level" means "hold the
 * flange level" - the same thing only if the tool was bolted on perfectly
 * square. It never is, and this is the only place that difference can go.
 *
 * Fields are local until Apply. They used to be bound straight to the store,
 * which rebuilt them on every position report - twenty times a second - so each
 * keystroke was overwritten before the next one landed.
 */
export const ToolFramePanel: React.FC = () => {
  const {
    toolFrame,
    setToolFrame,
    resetToolFrame,
    currentPosition,
    calibratingTool,
    toolTouches,
    toolCalibration,
    beginToolCalibration,
    touchToolPoint,
    undoToolTouch,
    cancelToolCalibration,
    finishToolCalibration
  } = useRobotStore();

  // Millimetres and degrees in the UI, metres and radians in the model. The
  // model is in SI because the kinematics are; nobody measures a tool in metres.
  const [x, setX] = useState((toolFrame.xyz.x * 1000).toFixed(1));
  const [y, setY] = useState((toolFrame.xyz.y * 1000).toFixed(1));
  const [z, setZ] = useState((toolFrame.xyz.z * 1000).toFixed(1));
  const [roll, setRoll] = useState((toolFrame.rpy.roll / RAD).toFixed(2));
  const [pitch, setPitch] = useState((toolFrame.rpy.pitch / RAD).toFixed(2));
  const [yaw, setYaw] = useState((toolFrame.rpy.yaw / RAD).toFixed(2));
  const [problem, setProblem] = useState<string | null>(null);

  const seedFrom = (frame: typeof toolFrame) => {
    setX((frame.xyz.x * 1000).toFixed(1));
    setY((frame.xyz.y * 1000).toFixed(1));
    setZ((frame.xyz.z * 1000).toFixed(1));
    setRoll((frame.rpy.roll / RAD).toFixed(2));
    setPitch((frame.rpy.pitch / RAD).toFixed(2));
    setYaw((frame.rpy.yaw / RAD).toFixed(2));
  };

  const apply = () => {
    const values = [x, y, z, roll, pitch, yaw].map(v => Number(v));
    if (!values.every(Number.isFinite)) {
      setProblem('Every field has to be a number');
      return;
    }
    // A tool the length of the arm is a typo, not a tool, and it would move the
    // whole reachable workspace with it.
    if (Math.hypot(values[0], values[1], values[2]) > 500) {
      setProblem('That offset is more than 500 mm from the flange — check the units');
      return;
    }
    setProblem(null);

    setToolFrame({
      xyz: { x: values[0] / 1000, y: values[1] / 1000, z: values[2] / 1000 },
      rpy: { roll: values[3] * RAD, pitch: values[4] * RAD, yaw: values[5] * RAD }
    });
  };

  const isDefault =
    toolFrame.xyz.x === DEFAULT_TOOL_FRAME.xyz.x &&
    toolFrame.xyz.y === DEFAULT_TOOL_FRAME.xyz.y &&
    toolFrame.xyz.z === DEFAULT_TOOL_FRAME.xyz.z &&
    toolFrame.rpy.roll === DEFAULT_TOOL_FRAME.rpy.roll &&
    toolFrame.rpy.pitch === DEFAULT_TOOL_FRAME.rpy.pitch &&
    toolFrame.rpy.yaw === DEFAULT_TOOL_FRAME.rpy.yaw;

  const reach = Math.hypot(toolFrame.xyz.x, toolFrame.xyz.y, toolFrame.xyz.z) * 1000;

  const modelSource = () => {
    const m = (v: number) => v.toFixed(5);
    const r = (v: number) => v.toFixed(6);
    return (
      'export const DEFAULT_TOOL_FRAME: ToolFrame = {\n' +
      `  xyz: { x: ${m(toolFrame.xyz.x)}, y: ${m(toolFrame.xyz.y)}, z: ${m(toolFrame.xyz.z)} },\n` +
      `  rpy: { roll: ${r(toolFrame.rpy.roll)}, pitch: ${r(toolFrame.rpy.pitch)}, ` +
      `yaw: ${r(toolFrame.rpy.yaw)} }\n};`
    );
  };

  const field = (
    label: string,
    unit: string,
    value: string,
    onChange: (v: string) => void
  ) => (
    <div key={label}>
      <label className="block text-xs text-gray-600 mb-1">
        {label} <span className="text-gray-400">{unit}</span>
      </label>
      <input
        type="number"
        value={value}
        onChange={e => onChange(e.target.value)}
        onKeyDown={e => {
          if (e.key === 'Enter') apply();
        }}
        className="w-full px-2 py-1 border rounded text-sm font-mono"
        step="0.1"
      />
    </div>
  );

  return (
    <div className="p-4 bg-white border-b">
      <h2 className="text-lg font-bold mb-1">Tool frame</h2>
      <p className="text-xs text-gray-600 mb-3">
        Where the tool sits relative to the flange, and which way it points. Until
        this is measured the software treats the flange face as the tool: the
        reported position is the flange origin, and <em>hold the tool level</em>{' '}
        means <em>hold the flange level</em>.
      </p>

      {/* Finding the offset by touching, rather than by measuring.
          Typing it in needs somebody who knows which way frame 6's X, Y and Z
          point, and that is not visible on the machine - it falls out of a chain
          of URDF rotations. Touching one point from four directions asks nothing
          of the operator but the touching. */}
      <div className="mb-3 p-3 border rounded bg-blue-50">
        <h3 className="text-sm font-semibold mb-1">Find the tip by touching</h3>

        {!calibratingTool ? (
          <>
            <p className="text-xs text-gray-700 mb-2">
              Put something pointed on the bench — a nail, a sharpened screw.
              Touch its tip with the tool&apos;s tip four times, approaching from
              genuinely different directions each time. The point never moves, so
              the only unknown left is where your tip sits relative to the flange.
            </p>
            <button
              onClick={beginToolCalibration}
              className="px-3 py-1 bg-blue-600 text-white rounded text-sm hover:bg-blue-700"
            >
              Start
            </button>
          </>
        ) : (
          <>
            <p className="text-xs text-gray-700 mb-2">
              Move the tool tip onto the <strong>same</strong> point as last time,
              from a different angle, then touch.
            </p>

            <ol className="text-xs mb-2 space-y-0.5">
              {[0, 1, 2, 3].map(i => (
                <li key={i} className={toolTouches[i] ? 'text-gray-700' : 'text-gray-400'}>
                  {i + 1}. {toolTouches[i] ? 'recorded' : 'not yet'}
                </li>
              ))}
            </ol>

            <div className="flex gap-2 mb-1">
              <button
                onClick={touchToolPoint}
                className="flex-1 px-3 py-1 bg-blue-600 text-white rounded text-sm hover:bg-blue-700"
              >
                Touch {Math.min(toolTouches.length + 1, 4)}
              </button>
              <button
                onClick={finishToolCalibration}
                disabled={toolTouches.length < 4}
                className="px-3 py-1 bg-green-700 text-white rounded text-sm hover:bg-green-800 disabled:bg-gray-300"
              >
                Set tool
              </button>
              <button
                onClick={cancelToolCalibration}
                className="px-3 py-1 border rounded text-sm hover:bg-gray-50"
              >
                Cancel
              </button>
            </div>

            {toolTouches.length > 0 && (
              <button onClick={undoToolTouch} className="text-xs text-gray-600 underline">
                Undo last touch
              </button>
            )}

            <div className="mt-3 -mx-3 -mb-3 border-t bg-white rounded-b">
              <CartesianJogPanel />
            </div>
          </>
        )}

        {toolCalibration && !toolCalibration.ok && (
          <p className="mt-2 text-xs text-red-700">{toolCalibration.reason}</p>
        )}
        {toolCalibration && toolCalibration.ok && (
          <div
            className={`mt-2 text-xs p-2 rounded ${
              toolCalibration.verdict.kind === 'good'
                ? 'bg-green-50 text-green-900'
                : toolCalibration.verdict.kind === 'poorly-conditioned'
                  ? 'bg-amber-50 text-amber-900'
                  : 'bg-red-50 text-red-900'
            }`}
          >
            <p className="font-semibold mb-1">
              {toolCalibration.verdict.kind === 'good'
                ? 'Measured'
                : toolCalibration.verdict.kind === 'poorly-conditioned'
                  ? 'Usable, but soft'
                  : 'Not a measurement — nothing applied'}
            </p>
            <p>{toolCalibration.verdict.note}</p>
            <p className="mt-1 text-gray-600">
              The <strong>{toolCalibration.residualMm.toFixed(2)} mm</strong> is your
              aim, the arm&apos;s repeatability and the model&apos;s fidelity added
              together. Nothing else measured on this arm reports any of them.
            </p>
          </div>
        )}
      </div>

      <div className="mb-3">
        <h3 className="text-sm font-semibold mb-2 text-gray-700">
          Offset — where the tip is
        </h3>
        <div className="grid grid-cols-3 gap-3">
          {field('X', 'mm', x, setX)}
          {field('Y', 'mm', y, setY)}
          {field('Z', 'mm', z, setZ)}
        </div>
      </div>

      <div className="mb-3">
        <h3 className="text-sm font-semibold mb-2 text-gray-700">
          Rotation — which way it points
        </h3>
        <div className="grid grid-cols-3 gap-3">
          {field('Roll', '°', roll, setRoll)}
          {field('Pitch', '°', pitch, setPitch)}
          {field('Yaw', '°', yaw, setYaw)}
        </div>
      </div>

      {problem && <p className="mb-2 text-xs text-red-700">{problem}</p>}

      <div className="flex gap-2 mb-3">
        <button
          onClick={apply}
          className="flex-1 bg-blue-600 text-white px-4 py-2 rounded font-semibold text-sm hover:bg-blue-700"
        >
          Apply
        </button>
        <button
          onClick={() => {
            resetToolFrame();
            seedFrom(DEFAULT_TOOL_FRAME);
            setProblem(null);
          }}
          disabled={isDefault}
          className="px-4 py-2 bg-gray-200 text-gray-700 rounded text-sm hover:bg-gray-300 disabled:opacity-40"
          title="Treat the flange face as the tool again"
        >
          Bare flange
        </button>
      </div>

      <div className="p-3 bg-gray-50 rounded text-xs space-y-1">
        <div>
          <span className="text-gray-600">In force:</span>{' '}
          {isDefault ? (
            <span className="font-mono">bare flange — no tool measured</span>
          ) : (
            <span className="font-mono">
              {reach.toFixed(1)} mm from the flange
            </span>
          )}
        </div>
        <div>
          <span className="text-gray-600">Tool tip now at:</span>{' '}
          <span className="font-mono">
            {currentPosition
              ? `${(currentPosition.x * 1000).toFixed(1)}, ${(currentPosition.y * 1000).toFixed(1)}, ${(currentPosition.z * 1000).toFixed(1)} mm`
              : '— not connected'}
          </span>
        </div>
      </div>

      {!isDefault && (
        <div className="mt-3">
          <p className="text-xs text-amber-700 mb-1">
            Remembered in this browser only. Paste this into{' '}
            <code>src/kinematics/robotModel.ts</code> to make it the model&apos;s own
            answer — otherwise another machine driving this arm has a different
            idea of where the tool is.
          </p>
          <button
            onClick={() => navigator.clipboard?.writeText(modelSource())}
            className="px-2 py-1 text-xs bg-gray-700 text-white rounded hover:bg-gray-800"
          >
            Copy for robotModel.ts
          </button>
          <pre className="mt-2 p-2 bg-gray-50 rounded text-[10px] overflow-x-auto">
            {modelSource()}
          </pre>
        </div>
      )}
    </div>
  );
};
