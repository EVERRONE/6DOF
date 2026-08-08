import React, { useState } from 'react';
import { useRobotStore } from '../store/robotStore';
import { ConnectionStatus } from '../types/robot';

/**
 * Nudging the tool in a straight line, a fixed distance at a time.
 *
 * Per-joint jogging is what the arm has, and it is the wrong tool for putting
 * the tip somewhere: nudge J2 and the tool swings through an arc, so touching a
 * particular corner takes a dozen corrections across three joints. Teaching a
 * work object needs three touched points, which makes this the thing standing
 * between that feature and being usable.
 *
 * Every jog holds the tool's attitude. A jog is a translation - "5 mm up" that
 * also tips the tool 3 degrees is a surprise, and it is exactly what an
 * unconstrained solve produces: 8 degrees for a 20 mm move in Z from the parked
 * pose.
 */

const STEPS_MM = [0.1, 0.5, 1, 5, 10, 25, 50];
const STEPS_DEG = [0.5, 1, 5, 10, 45];

export const CartesianJogPanel: React.FC = () => {
  const {
    jogFrame,
    setJogFrame,
    jogStepMm,
    setJogStepMm,
    jogStepDeg,
    setJogStepDeg,
    jogCartesian,
    jogRotation,
    connectionStatus,
    firmwareStatus,
    currentPosition,
    activeFrame
  } = useRobotStore();

  const [busy, setBusy] = useState(false);
  const [customMm, setCustomMm] = useState('');

  const connected = connectionStatus === ConnectionStatus.CONNECTED;

  const blocked = !connected
    ? 'Not connected'
    : firmwareStatus && !firmwareStatus.positionTrusted
      ? 'Home the arm first — its position is not trusted'
      : firmwareStatus && !firmwareStatus.enabled
        ? 'Motors are off'
        : !currentPosition
          ? 'No tool position reported yet'
          : null;

  const run = async (fn: () => Promise<void>) => {
    if (busy) return;
    setBusy(true);
    try {
      await fn();
    } finally {
      setBusy(false);
    }
  };

  const move = (axis: 'x' | 'y' | 'z', sign: 1 | -1) =>
    run(() => jogCartesian(axis, sign * jogStepMm));
  const turn = (axis: 'x' | 'y' | 'z', sign: 1 | -1) =>
    run(() => jogRotation(axis, sign * jogStepDeg));

  const pair = (
    axis: 'x' | 'y' | 'z',
    label: string,
    minus: string,
    plus: string,
    onJog: (axis: 'x' | 'y' | 'z', sign: 1 | -1) => void
  ) => (
    <div className="flex items-center gap-2">
      <span className="w-8 font-mono text-sm font-semibold">{label}</span>
      <button
        onClick={() => onJog(axis, -1)}
        disabled={blocked !== null || busy}
        className="flex-1 px-2 py-2 border rounded text-sm hover:bg-gray-50 disabled:opacity-40"
      >
        − {minus}
      </button>
      <button
        onClick={() => onJog(axis, 1)}
        disabled={blocked !== null || busy}
        className="flex-1 px-2 py-2 border rounded text-sm hover:bg-gray-50 disabled:opacity-40"
      >
        + {plus}
      </button>
    </div>
  );

  return (
    <div className="p-4 bg-white border-b">
      <h2 className="text-lg font-bold mb-1">Move the tool</h2>
      <p className="text-xs text-gray-600 mb-3">
        A straight line, one step at a time, with the tool held at the angle it is
        already at.
      </p>

      {/* Which axes. This is the choice that decides what the buttons mean. */}
      <div className="mb-3">
        <label className="block text-xs text-gray-600 mb-1">Along the axes of</label>
        <div className="flex gap-1">
          {([
            ['base', 'World', 'Z is up. The only one that means the same wherever you stand.'],
            ['tool', 'Tool', "The tool's own axes — Z is into whatever it points at."],
            ['work', 'Work object', "The active fixture's own edges."]
          ] as const).map(([id, label, hint]) => (
            <button
              key={id}
              onClick={() => setJogFrame(id)}
              title={hint}
              className={`flex-1 px-2 py-1 text-xs rounded border ${
                jogFrame === id
                  ? 'bg-blue-50 border-blue-400 text-blue-900 font-semibold'
                  : 'bg-white border-gray-200 text-gray-600 hover:bg-gray-50'
              }`}
            >
              {label}
            </button>
          ))}
        </div>
        {jogFrame === 'work' && (
          <p className="mt-1 text-xs text-gray-500">
            {activeFrame().name}
          </p>
        )}
      </div>

      {/* Step size */}
      <div className="mb-3">
        <label className="block text-xs text-gray-600 mb-1">Step</label>
        <div className="flex flex-wrap gap-1 items-center">
          {STEPS_MM.map(mm => (
            <button
              key={mm}
              onClick={() => { setJogStepMm(mm); setCustomMm(''); }}
              className={`px-2 py-1 text-xs rounded border font-mono ${
                jogStepMm === mm
                  ? 'bg-gray-700 text-white border-gray-700'
                  : 'bg-white border-gray-200 text-gray-700 hover:bg-gray-50'
              }`}
            >
              {mm}
            </button>
          ))}
          <input
            type="number"
            value={customMm}
            onChange={e => {
              setCustomMm(e.target.value);
              const v = Number(e.target.value);
              if (Number.isFinite(v) && v > 0) setJogStepMm(v);
            }}
            placeholder="mm"
            className="w-16 px-2 py-1 border rounded text-xs font-mono"
          />
          <span className="text-xs text-gray-500 font-mono">
            {jogStepMm} mm
          </span>
        </div>
      </div>

      {blocked && <p className="mb-2 text-xs text-amber-700">{blocked}</p>}

      <div className="space-y-2 mb-4">
        {/* Z first: it is the one direction whose name is not a matter of where
            you happen to be standing. */}
        {pair('z', 'Z', 'down', 'up', move)}
        {pair('x', 'X', 'X', 'X', move)}
        {pair('y', 'Y', 'Y', 'Y', move)}
      </div>

      <div className="mb-2">
        <label className="block text-xs text-gray-600 mb-1">
          Turn the tool — the tip stays put
        </label>
        <div className="flex flex-wrap gap-1 items-center mb-2">
          {STEPS_DEG.map(deg => (
            <button
              key={deg}
              onClick={() => setJogStepDeg(deg)}
              className={`px-2 py-1 text-xs rounded border font-mono ${
                jogStepDeg === deg
                  ? 'bg-gray-700 text-white border-gray-700'
                  : 'bg-white border-gray-200 text-gray-700 hover:bg-gray-50'
              }`}
            >
              {deg}°
            </button>
          ))}
        </div>
        <div className="space-y-2">
          {pair('x', 'RX', 'RX', 'RX', turn)}
          {pair('y', 'RY', 'RY', 'RY', turn)}
          {pair('z', 'RZ', 'RZ', 'RZ', turn)}
        </div>
      </div>

      <p className="text-xs text-gray-500">
        X and Y are not labelled left and right on purpose — which is which
        depends on where you are standing. Watch the axes marker on the tool in
        the 3D view; a jog moves along the same colours.
      </p>
    </div>
  );
};
