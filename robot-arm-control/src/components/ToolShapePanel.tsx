import React, { useState } from 'react';
import { useRobotStore } from '../store/robotStore';

/**
 * How much space the tool takes up.
 *
 * Not the same thing as the tool frame above it, and the difference is the whole
 * point. The **frame** says where the tip is: it moves what the arm reports. This
 * says how big the tool is: it moves what the arm refuses. A tool can be
 * measured, fitted and driven around with a perfect tool frame while the
 * collision checker still believes the arm ends at the flange face - which is
 * exactly the state this arm was in until now.
 *
 * One box, because the tool is whatever was bolted on this morning and there is
 * no model of it to load. A bounding box over-reports rather than under-reports,
 * which is the right direction for a guard.
 */
export const ToolShapePanel: React.FC = () => {
  const { toolShape, setToolShape, clearToolShape } = useRobotStore();

  // Millimetres in the UI, metres in the model.
  const mm = (v: number) => (v * 1000).toFixed(1);
  const [sx, setSx] = useState(toolShape ? mm(toolShape.size.x) : '');
  const [sy, setSy] = useState(toolShape ? mm(toolShape.size.y) : '');
  const [sz, setSz] = useState(toolShape ? mm(toolShape.size.z) : '');
  const [cx, setCx] = useState(toolShape ? mm(toolShape.centre.x) : '0');
  const [cy, setCy] = useState(toolShape ? mm(toolShape.centre.y) : '0');
  const [cz, setCz] = useState(toolShape ? mm(toolShape.centre.z) : '');
  const [problem, setProblem] = useState<string | null>(null);

  const sizes = [sx, sy, sz].map(Number);
  const sizesValid = sizes.every(v => Number.isFinite(v) && v > 0);

  const sitFlush = () => {
    if (!sizesValid) {
      setProblem('Fill in the three sizes first');
      return;
    }
    setProblem(null);
    setCx('0');
    setCy('0');
    setCz((sizes[2] / 2).toFixed(1));
  };

  const apply = () => {
    const centre = [cx, cy, cz].map(Number);
    if (!sizesValid) {
      setProblem('Every side has to be a positive length in millimetres');
      return;
    }
    if (!centre.every(Number.isFinite)) {
      setProblem('The centre has to be three numbers');
      return;
    }

    const refused = setToolShape({
      size: { x: sizes[0] / 1000, y: sizes[1] / 1000, z: sizes[2] / 1000 },
      centre: { x: centre[0] / 1000, y: centre[1] / 1000, z: centre[2] / 1000 }
    });
    setProblem(refused);
  };

  const clear = () => {
    clearToolShape();
    setProblem(null);
    setSx(''); setSy(''); setSz('');
    setCx('0'); setCy('0'); setCz('');
  };

  const field = (
    label: string,
    value: string,
    onChange: (v: string) => void
  ) => (
    <label className="flex-1 min-w-[4.5rem]">
      <span className="block text-xs text-gray-500 mb-0.5">{label}</span>
      <input
        type="number"
        value={value}
        onChange={e => onChange(e.target.value)}
        className="w-full px-2 py-1 border rounded text-sm font-mono text-right"
      />
    </label>
  );

  return (
    <div className="p-4 bg-white border-b">
      <h2 className="text-lg font-bold mb-1">Tool size</h2>
      <p className="text-xs text-gray-600 mb-3">
        What is bolted to the flange, as a box. The collision checker knows only
        about the arm's own links — until this is filled in it will happily clear
        a pose that drives the tool into the shoulder.
      </p>

      <div
        className={`mb-3 px-2 py-1.5 rounded text-xs ${
          toolShape
            ? 'bg-green-50 text-green-800'
            : 'bg-amber-50 text-amber-800'
        }`}
      >
        {toolShape ? (
          <>
            Checked against the shoulder, upper arm and elbow:{' '}
            <span className="font-mono">
              {mm(toolShape.size.x)} × {mm(toolShape.size.y)} × {mm(toolShape.size.z)} mm
            </span>
          </>
        ) : (
          'Nothing fitted — the checker believes the arm ends at the flange face.'
        )}
      </div>

      <div className="mb-2">
        <div className="text-xs text-gray-600 mb-1">Size, mm</div>
        <div className="flex gap-2">
          {field('X', sx, setSx)}
          {field('Y', sy, setSy)}
          {field('Z — along the tool', sz, setSz)}
        </div>
      </div>

      <div className="mb-2">
        <div className="flex items-baseline justify-between mb-1">
          <span className="text-xs text-gray-600">Centre of the box, mm from the flange face</span>
          <button
            onClick={sitFlush}
            className="text-xs text-blue-700 hover:underline"
          >
            sit it on the flange
          </button>
        </div>
        <div className="flex gap-2">
          {field('X', cx, setCx)}
          {field('Y', cy, setCy)}
          {field('Z', cz, setCz)}
        </div>
      </div>

      {/* An L-bracket hangs to one side, and a box centred on the bolt circle
          would claim space that is empty and miss space that is not. */}
      <p className="text-xs text-gray-500 mb-3">
        The flange face is zero and +Z points away from the arm, so a tool sitting
        flush has its centre half its length out. Move X and Y if the tool hangs
        to one side rather than sitting on the bolt circle.
      </p>

      {problem && <p className="mb-2 text-xs text-red-700">{problem}</p>}

      <div className="flex gap-2">
        <button
          onClick={apply}
          className="flex-1 px-3 py-2 rounded font-semibold text-white bg-blue-600 hover:bg-blue-700"
        >
          {toolShape ? 'Update' : 'Fit it'}
        </button>
        <button
          onClick={clear}
          disabled={!toolShape}
          className="px-3 py-2 border rounded text-sm text-gray-700 hover:bg-gray-50 disabled:opacity-40"
        >
          Bare flange
        </button>
      </div>

      <p className="text-xs text-gray-500 mt-3">
        Measure the box round everything that moves with the flange — brackets and
        cable loops included. It is a guard, so too big is the safe mistake.
        The forearm is not checked against it: the forearm's own geometry encloses
        the wrist mount, so it overlaps anything on the flange whether or not a
        tool is there.
      </p>
    </div>
  );
};
