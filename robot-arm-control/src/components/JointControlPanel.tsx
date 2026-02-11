import React from 'react';
import { useRobotStore } from '../store/robotStore';

const JOINT_LIMITS = {
  J1: { min: -40, max: 30 },
  J2: { min: 0, max: 60 },
  J3: { min: 0, max: 70 },
  J4: { min: 0, max: 274 },
  J5: { min: 0, max: 280 },
  J6: { min: -360, max: 360 }
};

export const JointControlPanel: React.FC = () => {
  const {
    targetAngles,
    currentAngles,
    setTargetAngles,
    moveToTarget,
    motorsEnabled,
    manualSpeed,
    setManualSpeed,
    enableMotors,
    homeJoints
  } = useRobotStore();

  const handleSliderChange = (joint: keyof typeof JOINT_LIMITS, value: number) => {
    setTargetAngles({ [joint]: value });
  };

  return (
    <div className="p-4 bg-white">
      <h2 className="text-xl font-bold mb-4">Manual Joint Control</h2>

      {/* Motor enable toggle */}
      <div className="mb-4">
        <label className="flex items-center gap-2">
          <input
            type="checkbox"
            checked={motorsEnabled}
            onChange={(e) => enableMotors(e.target.checked)}
            className="w-4 h-4"
          />
          <span className="font-medium">Motors Enabled</span>
        </label>
      </div>

      {/* Speed control */}
      <div className="mb-6">
        <label className="block text-sm font-medium mb-2">
          Speed: {manualSpeed}°/s
        </label>
        <input
          type="range"
          min="5"
          max="100"
          value={manualSpeed}
          onChange={(e) => setManualSpeed(parseFloat(e.target.value))}
          className="w-full"
        />
      </div>

      {/* Joint sliders */}
      <div className="space-y-4">
        {Object.entries(JOINT_LIMITS).map(([joint, limits]) => (
          <div key={joint}>
            <div className="flex justify-between items-center mb-1">
              <label className="text-sm font-medium">{joint}</label>
              <div className="flex gap-4 text-sm">
                <span className="text-gray-600">
                  Current: {currentAngles[joint as keyof typeof currentAngles].toFixed(1)}°
                </span>
                <span className="text-blue-600 font-semibold">
                  Target: {targetAngles[joint as keyof typeof targetAngles].toFixed(1)}°
                </span>
              </div>
            </div>
            <input
              type="range"
              min={limits.min}
              max={limits.max}
              step="0.1"
              value={targetAngles[joint as keyof typeof targetAngles]}
              onChange={(e) => handleSliderChange(
                joint as keyof typeof JOINT_LIMITS,
                parseFloat(e.target.value)
              )}
              disabled={!motorsEnabled}
              className="w-full"
            />
            <div className="flex justify-between text-xs text-gray-500">
              <span>{limits.min}°</span>
              <span>{limits.max}°</span>
            </div>
          </div>
        ))}
      </div>

      {/* Move button */}
      <button
        onClick={moveToTarget}
        disabled={!motorsEnabled}
        className="w-full mt-6 px-4 py-3 bg-blue-600 text-white rounded hover:bg-blue-700 disabled:bg-gray-400 disabled:cursor-not-allowed font-semibold"
      >
        Move to Target
      </button>

      {/* Homing buttons */}
      <div className="mt-6">
        <h3 className="text-sm font-semibold mb-2">Homing</h3>
        <div className="grid grid-cols-3 gap-2">
          <button
            onClick={() => homeJoints('2')}
            disabled={!motorsEnabled}
            className="px-2 py-1 bg-gray-600 text-white text-sm rounded hover:bg-gray-700 disabled:bg-gray-400"
          >
            Home J2
          </button>
          <button
            onClick={() => homeJoints('3')}
            disabled={!motorsEnabled}
            className="px-2 py-1 bg-gray-600 text-white text-sm rounded hover:bg-gray-700 disabled:bg-gray-400"
          >
            Home J3
          </button>
          <button
            onClick={() => homeJoints('4')}
            disabled={!motorsEnabled}
            className="px-2 py-1 bg-gray-600 text-white text-sm rounded hover:bg-gray-700 disabled:bg-gray-400"
          >
            Home J4
          </button>
          <button
            onClick={() => homeJoints('5')}
            disabled={!motorsEnabled}
            className="px-2 py-1 bg-gray-600 text-white text-sm rounded hover:bg-gray-700 disabled:bg-gray-400"
          >
            Home J5
          </button>
          <button
            onClick={() => homeJoints('ALL')}
            disabled={!motorsEnabled}
            className="px-2 py-1 bg-purple-600 text-white text-sm rounded hover:bg-purple-700 disabled:bg-gray-400 col-span-2"
          >
            Home All
          </button>
        </div>
      </div>
    </div>
  );
};
