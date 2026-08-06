import React from 'react';
import { useRobotStore } from '../store/robotStore';
import { ConnectionStatus, RobotState } from '../types/robot';

const DEFAULT_JOINT_LIMITS = {
  J1: { min: -40, max: 30 },
  J2: { min: -60, max: 0 },
  J3: { min: 0, max: 70 },
  J4: { min: -274, max: 0 },
  J5: { min: -280, max: 0 },
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
    homeJoints,
    connectionStatus,
    robotState,
    firmwareConfig
  } = useRobotStore();

  const isConnected = connectionStatus === ConnectionStatus.CONNECTED;
  const isHoming = robotState === RobotState.HOMING;
  const isMoving = robotState === RobotState.MOVING;

  const clampSpeed = (value: number) => Math.min(100, Math.max(5, value));
  const safeManualSpeed = Number.isFinite(manualSpeed) ? clampSpeed(manualSpeed) : 30;

  const JOINT_LIMITS = firmwareConfig
    ? {
        J1: { min: firmwareConfig.joints[0]?.min ?? DEFAULT_JOINT_LIMITS.J1.min, max: firmwareConfig.joints[0]?.max ?? DEFAULT_JOINT_LIMITS.J1.max },
        J2: { min: firmwareConfig.joints[1]?.min ?? DEFAULT_JOINT_LIMITS.J2.min, max: firmwareConfig.joints[1]?.max ?? DEFAULT_JOINT_LIMITS.J2.max },
        J3: { min: firmwareConfig.joints[2]?.min ?? DEFAULT_JOINT_LIMITS.J3.min, max: firmwareConfig.joints[2]?.max ?? DEFAULT_JOINT_LIMITS.J3.max },
        J4: { min: firmwareConfig.joints[3]?.min ?? DEFAULT_JOINT_LIMITS.J4.min, max: firmwareConfig.joints[3]?.max ?? DEFAULT_JOINT_LIMITS.J4.max },
        J5: { min: firmwareConfig.joints[4]?.min ?? DEFAULT_JOINT_LIMITS.J5.min, max: firmwareConfig.joints[4]?.max ?? DEFAULT_JOINT_LIMITS.J5.max },
        J6: { min: firmwareConfig.joints[5]?.min ?? DEFAULT_JOINT_LIMITS.J6.min, max: firmwareConfig.joints[5]?.max ?? DEFAULT_JOINT_LIMITS.J6.max }
      }
    : DEFAULT_JOINT_LIMITS;

  const clampJointValue = (joint: keyof typeof JOINT_LIMITS, value: number) => {
    const limits = JOINT_LIMITS[joint];
    if (!Number.isFinite(value)) return limits.min;
    return Math.min(limits.max, Math.max(limits.min, value));
  };

  const handleSliderChange = (joint: keyof typeof JOINT_LIMITS, value: number) => {
    setTargetAngles({ [joint]: clampJointValue(joint, value) });
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
            disabled={!isConnected}
          />
          <span className="font-medium">Motors Enabled</span>
        </label>
        {!isConnected && (
          <div className="text-xs text-gray-500 mt-1">
            Connect to the robot to enable motors.
          </div>
        )}
      </div>

      {/* Speed control */}
      <div className="mb-6">
        <label className="block text-sm font-medium mb-2">
          Speed: {safeManualSpeed}deg/s
        </label>
        <input
          type="range"
          min="5"
          max="100"
          value={safeManualSpeed}
          onChange={(e) => setManualSpeed(clampSpeed(parseFloat(e.target.value)))}
          className="w-full"
        />
      </div>

      {/* Joint sliders */}
      <div className="space-y-4">
        {Object.entries(JOINT_LIMITS).map(([joint, limits]) => (
          <div key={joint}>
            {(() => {
              const key = joint as keyof typeof JOINT_LIMITS;
              const clampedRawTarget = clampJointValue(
                key,
                targetAngles[joint as keyof typeof targetAngles]
              );
              const logicalTarget = clampedRawTarget;
              const logicalCurrent = currentAngles[joint as keyof typeof currentAngles];

              return (
                <>
                  <div className="flex justify-between items-center mb-1">
                    <label className="text-sm font-medium">{joint}</label>
                    <div className="flex gap-4 text-sm">
                      <span className="text-gray-600">
                        Current: {logicalCurrent.toFixed(1)}deg
                      </span>
                      <span className="text-blue-600 font-semibold">
                        Target: {logicalTarget.toFixed(1)}deg
                      </span>
                    </div>
                  </div>
                  <input
                    type="range"
                    min={limits.min}
                    max={limits.max}
                    step="0.1"
                    value={logicalTarget}
                    onChange={(e) => handleSliderChange(
                      joint as keyof typeof JOINT_LIMITS,
                      parseFloat(e.target.value)
                    )}
                    className="w-full"
                  />
                  <div className="flex justify-between text-xs text-gray-500">
                    <span>{limits.min.toFixed(1)}deg</span>
                    <span>{limits.max.toFixed(1)}deg</span>
                  </div>
                </>
              );
            })()}
          </div>
        ))}
      </div>

      {/* Move button */}
      <button
        onClick={moveToTarget}
        disabled={!motorsEnabled || !isConnected || isHoming || isMoving}
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
            disabled={!motorsEnabled || !isConnected}
            className="px-2 py-1 bg-gray-600 text-white text-sm rounded hover:bg-gray-700 disabled:bg-gray-400"
          >
            Home J2
          </button>
          <button
            onClick={() => homeJoints('3')}
            disabled={!motorsEnabled || !isConnected}
            className="px-2 py-1 bg-gray-600 text-white text-sm rounded hover:bg-gray-700 disabled:bg-gray-400"
          >
            Home J3
          </button>
          <button
            onClick={() => homeJoints('4')}
            disabled={!motorsEnabled || !isConnected}
            className="px-2 py-1 bg-gray-600 text-white text-sm rounded hover:bg-gray-700 disabled:bg-gray-400"
          >
            Home J4
          </button>
          <button
            onClick={() => homeJoints('5')}
            disabled={!motorsEnabled || !isConnected}
            className="px-2 py-1 bg-gray-600 text-white text-sm rounded hover:bg-gray-700 disabled:bg-gray-400"
          >
            Home J5
          </button>
          <button
            onClick={() => homeJoints('ALL')}
            disabled={!motorsEnabled || !isConnected}
            className="px-2 py-1 bg-purple-600 text-white text-sm rounded hover:bg-purple-700 disabled:bg-gray-400 col-span-2"
          >
            Home All
          </button>
        </div>
      </div>
    </div>
  );
};
