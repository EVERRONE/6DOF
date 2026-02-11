import React from 'react';
import { useRobotStore } from '../store/robotStore';
import { RobotState } from '../types/robot';

export const StatusBar: React.FC = () => {
  const { robotState, currentAngles, endstopState, currentPosition } = useRobotStore();

  const stateColor = {
    [RobotState.IDLE]: 'bg-green-100 text-green-800',
    [RobotState.MOVING]: 'bg-blue-100 text-blue-800',
    [RobotState.HOMING]: 'bg-yellow-100 text-yellow-800',
    [RobotState.ERROR]: 'bg-red-100 text-red-800',
    [RobotState.ESTOPPED]: 'bg-red-100 text-red-800'
  };

  return (
    <div className="p-3 bg-gray-100 border-t">
      <div className="flex items-center justify-between">
        {/* Robot state */}
        <div className="flex items-center gap-4">
          <div className="flex items-center gap-2">
            <span className="text-sm font-medium">State:</span>
            <span className={`px-3 py-1 rounded-full text-sm font-semibold ${stateColor[robotState]}`}>
              {robotState.toUpperCase()}
            </span>
          </div>

          {/* Current XYZ position */}
          {currentPosition && (
            <div className="text-sm text-gray-700">
              <span className="font-medium">Position:</span>{' '}
              X:{(currentPosition.x * 1000).toFixed(1)}mm{' '}
              Y:{(currentPosition.y * 1000).toFixed(1)}mm{' '}
              Z:{(currentPosition.z * 1000).toFixed(1)}mm
            </div>
          )}

          {/* Current joint angles */}
          <div className="text-sm text-gray-700">
            <span className="font-medium">Angles:</span> J1:{currentAngles.J1.toFixed(1)}°
            J2:{currentAngles.J2.toFixed(1)}° J3:{currentAngles.J3.toFixed(1)}°
            J4:{currentAngles.J4.toFixed(1)}° J5:{currentAngles.J5.toFixed(1)}°
            J6:{currentAngles.J6.toFixed(1)}°
          </div>
        </div>

        {/* Endstop indicators */}
        <div className="flex items-center gap-2">
          <span className="text-sm font-medium">Endstops:</span>
          {Object.entries(endstopState).map(([joint, triggered]) => (
            <div
              key={joint}
              className={`w-6 h-6 rounded flex items-center justify-center text-xs font-bold ${
                triggered ? 'bg-red-500 text-white' : 'bg-gray-300 text-gray-600'
              }`}
              title={`${joint} ${triggered ? 'triggered' : 'free'}`}
            >
              {joint.charAt(1)}
            </div>
          ))}
        </div>
      </div>
    </div>
  );
};
