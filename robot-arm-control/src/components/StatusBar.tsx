import React from 'react';
import { useRobotStore } from '../store/robotStore';
import { RobotState } from '../types/robot';

export const StatusBar: React.FC = () => {
  const { robotState, currentAngles, endstopState, currentPosition, firmwareStatus } =
    useRobotStore();

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

          {/* A hard stop can lose steps, so the reported angles stop meaning
              anything until the arm is re-homed. Say so plainly. */}
          {firmwareStatus && !firmwareStatus.positionTrusted && (
            <span
              className="px-3 py-1 rounded-full text-sm font-semibold bg-amber-100 text-amber-800"
              title="A hard stop may have lost steps, or the arm has not been homed yet. Re-home before relying on these angles."
            >
              POSITION UNVERIFIED
            </span>
          )}

          {/* Firmware motion queue. Depth is what keeps a streamed trajectory
              continuous, so it is worth being able to see it. */}
          {firmwareStatus && (
            <div className="text-sm text-gray-700" title="Free slots in the firmware motion queue">
              <span className="font-medium">Queue:</span>{' '}
              {firmwareStatus.queueFree} free
            </div>
          )}

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

        <div className="flex items-center gap-4">
          {/* Which joints have a datum. Without one the reported angle for that
              joint is only relative to wherever it happened to be at power-up. */}
          {firmwareStatus && (
            <div className="flex items-center gap-2">
              <span className="text-sm font-medium">Homed:</span>
              {firmwareStatus.homed.map((homed, index) => (
                <div
                  key={index}
                  className={`w-6 h-6 rounded flex items-center justify-center text-xs font-bold ${
                    homed ? 'bg-green-500 text-white' : 'bg-gray-300 text-gray-600'
                  }`}
                  title={`J${index + 1} ${homed ? 'homed' : 'not homed'}`}
                >
                  {index + 1}
                </div>
              ))}
            </div>
          )}

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
    </div>
  );
};
