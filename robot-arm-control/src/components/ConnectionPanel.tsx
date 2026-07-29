import React from 'react';
import { useRobotStore } from '../store/robotStore';
import { ConnectionStatus } from '../types/robot';
import { SerialManager } from '../communication/SerialManager';

const STATUS_COLOR: Record<ConnectionStatus, string> = {
  [ConnectionStatus.DISCONNECTED]: 'bg-gray-500',
  [ConnectionStatus.CONNECTING]: 'bg-yellow-500',
  [ConnectionStatus.CONNECTED]: 'bg-green-500',
  [ConnectionStatus.RECONNECTING]: 'bg-orange-500',
  [ConnectionStatus.ERROR]: 'bg-red-500'
};

const STATUS_LABEL: Record<ConnectionStatus, string> = {
  [ConnectionStatus.DISCONNECTED]: 'Disconnected',
  [ConnectionStatus.CONNECTING]: 'Connecting',
  [ConnectionStatus.CONNECTED]: 'Connected',
  [ConnectionStatus.RECONNECTING]: 'Reconnecting',
  [ConnectionStatus.ERROR]: 'Connection error'
};

export const ConnectionPanel: React.FC = () => {
  const { connectionStatus, connectionDetail, connect, disconnect } = useRobotStore();

  const handleConnect = async () => {
    if (!SerialManager.isSupported()) {
      alert('Web Serial is not available in this browser. Use Chrome or Edge.');
      return;
    }

    try {
      await connect();
    } catch (error) {
      // The panel already shows the state and the reason, so a dialog on top of it
      // adds nothing - and the user dismissing the port picker is not a fault.
      console.warn('Connection failed:', error);
    }
  };

  const canConnect =
    connectionStatus === ConnectionStatus.DISCONNECTED ||
    connectionStatus === ConnectionStatus.ERROR;

  // Reconnecting counts: the user has to be able to call off a retry loop.
  const canDisconnect =
    connectionStatus === ConnectionStatus.CONNECTED ||
    connectionStatus === ConnectionStatus.RECONNECTING;

  const busy =
    connectionStatus === ConnectionStatus.CONNECTING ||
    connectionStatus === ConnectionStatus.RECONNECTING;

  return (
    <div className="p-4 bg-white border-b">
      <div className="flex items-center gap-4">
        <div className="flex items-center gap-2">
          <div
            className={`w-3 h-3 rounded-full ${STATUS_COLOR[connectionStatus]} ${
              busy ? 'animate-pulse' : ''
            }`}
          />
          <span className="font-semibold">{STATUS_LABEL[connectionStatus]}</span>
        </div>

        {canConnect && (
          <button
            onClick={handleConnect}
            className="px-4 py-2 bg-blue-600 text-white rounded hover:bg-blue-700"
          >
            Connect to Robot
          </button>
        )}

        {canDisconnect && (
          <button
            onClick={disconnect}
            className="px-4 py-2 bg-gray-600 text-white rounded hover:bg-gray-700"
          >
            {connectionStatus === ConnectionStatus.RECONNECTING
              ? 'Stop reconnecting'
              : 'Disconnect'}
          </button>
        )}

        {/* Why the link is down, and what is being done about it. */}
        {connectionDetail && connectionStatus !== ConnectionStatus.CONNECTED && (
          <span
            className={`text-sm ${
              connectionStatus === ConnectionStatus.ERROR
                ? 'text-red-700'
                : 'text-gray-600'
            }`}
          >
            {connectionDetail}
          </span>
        )}
      </div>

      {connectionStatus === ConnectionStatus.RECONNECTING && (
        <div className="mt-2 p-2 rounded text-xs bg-orange-50 text-orange-800">
          The link dropped, so the arm is no longer under control and any running
          path has been abandoned. The controller restarts when the cable is
          replugged, which means the arm needs re-homing before its reported
          position means anything again.
        </div>
      )}
    </div>
  );
};
