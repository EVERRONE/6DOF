import React from 'react';
import { useRobotStore } from '../store/robotStore';
import { ConnectionStatus } from '../types/robot';
import { SerialManager } from '../communication/SerialManager';

export const ConnectionPanel: React.FC = () => {
  const { connectionStatus, connect, disconnect } = useRobotStore();

  const handleConnect = async () => {
    if (!SerialManager.isSupported()) {
      alert('Web Serial API not supported. Use Chrome or Edge browser.');
      return;
    }

    try {
      await connect();
    } catch (error) {
      console.error('Connection failed:', error);
      alert('Failed to connect. Make sure the robot is plugged in.');
    }
  };

  const statusColor = {
    [ConnectionStatus.DISCONNECTED]: 'bg-gray-500',
    [ConnectionStatus.CONNECTING]: 'bg-yellow-500',
    [ConnectionStatus.CONNECTED]: 'bg-green-500',
    [ConnectionStatus.ERROR]: 'bg-red-500'
  };

  return (
    <div className="p-4 bg-white border-b">
      <div className="flex items-center gap-4">
        <div className="flex items-center gap-2">
          <div className={`w-3 h-3 rounded-full ${statusColor[connectionStatus]}`} />
          <span className="font-semibold">
            {connectionStatus.charAt(0).toUpperCase() + connectionStatus.slice(1)}
          </span>
        </div>

        {connectionStatus === ConnectionStatus.DISCONNECTED && (
          <button
            onClick={handleConnect}
            className="px-4 py-2 bg-blue-600 text-white rounded hover:bg-blue-700"
          >
            Connect to Robot
          </button>
        )}

        {connectionStatus === ConnectionStatus.CONNECTED && (
          <button
            onClick={disconnect}
            className="px-4 py-2 bg-gray-600 text-white rounded hover:bg-gray-700"
          >
            Disconnect
          </button>
        )}
      </div>
    </div>
  );
};
