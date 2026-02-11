import React from 'react';
import { useRobotStore } from '../store/robotStore';
import { ConnectionStatus } from '../types/robot';

export const EmergencyStop: React.FC = () => {
  const { emergencyStop, connectionStatus } = useRobotStore();

  const handleEStop = async () => {
    if (connectionStatus === ConnectionStatus.CONNECTED) {
      await emergencyStop();
    }
  };

  return (
    <div className="fixed bottom-6 right-6">
      <button
        onClick={handleEStop}
        disabled={connectionStatus !== ConnectionStatus.CONNECTED}
        className="w-24 h-24 bg-red-600 hover:bg-red-700 disabled:bg-gray-400 text-white rounded-full shadow-lg font-bold text-lg flex items-center justify-center"
        title="Emergency Stop"
      >
        E-STOP
      </button>
    </div>
  );
};
