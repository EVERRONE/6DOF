import React, { useEffect } from 'react';
import { ConnectionPanel } from './components/ConnectionPanel';
import { JointControlPanel } from './components/JointControlPanel';
import { CartesianControlPanel } from './components/CartesianControlPanel';
import { PathPlannerPanel } from './components/PathPlannerPanel';
import { CalibrationPanel } from './components/CalibrationPanel';
import { AgentControlPanel } from './components/AgentControlPanel';
import { RobotViewer3D } from './components/RobotViewer3D';
import { StatusBar } from './components/StatusBar';
import { EmergencyStop } from './components/EmergencyStop';
import { SerialManager } from './communication/SerialManager';

function App() {
  useEffect(() => {
    if (!SerialManager.isSupported()) {
      console.warn('Web Serial API not supported. Please use Chrome or Edge browser.');
    }
  }, []);

  return (
    <div className="h-screen flex flex-col bg-gray-50">
      <ConnectionPanel />

      <div className="flex-1 flex overflow-hidden">
        {/* Left panel - Control panels */}
        <div className="w-1/3 overflow-y-auto border-r bg-white">
          <PathPlannerPanel />
          <CartesianControlPanel />
          <CalibrationPanel />
          <JointControlPanel />
          <AgentControlPanel />
        </div>

        {/* Right panel - 3D Viewer */}
        <div className="flex-1 bg-gray-100">
          <RobotViewer3D />
        </div>
      </div>

      <StatusBar />
      <EmergencyStop />
    </div>
  );
}

export default App;
