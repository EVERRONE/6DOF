import React, { useEffect, useState } from 'react';
import { ConnectionPanel } from './components/ConnectionPanel';
import { JointControlPanel } from './components/JointControlPanel';
import { CartesianControlPanel } from './components/CartesianControlPanel';
import { PathPlannerPanel } from './components/PathPlannerPanel';
import { CommandConsole } from './components/CommandConsole';
import { TuningPanel } from './components/TuningPanel';
import { ToolFramePanel } from './components/ToolFramePanel';
import { WorkObjectPanel } from './components/WorkObjectPanel';
import { IOPanel } from './components/IOPanel';
import { RobotViewer3D } from './components/RobotViewer3D';
import { StatusBar } from './components/StatusBar';
import { EmergencyStop } from './components/EmergencyStop';
import { SerialManager } from './communication/SerialManager';

/**
 * Tabs rather than one long scrolling column.
 *
 * Jog is first and is the default, because that is what bench work needs: you want
 * the jog buttons and the console to hand, not to scroll past a 500-line path
 * planner to reach them.
 */
const TABS = [
  { id: 'jog', label: 'Jog', hint: 'Per-joint nudging and homing' },
  { id: 'tuning', label: 'Tuning', hint: 'Speed and acceleration, set by ear' },
  { id: 'cartesian', label: 'Cartesian', hint: 'XYZ target through IK' },
  { id: 'paths', label: 'Paths', hint: 'Teach waypoints and play them back' },
  { id: 'io', label: 'I/O', hint: 'Grippers, valves and sensors' }
] as const;

type TabId = (typeof TABS)[number]['id'];

function App() {
  const [tab, setTab] = useState<TabId>('jog');

  useEffect(() => {
    if (!SerialManager.isSupported()) {
      console.warn('Web Serial is not available. Use Chrome or Edge.');
    }
  }, []);

  return (
    <div className="h-screen flex flex-col bg-gray-50">
      <ConnectionPanel />

      <div className="flex-1 flex overflow-hidden">
        {/* Left: controls */}
        <div className="w-1/3 min-w-[24rem] flex flex-col border-r bg-white">
          <div className="flex border-b bg-gray-50">
            {TABS.map(({ id, label, hint }) => (
              <button
                key={id}
                onClick={() => setTab(id)}
                title={hint}
                className={`px-4 py-2 text-sm font-medium border-b-2 -mb-px ${
                  tab === id
                    ? 'border-blue-600 text-blue-700 bg-white'
                    : 'border-transparent text-gray-600 hover:text-gray-900'
                }`}
              >
                {label}
              </button>
            ))}
          </div>

          <div className="flex-1 overflow-y-auto">
            {tab === 'jog' && <JointControlPanel />}
            {tab === 'tuning' && <TuningPanel />}
            {tab === 'cartesian' && (
              <>
                <ToolFramePanel />
                <CartesianControlPanel />
              </>
            )}
            {tab === 'paths' && (
              <>
                <WorkObjectPanel />
                <PathPlannerPanel />
              </>
            )}
            {tab === 'io' && <IOPanel />}
          </div>

          {/* Always visible, whichever tab is open: what the arm said, and a way
              to talk to it directly. */}
          <CommandConsole />
        </div>

        {/* Right: 3D view */}
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
