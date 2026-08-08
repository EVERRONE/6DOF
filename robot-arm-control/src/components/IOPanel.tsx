import React, { useEffect } from 'react';
import { useRobotStore } from '../store/robotStore';
import { ConnectionStatus } from '../types/robot';

/**
 * Digital outputs and inputs.
 *
 * Everything the arm does besides move: a gripper, a valve, a light, a signal to
 * another machine, a sensor saying a part has arrived. Without them the arm can
 * go to a place and nothing else, which is the difference between a moving arm
 * and a robot.
 *
 * The names come from the firmware rather than from here. The pin a gripper is
 * on is a fact about the machine, and a panel carrying its own copy would be
 * wrong the first time somebody moved a wire and right about nothing afterwards.
 */
export const IOPanel: React.FC = () => {
  const {
    ioState,
    ioNames,
    setOutput,
    refreshIO,
    connectionStatus,
    firmwareStatus,
    sendRawCommand
  } = useRobotStore();

  const connected = connectionStatus === ConnectionStatus.CONNECTED;

  useEffect(() => {
    if (connected && ioNames.outputs.length === 0) void refreshIO();
  }, [connected, ioNames.outputs.length, refreshIO]);

  if (!connected) {
    return (
      <div className="p-4 bg-white border-b">
        <h2 className="text-lg font-bold mb-1">I/O</h2>
        <p className="text-xs text-gray-500">Not connected.</p>
      </div>
    );
  }

  const outputs = ioState?.outputs ?? [];
  const inputs = ioState?.inputs ?? [];

  return (
    <div className="p-4 bg-white border-b">
      <h2 className="text-lg font-bold mb-1">I/O</h2>
      <p className="text-xs text-gray-600 mb-3">
        Names and pins come from <code>firmware/config.h</code>. Outputs go to
        their safe state at start-up and on <code>E 0</code> — but{' '}
        <strong>not</strong> on an emergency stop, because a gripper that opens
        mid-stop drops what it is holding.
      </p>

      <div className="mb-3">
        <h3 className="text-sm font-semibold mb-2 text-gray-700">Outputs</h3>
        {outputs.length === 0 ? (
          <p className="text-xs text-gray-400">None reported yet.</p>
        ) : (
          <div className="space-y-1">
            {outputs.map((high, i) => (
              <div key={i} className="flex items-center gap-2 text-sm">
                <span
                  className={`w-3 h-3 rounded-full shrink-0 ${
                    high ? 'bg-green-500' : 'bg-gray-300'
                  }`}
                />
                <span className="flex-1 font-mono text-xs">
                  {ioNames.outputs[i] ?? `out${i + 1}`}
                </span>
                <button
                  onClick={() => void setOutput(i, !high)}
                  disabled={firmwareStatus ? !firmwareStatus.enabled : false}
                  title={
                    firmwareStatus && !firmwareStatus.enabled
                      ? 'Motors are off — the firmware has put the outputs down'
                      : high ? 'Switch off' : 'Switch on'
                  }
                  className={`px-3 py-1 rounded text-xs font-semibold disabled:opacity-40 ${
                    high
                      ? 'bg-green-600 text-white hover:bg-green-700'
                      : 'bg-gray-200 text-gray-700 hover:bg-gray-300'
                  }`}
                >
                  {high ? 'ON' : 'OFF'}
                </button>
              </div>
            ))}
          </div>
        )}
      </div>

      <div className="mb-3">
        <h3 className="text-sm font-semibold mb-2 text-gray-700">Inputs</h3>
        {inputs.length === 0 ? (
          <p className="text-xs text-gray-400">None reported yet.</p>
        ) : (
          <div className="space-y-1">
            {inputs.map((on, i) => (
              <div key={i} className="flex items-center gap-2 text-sm">
                <span
                  className={`w-3 h-3 rounded-full shrink-0 ${
                    on ? 'bg-blue-500' : 'bg-gray-300'
                  }`}
                />
                <span className="flex-1 font-mono text-xs">
                  {ioNames.inputs[i] ?? `in${i + 1}`}
                </span>
                <span className="text-xs text-gray-500 w-16 text-right">
                  {on ? 'closed' : 'open'}
                </span>
              </div>
            ))}
          </div>
        )}
      </div>

      <button
        onClick={() => void sendRawCommand('O SAFE')}
        className="px-3 py-1 border rounded text-xs text-gray-700 hover:bg-gray-50"
        title="Drive every output to the safe state in config.h"
      >
        All outputs to safe state
      </button>

      <p className="mt-3 text-xs text-amber-700">
        The pin numbers in <code>config.h</code> have not been checked against
        this arm&apos;s wiring. They are the ones the firmware provably does not
        use for anything else — that is an argument, not a measurement. Check with
        a meter before connecting anything.
      </p>
    </div>
  );
};
