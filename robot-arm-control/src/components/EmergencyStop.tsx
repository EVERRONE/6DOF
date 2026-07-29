import React, { useCallback, useEffect, useState } from 'react';
import { useRobotStore } from '../store/robotStore';
import { ConnectionStatus, RobotState } from '../types/robot';

/**
 * Software emergency stop.
 *
 * Bound to Escape as well as the button, because reaching for a mouse target is
 * the wrong motion when an arm is doing something you did not intend.
 *
 * It stays active while the link is reconnecting. The command will not get
 * through, but the button greying out at the exact moment you want it is worse
 * than a press that reports it could not be sent - and the panel says plainly that
 * the physical switch is the one that always works.
 */
export const EmergencyStop: React.FC = () => {
  const { emergencyStop, connectionStatus, robotState, logEvent } = useRobotStore();
  const [flash, setFlash] = useState(false);

  const reachable =
    connectionStatus === ConnectionStatus.CONNECTED ||
    connectionStatus === ConnectionStatus.RECONNECTING;

  const latched = robotState === RobotState.ESTOPPED;

  const trigger = useCallback(async () => {
    if (connectionStatus !== ConnectionStatus.CONNECTED) {
      logEvent('error', 'Emergency stop could not be sent: no link. Use the physical switch.');
      return;
    }

    setFlash(true);
    setTimeout(() => setFlash(false), 400);
    await emergencyStop();
  }, [connectionStatus, emergencyStop, logEvent]);

  useEffect(() => {
    const onKeyDown = (event: KeyboardEvent) => {
      if (event.key !== 'Escape') return;
      event.preventDefault();
      void trigger();
    };

    window.addEventListener('keydown', onKeyDown);
    return () => window.removeEventListener('keydown', onKeyDown);
  }, [trigger]);

  return (
    <div className="fixed bottom-6 right-6 flex flex-col items-center gap-1">
      <button
        onClick={() => void trigger()}
        disabled={!reachable}
        className={`w-24 h-24 rounded-full shadow-lg font-bold text-white flex flex-col items-center justify-center leading-tight
          ${flash ? 'ring-4 ring-red-300' : ''}
          ${
            latched
              ? 'bg-red-800'
              : reachable
              ? 'bg-red-600 hover:bg-red-700'
              : 'bg-gray-400'
          }`}
        title="Emergency stop (Escape). Cuts step pulses at once; the arm keeps holding torque."
      >
        <span className="text-lg">{latched ? 'STOPPED' : 'E-STOP'}</span>
        <span className="text-[10px] font-normal opacity-80">Esc</span>
      </button>

      {latched && (
        <span className="px-2 py-0.5 rounded bg-red-100 text-red-800 text-[11px] font-semibold">
          Re-enable motors to clear
        </span>
      )}

      {!reachable && (
        <span
          className="px-2 py-0.5 rounded bg-gray-100 text-gray-600 text-[11px] text-center max-w-[7rem]"
          title="Without a link the software cannot stop the arm"
        >
          No link — use the physical switch
        </span>
      )}
    </div>
  );
};
