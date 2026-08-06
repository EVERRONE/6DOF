import React, { useEffect, useState } from 'react';
import { useRobotStore } from '../store/robotStore';
import { ConnectionStatus } from '../types/robot';
import { HOME_POSE_DEG, JOINT_LIMITS_DEG, ROBOT_JOINTS } from '../kinematics/robotModel';

/**
 * Tuning speed and acceleration by ear.
 *
 * Acceleration is the parameter that decides whether the arm is quiet, and there
 * is no instrument for it here - the arm tells you, and it tells you in sound:
 *
 *   knock or thud at the start or stop of a move   acceleration too high
 *   grinding through the constant-speed part       speed past the driver's torque
 *   growl at one particular speed only             motor resonance, not a fault
 *
 * A stepper is audibly in trouble before it is measurably in trouble: by the time
 * position error can be measured, steps have already been lost. So the loop is
 * run, listen, adjust, run again - which is why these values are settable over
 * the link instead of only in config.h, where every attempt would cost a
 * re-flash.
 *
 * Nothing here is persisted. The firmware forgets on reboot, deliberately, so a
 * tuning value cannot be left in by accident with config.h no longer describing
 * the machine. Copy the numbers out when they are right.
 */
export const TuningPanel: React.FC = () => {
  const {
    axisLimits,
    refreshAxisLimits,
    setAxisLimit,
    connectionStatus,
    firmwareStatus,
    setTargetAngles,
    moveToTarget,
    logEvent
  } = useRobotStore();

  const connected = connectionStatus === ConnectionStatus.CONNECTED;
  const [busy, setBusy] = useState(false);

  useEffect(() => {
    if (connected) refreshAxisLimits();
  }, [connected, refreshAxisLimits]);

  /**
   * Run one axis across most of its travel, so there is a full
   * accelerate-cruise-decelerate cycle to listen to. A short move never reaches
   * cruise, and then only the ramp is being judged.
   */
  const listenTo = async (axis: number) => {
    if (busy) return;
    setBusy(true);
    try {
      const lo = JOINT_LIMITS_DEG.min[axis];
      const hi = JOINT_LIMITS_DEG.max[axis];
      const span = hi - lo;
      const near = lo + span * 0.2;
      const far = lo + span * 0.8;

      const key = (['J1', 'J2', 'J3', 'J4', 'J5', 'J6'] as const)[axis];
      const pose = { ...HOME_POSE_DEG };

      for (const target of [near, far, near]) {
        setTargetAngles({ [key]: target });
        // eslint-disable-next-line no-await-in-loop
        await moveToTarget();
        // eslint-disable-next-line no-await-in-loop
        await new Promise(r => setTimeout(r, 400));
      }
      void pose;
      logEvent('info', `Listen to J${axis + 1}: knock at the ends is acceleration, grinding in the middle is speed`);
    } finally {
      setBusy(false);
    }
  };

  const configLines = () => {
    const speeds = ROBOT_JOINTS.map((_, i) => (axisLimits[i]?.speed ?? 0).toFixed(1) + 'f');
    const accels = ROBOT_JOINTS.map((_, i) => (axisLimits[i]?.accel ?? 0).toFixed(1) + 'f');
    return (
      `const float MAX_JOINT_SPEED[NUM_AXES] = {${speeds.join(', ')}};\n` +
      `const float MAX_JOINT_ACCEL[NUM_AXES] = {${accels.join(', ')}};`
    );
  };

  const blocked = !connected
    ? 'Not connected'
    : firmwareStatus && !firmwareStatus.enabled
      ? 'Motors are off'
      : firmwareStatus?.moving
        ? 'Wait for the arm to stop — limits cannot change under a running move'
        : null;

  return (
    <div className="p-4 border-b">
      <h2 className="text-lg font-bold mb-1">Speed &amp; acceleration</h2>
      <p className="text-xs text-gray-600 mb-3">
        Set by ear. A <strong>knock at the start or stop</strong> of a move means the
        acceleration is too high for that joint. <strong>Grinding through the middle</strong>,
        at constant speed, means the speed is past where the driver holds torque. A{' '}
        <strong>growl at one speed only</strong> is motor resonance, not a fault.
      </p>
      <p className="text-xs text-amber-700 mb-3">
        These are forgotten on reboot. Copy them into <code>config.h</code> when they
        are right — and re-home after any run that ground, because lost steps are
        silent.
      </p>

      {blocked && <p className="mb-2 text-xs text-amber-700">{blocked}</p>}

      <div className="space-y-2">
        {ROBOT_JOINTS.map((joint, i) => {
          const limit = axisLimits[i];
          return (
            <div key={joint.label} className="flex items-center gap-2 text-xs">
              <span className="w-6 font-semibold">{joint.label}</span>

              <label className="flex items-center gap-1">
                <span className="text-gray-500">speed</span>
                <input
                  type="range"
                  min={1}
                  max={limit?.maxSpeed ?? 60}
                  step={1}
                  value={limit?.speed ?? 0}
                  disabled={blocked !== null || !limit}
                  onChange={e => setAxisLimit(i, Number(e.target.value), limit?.accel ?? 0)}
                  className="w-28"
                />
                <span className="w-14 font-mono text-right">
                  {limit ? `${limit.speed.toFixed(0)}°/s` : '—'}
                </span>
              </label>

              <label className="flex items-center gap-1">
                <span className="text-gray-500">accel</span>
                <input
                  type="range"
                  min={5}
                  max={limit?.maxAccel ?? 400}
                  step={5}
                  value={limit?.accel ?? 0}
                  disabled={blocked !== null || !limit}
                  onChange={e => setAxisLimit(i, limit?.speed ?? 0, Number(e.target.value))}
                  className="w-28"
                />
                <span className="w-16 font-mono text-right">
                  {limit ? `${limit.accel.toFixed(0)}°/s²` : '—'}
                </span>
              </label>

              <button
                onClick={() => listenTo(i)}
                disabled={blocked !== null || busy}
                title="Run this joint across most of its travel, so there is a full ramp-cruise-ramp to listen to"
                className="px-2 py-0.5 border rounded hover:bg-gray-50 disabled:opacity-40"
              >
                Listen
              </button>
            </div>
          );
        })}
      </div>

      <div className="mt-3 flex items-center gap-2">
        <button
          onClick={() => navigator.clipboard?.writeText(configLines())}
          disabled={axisLimits.length === 0}
          className="px-2 py-1 text-xs bg-gray-700 text-white rounded hover:bg-gray-800 disabled:opacity-40"
        >
          Copy config.h lines
        </button>
        <button
          onClick={() => useRobotStore.getState().sendRawCommand('V RESET')}
          disabled={!connected}
          className="px-2 py-1 text-xs border rounded hover:bg-gray-50 disabled:opacity-40"
        >
          Back to config values
        </button>
      </div>

      {axisLimits.length > 0 && (
        <pre className="mt-2 p-2 bg-gray-50 rounded text-[10px] overflow-x-auto">
          {configLines()}
        </pre>
      )}
    </div>
  );
};
