import React, { useEffect, useState } from 'react';
import { useRobotStore } from '../store/robotStore';
import { ConnectionStatus } from '../types/robot';
import { JOINT_LIMITS_DEG, ROBOT_JOINTS } from '../kinematics/robotModel';

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
   * Run one axis far enough to give a full accelerate-cruise-decelerate cycle,
   * and no further.
   *
   * The sweep is derived from the limits being tested rather than being a fixed
   * fraction of the joint's travel. Two ramps cover v^2/a in total, so twice
   * that leaves about half the move at constant speed - enough to hear cruise as
   * its own phase, which is what separates a speed problem from an acceleration
   * problem.
   *
   * A fixed fraction was wrong in both directions. It under-ran the geared
   * joints, where a short move never reaches cruise at all and only the ramp is
   * being judged. And on J6 - continuous, so its limits are +/-360 - twenty to
   * eighty percent of travel is a 432 degree swing, more than a full turn, which
   * wraps whatever is cabled to the tool.
   *
   * Centred on the middle of the joint's range so the sweep is symmetric and
   * cannot walk into a limit.
   */
  const sweepFor = (axis: number, speed: number, accel: number) => {
    const lo = JOINT_LIMITS_DEG.min[axis];
    const hi = JOINT_LIMITS_DEG.max[axis];
    const travel = hi - lo;

    const available = travel * 0.8;
    // Two ramps cover v^2/a between them. Below that the move is triangular and
    // never reaches the speed being asked for at all.
    const toReachCruise = accel > 0 ? (speed * speed) / accel : travel;
    // Twice that leaves about half the move at constant speed, which is what
    // makes cruise audible as its own phase.
    const wanted = 2 * toReachCruise;

    // Never more than 80% of travel, so the ends stay clear, and never so short
    // that the move is over before anything can be heard.
    const span = Math.max(Math.min(wanted, available), Math.min(10, available));
    const mid = (lo + hi) / 2;

    return {
      from: mid - span / 2,
      to: mid + span / 2,
      span,
      // True only when the joint runs out of room before it can reach the
      // commanded speed - then "grinding in the middle" is not a verdict
      // available here, because there is no middle. A sweep that merely has
      // less cruise than we would like still tests the speed, so it is not
      // flagged: crying wolf on the geared joints, whose travel is short, would
      // make the warning worth ignoring.
      clipped: toReachCruise > available
    };
  };

  const listenTo = async (axis: number) => {
    if (busy) return;
    const limit = axisLimits[axis];
    if (!limit) return;

    setBusy(true);
    try {
      const sweep = sweepFor(axis, limit.speed, limit.accel);
      const key = (['J1', 'J2', 'J3', 'J4', 'J5', 'J6'] as const)[axis];

      logEvent(
        'info',
        `Listen to J${axis + 1}: ${sweep.span.toFixed(0)}° sweep, ` +
          `${sweep.from.toFixed(0)}° to ${sweep.to.toFixed(0)}°` +
          (sweep.clipped ? ' — too short to reach cruise at this speed' : '')
      );

      for (const target of [sweep.from, sweep.to, sweep.from]) {
        setTargetAngles({ [key]: target });
        // eslint-disable-next-line no-await-in-loop
        await moveToTarget();
        // eslint-disable-next-line no-await-in-loop
        await new Promise(r => setTimeout(r, 400));
      }

      logEvent(
        'info',
        `J${axis + 1}: knock at the ends is acceleration, grinding in the middle is speed`
      );
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
                disabled={blocked !== null || busy || !limit}
                title={
                  limit
                    ? `Sweep ${sweepFor(i, limit.speed, limit.accel).span.toFixed(0)}° ` +
                      `(${sweepFor(i, limit.speed, limit.accel).from.toFixed(0)}° to ` +
                      `${sweepFor(i, limit.speed, limit.accel).to.toFixed(0)}°), there and back — ` +
                      'long enough for a full ramp-cruise-ramp at these limits'
                    : 'Waiting for the axis limits'
                }
                className="px-2 py-0.5 border rounded hover:bg-gray-50 disabled:opacity-40"
              >
                Listen
                {limit && (
                  <span className="ml-1 text-gray-400">
                    {sweepFor(i, limit.speed, limit.accel).span.toFixed(0)}°
                  </span>
                )}
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
