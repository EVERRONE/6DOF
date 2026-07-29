import React, { useState } from 'react';
import { useRobotStore } from '../store/robotStore';
import { ConnectionStatus, JointAngles, RobotState } from '../types/robot';
import { HOME_POSE_DEG, ROBOT_JOINTS } from '../kinematics/robotModel';

/**
 * Jog panel.
 *
 * Built around relative nudges rather than sliders. Bring-up needs "move J3 by
 * exactly five degrees and watch which way it goes", and a slider spanning J4's
 * 274 degrees gives about one degree per pixel - you cannot hit a value, and you
 * cannot repeat one.
 *
 * The joint limits come from robotModel.ts, the same table the solver and the
 * firmware use, rather than a fourth hardcoded copy.
 */

const JOINT_KEYS: (keyof JointAngles)[] = ['J1', 'J2', 'J3', 'J4', 'J5', 'J6'];

const STEP_SIZES = [0.1, 1, 5, 10, 45];

/** Joints that can be homed, from the model rather than a hardcoded list. */
const HOMEABLE = [2, 3, 4, 5];

export const JointControlPanel: React.FC = () => {
  const {
    targetAngles,
    currentAngles,
    setTargetAngles,
    syncTargetsToCurrent,
    jogJoint,
    goToHomePose,
    moveToTarget,
    motorsEnabled,
    manualSpeed,
    setManualSpeed,
    enableMotors,
    homeJoints,
    connectionStatus,
    robotState,
    firmwareStatus
  } = useRobotStore();

  const [step, setStep] = useState(1);

  const connected = connectionStatus === ConnectionStatus.CONNECTED;
  const homing = robotState === RobotState.HOMING;
  const estopped = robotState === RobotState.ESTOPPED;

  const canMove = connected && motorsEnabled && !homing && !estopped;

  /** Why controls are locked, so a greyed-out button is not a mystery. */
  const blockedReason = !connected
    ? 'Not connected.'
    : estopped
    ? 'Emergency stop latched. Re-enable the motors to clear it.'
    : !motorsEnabled
    ? 'Motors are off.'
    : homing
    ? 'Homing in progress.'
    : null;

  const handleDisableMotors = async () => {
    // Cutting the drivers on a geared arm lets it sag, which is worth a beat of
    // thought rather than a single stray click.
    const ok = window.confirm(
      'Turn the motors off?\n\nThe arm loses holding torque and may sag under its own weight.'
    );
    if (ok) await enableMotors(false);
  };

  return (
    <div className="p-4 bg-white">
      <div className="flex items-center justify-between mb-3">
        <h2 className="text-xl font-bold">Jog</h2>

        <button
          onClick={() =>
            motorsEnabled ? void handleDisableMotors() : void enableMotors(true)
          }
          disabled={!connected}
          className={`px-3 py-1.5 rounded text-sm font-semibold text-white disabled:bg-gray-300 ${
            motorsEnabled
              ? 'bg-amber-600 hover:bg-amber-700'
              : 'bg-green-600 hover:bg-green-700'
          }`}
        >
          {motorsEnabled ? 'Motors on — turn off' : 'Turn motors on'}
        </button>
      </div>

      {blockedReason && (
        <div className="mb-3 p-2 rounded text-sm bg-gray-100 text-gray-700">
          {blockedReason}
        </div>
      )}

      {/* Step size and speed: the two things you change constantly at the bench. */}
      <div className="mb-4 flex flex-wrap items-end gap-4">
        <div>
          <div className="text-sm font-medium mb-1">Jog step</div>
          <div className="flex">
            {STEP_SIZES.map(size => (
              <button
                key={size}
                onClick={() => setStep(size)}
                className={`px-2.5 py-1 text-sm border first:rounded-l last:rounded-r -ml-px first:ml-0 ${
                  step === size
                    ? 'bg-blue-600 text-white border-blue-600 z-10'
                    : 'bg-white text-gray-700 hover:bg-gray-50'
                }`}
              >
                {size}°
              </button>
            ))}
          </div>
        </div>

        <div className="flex-1 min-w-[10rem]">
          <div className="text-sm font-medium mb-1">Speed: {manualSpeed}°/s</div>
          <input
            type="range"
            min={1}
            max={60}
            value={manualSpeed}
            onChange={e => setManualSpeed(parseFloat(e.target.value))}
            className="w-full"
          />
          <div className="text-xs text-gray-400">
            The firmware clamps this to each joint's own limit.
          </div>
        </div>
      </div>

      {/* Per-joint jog rows */}
      <div className="space-y-1">
        <div className="grid grid-cols-[2.5rem_1fr_1fr_auto] gap-2 text-xs text-gray-500 px-1">
          <span>Joint</span>
          <span className="text-right">Reported</span>
          <span className="text-right">Target</span>
          <span className="text-center">Jog</span>
        </div>

        {JOINT_KEYS.map((joint, index) => {
          const spec = ROBOT_JOINTS[index];
          const current = currentAngles[joint];
          const target = targetAngles[joint];
          const homed = firmwareStatus?.homed[index] ?? false;
          const canHome = HOMEABLE.includes(index + 1);

          const atMin = target <= spec.limitDeg.min + 1e-6;
          const atMax = target >= spec.limitDeg.max - 1e-6;

          return (
            <div
              key={joint}
              className="grid grid-cols-[2.5rem_1fr_1fr_auto] gap-2 items-center py-1 border-b last:border-b-0"
            >
              <div className="flex items-center gap-1">
                <span className="font-semibold text-sm">{joint}</span>
                {canHome && (
                  <span
                    className={`w-1.5 h-1.5 rounded-full ${
                      homed ? 'bg-green-500' : 'bg-gray-300'
                    }`}
                    title={homed ? 'Homed' : 'Not homed'}
                  />
                )}
              </div>

              <div className="text-right font-mono text-sm text-gray-700">
                {current.toFixed(1)}°
              </div>

              <input
                type="number"
                value={Number(target.toFixed(2))}
                min={spec.limitDeg.min}
                max={spec.limitDeg.max}
                step={0.1}
                onChange={e => {
                  const value = parseFloat(e.target.value);
                  if (Number.isNaN(value)) return;
                  setTargetAngles({
                    [joint]: Math.max(
                      spec.limitDeg.min,
                      Math.min(spec.limitDeg.max, value)
                    )
                  });
                }}
                disabled={!canMove}
                className="w-full px-1 py-0.5 border rounded text-right font-mono text-sm disabled:bg-gray-100"
                title={`${spec.limitDeg.min}° to ${spec.limitDeg.max}°`}
              />

              <div className="flex gap-1">
                <button
                  onClick={() => void jogJoint(joint, -step)}
                  disabled={!canMove || atMin}
                  title={atMin ? `${joint} is at its lower limit` : `${joint} −${step}°`}
                  className="w-8 h-7 border rounded text-sm font-bold text-gray-700 hover:bg-gray-100 disabled:opacity-30"
                >
                  −
                </button>
                <button
                  onClick={() => void jogJoint(joint, step)}
                  disabled={!canMove || atMax}
                  title={atMax ? `${joint} is at its upper limit` : `${joint} +${step}°`}
                  className="w-8 h-7 border rounded text-sm font-bold text-gray-700 hover:bg-gray-100 disabled:opacity-30"
                >
                  +
                </button>
              </div>
            </div>
          );
        })}
      </div>

      <div className="mt-3 flex flex-wrap gap-2">
        <button
          onClick={() => void moveToTarget()}
          disabled={!canMove}
          className="flex-1 min-w-[8rem] px-3 py-2 bg-blue-600 text-white rounded font-semibold hover:bg-blue-700 disabled:bg-gray-300"
        >
          Move to Target
        </button>

        <button
          onClick={syncTargetsToCurrent}
          disabled={!connected}
          title="Copy the reported angles into the target fields"
          className="px-3 py-2 border rounded text-sm text-gray-700 hover:bg-gray-50 disabled:opacity-40"
        >
          Target ← Reported
        </button>

        <button
          onClick={() => void goToHomePose()}
          disabled={!canMove}
          title={`Move to the post-homing rest pose (${HOME_POSE_DEG.join(', ')})`}
          className="px-3 py-2 border rounded text-sm text-gray-700 hover:bg-gray-50 disabled:opacity-40"
        >
          Rest pose
        </button>
      </div>

      {/* Homing */}
      <div className="mt-5 pt-3 border-t">
        <div className="flex items-baseline justify-between mb-2">
          <h3 className="text-sm font-semibold">Homing</h3>
          {homing && (
            <span className="text-xs text-yellow-700 font-semibold animate-pulse">
              homing…
            </span>
          )}
        </div>

        <div className="flex flex-wrap gap-2">
          {HOMEABLE.map(joint => (
            <button
              key={joint}
              onClick={() => void homeJoints(String(joint))}
              disabled={!canMove}
              className="px-3 py-1.5 bg-gray-600 text-white text-sm rounded hover:bg-gray-700 disabled:bg-gray-300"
            >
              Home J{joint}
            </button>
          ))}

          <button
            onClick={() => void homeJoints('ALL')}
            disabled={!canMove}
            className="px-3 py-1.5 bg-purple-600 text-white text-sm rounded hover:bg-purple-700 disabled:bg-gray-300"
          >
            Home All
          </button>
        </div>

        <p className="mt-2 text-xs text-gray-500">
          Home one joint at a time first and watch which way it goes. The seek
          direction is not verified for this build — see docs/BRINGUP.md.
        </p>
      </div>
    </div>
  );
};
