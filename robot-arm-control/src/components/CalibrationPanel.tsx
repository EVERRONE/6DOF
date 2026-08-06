import React, { useEffect, useState } from 'react';
import { useRobotStore } from '../store/robotStore';
import { ConnectionStatus } from '../types/robot';

const JOINTS = ['J1', 'J2', 'J3', 'J4', 'J5', 'J6'] as const;

type CalInput = {
  scale: string;
  offset: string;
  zeroAngle: string;
  jogDelta: string;
  jogSpeed: string;
};

type HomePoseInput = {
  enabled: boolean;
  speedDegS: string;
  J2: string;
  J3: string;
  J4: string;
  J5: string;
};

const createDefaultInputs = (): CalInput[] => (
  JOINTS.map(() => ({
    scale: '1',
    offset: '0',
    zeroAngle: '0',
    jogDelta: '5',
    jogSpeed: '30'
  }))
);

const createDefaultHomePoseInput = (): HomePoseInput => ({
  enabled: false,
  speedDegS: '10',
  J2: '0',
  J3: '0',
  J4: '0',
  J5: '0'
});

export const CalibrationPanel: React.FC = () => {
  const {
    connectionStatus,
    firmwareConfig,
    currentAngles,
    manualSpeed,
    setManualSpeed,
    requestConfig,
    requestHomePose,
    setCalibration,
    zeroCalibration,
    saveCalibration,
    loadCalibration,
    resetCalibration,
    jogRelative,
    setHomePoseEnabled,
    setHomePoseAll,
    setHomePoseSpeed,
    saveHomePose,
    loadHomePose,
    resetHomePose
  } = useRobotStore();

  const [inputs, setInputs] = useState<CalInput[]>(createDefaultInputs);
  const [homePose, setHomePose] = useState<HomePoseInput>(createDefaultHomePoseInput);
  const [isExpanded, setIsExpanded] = useState(false);

  const isConnected = connectionStatus === ConnectionStatus.CONNECTED;

  useEffect(() => {
    if (!firmwareConfig || firmwareConfig.joints.length < 6) return;

    setInputs((prev) => prev.map((entry, index) => {
      const cfg = firmwareConfig.joints[index];
      return {
        ...entry,
        scale: Number.isFinite(cfg.cal?.scale) ? cfg.cal.scale.toString() : entry.scale,
        offset: Number.isFinite(cfg.cal?.offset) ? cfg.cal.offset.toString() : entry.offset,
        zeroAngle: entry.zeroAngle,
        jogSpeed: entry.jogSpeed
      };
    }));

    const hp = firmwareConfig.homePose;
    if (hp && Array.isArray(hp.jointsDeg) && hp.jointsDeg.length === 6) {
      setHomePose({
        enabled: !!hp.enabled,
        speedDegS: Number.isFinite(hp.speedDegS) ? hp.speedDegS.toString() : '10',
        J2: Number.isFinite(hp.jointsDeg[1]) ? hp.jointsDeg[1].toString() : '0',
        J3: Number.isFinite(hp.jointsDeg[2]) ? hp.jointsDeg[2].toString() : '0',
        J4: Number.isFinite(hp.jointsDeg[3]) ? hp.jointsDeg[3].toString() : '0',
        J5: Number.isFinite(hp.jointsDeg[4]) ? hp.jointsDeg[4].toString() : '0'
      });
    }
  }, [firmwareConfig]);

  const updateInput = (index: number, patch: Partial<CalInput>) => {
    setInputs((prev) => prev.map((entry, i) => (i === index ? { ...entry, ...patch } : entry)));
  };

  const parseNumber = (value: string, fallback: number) => {
    const parsed = parseFloat(value);
    return Number.isFinite(parsed) ? parsed : fallback;
  };

  return (
    <div className="p-4 bg-white border-b">
      <div className="flex items-center justify-between mb-3">
        <button
          type="button"
          onClick={() => setIsExpanded((prev) => !prev)}
          className="flex items-center gap-3 text-left"
          aria-expanded={isExpanded}
          aria-controls="calibration-panel-content"
        >
          <h2 className="text-xl font-bold">Calibration</h2>
          <span
            className={`inline-flex h-6 w-6 items-center justify-center rounded border text-sm text-gray-600 transition-transform ${
              isExpanded ? 'rotate-180' : ''
            }`}
            aria-hidden="true"
          >
            <svg
              viewBox="0 0 20 20"
              fill="none"
              stroke="currentColor"
              strokeWidth="2"
              className="h-4 w-4"
            >
              <path d="M5 7.5L10 12.5L15 7.5" strokeLinecap="round" strokeLinejoin="round" />
            </svg>
          </span>
        </button>
        <button
          onClick={requestConfig}
          disabled={!isConnected}
          className="px-3 py-1 text-sm bg-gray-700 text-white rounded hover:bg-gray-800 disabled:bg-gray-400"
        >
          Refresh CFG
        </button>
      </div>

      {isExpanded && (
        <div id="calibration-panel-content">
          <div className="flex flex-wrap gap-2 mb-4">
            <button
              onClick={loadCalibration}
              disabled={!isConnected}
              className="px-3 py-1 text-sm bg-blue-600 text-white rounded hover:bg-blue-700 disabled:bg-gray-400"
            >
              Load EEPROM
            </button>
            <button
              onClick={saveCalibration}
              disabled={!isConnected}
              className="px-3 py-1 text-sm bg-green-600 text-white rounded hover:bg-green-700 disabled:bg-gray-400"
            >
              Save EEPROM
            </button>
            <button
              onClick={resetCalibration}
              disabled={!isConnected}
              className="px-3 py-1 text-sm bg-red-600 text-white rounded hover:bg-red-700 disabled:bg-gray-400"
            >
              Reset Defaults
            </button>
          </div>

          {!firmwareConfig && (
            <div className="text-xs text-red-600 mb-3">
              Firmware config not loaded yet. Click "Refresh CFG" after connecting.
            </div>
          )}

          <div className="border rounded p-3 mb-4">
            <h3 className="font-semibold mb-2">Operational Home Pose (J2-J5)</h3>
            <div className="flex items-center gap-2 mb-2">
              <input
                id="homepose-enabled"
                type="checkbox"
                checked={homePose.enabled}
                onChange={(e) => setHomePose((prev) => ({ ...prev, enabled: e.target.checked }))}
                disabled={!isConnected}
                className="w-4 h-4"
              />
              <label htmlFor="homepose-enabled" className="text-sm text-gray-700">
                Enable post-home move after H ALL
              </label>
            </div>

            <div className="grid grid-cols-2 gap-2 mb-2">
              <label className="text-xs text-gray-600">
                Home speed (deg/s)
                <input
                  type="number"
                  value={homePose.speedDegS}
                  onChange={(e) => setHomePose((prev) => ({ ...prev, speedDegS: e.target.value }))}
                  className="mt-1 w-full border rounded px-2 py-1 text-sm"
                  step="0.1"
                />
              </label>
              <div className="text-xs text-gray-500 self-end">
                Trigger: only after <code>H ALL</code>
              </div>
            </div>
            <p className="text-xs text-gray-500 mb-2">
              For <code>J2..J5</code>, these HP targets are used as the URDF default anchor in
              FK/IK/3D. Calibration (<code>CAL</code>) is separate and only changes scale/offset.
            </p>

            <div className="grid grid-cols-4 gap-2 mb-3">
              <label className="text-xs text-gray-600">
                J2
                <input
                  type="number"
                  value={homePose.J2}
                  onChange={(e) => setHomePose((prev) => ({ ...prev, J2: e.target.value }))}
                  className="mt-1 w-full border rounded px-2 py-1 text-sm"
                  step="0.1"
                />
              </label>
              <label className="text-xs text-gray-600">
                J3
                <input
                  type="number"
                  value={homePose.J3}
                  onChange={(e) => setHomePose((prev) => ({ ...prev, J3: e.target.value }))}
                  className="mt-1 w-full border rounded px-2 py-1 text-sm"
                  step="0.1"
                />
              </label>
              <label className="text-xs text-gray-600">
                J4
                <input
                  type="number"
                  value={homePose.J4}
                  onChange={(e) => setHomePose((prev) => ({ ...prev, J4: e.target.value }))}
                  className="mt-1 w-full border rounded px-2 py-1 text-sm"
                  step="0.1"
                />
              </label>
              <label className="text-xs text-gray-600">
                J5
                <input
                  type="number"
                  value={homePose.J5}
                  onChange={(e) => setHomePose((prev) => ({ ...prev, J5: e.target.value }))}
                  className="mt-1 w-full border rounded px-2 py-1 text-sm"
                  step="0.1"
                />
              </label>
            </div>

            <div className="flex flex-wrap gap-2">
              <button
                onClick={async () => {
                  const j2 = parseNumber(homePose.J2, 0);
                  const j3 = parseNumber(homePose.J3, 0);
                  const j4 = parseNumber(homePose.J4, 0);
                  const j5 = parseNumber(homePose.J5, 0);
                  const speedDegS = parseNumber(homePose.speedDegS, 10);
                  await setHomePoseAll(j2, j3, j4, j5);
                  await setHomePoseSpeed(speedDegS);
                  await setHomePoseEnabled(homePose.enabled);
                  await requestConfig();
                }}
                disabled={!isConnected}
                className="px-3 py-1 text-sm bg-indigo-600 text-white rounded hover:bg-indigo-700 disabled:bg-gray-400"
              >
                Apply Runtime
              </button>
              <button
                onClick={async () => {
                  await saveHomePose();
                }}
                disabled={!isConnected}
                className="px-3 py-1 text-sm bg-green-600 text-white rounded hover:bg-green-700 disabled:bg-gray-400"
              >
                Save HP EEPROM
              </button>
              <button
                onClick={async () => {
                  await loadHomePose();
                  await requestHomePose();
                }}
                disabled={!isConnected}
                className="px-3 py-1 text-sm bg-blue-600 text-white rounded hover:bg-blue-700 disabled:bg-gray-400"
              >
                Load HP EEPROM
              </button>
              <button
                onClick={async () => {
                  await resetHomePose();
                  await requestHomePose();
                }}
                disabled={!isConnected}
                className="px-3 py-1 text-sm bg-red-600 text-white rounded hover:bg-red-700 disabled:bg-gray-400"
              >
                Reset HP Defaults
              </button>
            </div>
          </div>

          <div className="space-y-4">
            {JOINTS.map((joint, index) => {
              const entry = inputs[index];
              const currentAngle = currentAngles[joint];

              return (
                <div key={joint} className="border rounded p-3">
                  <div className="flex items-center justify-between mb-2">
                    <h3 className="font-semibold">{joint}</h3>
                    <span className="text-sm text-gray-600">Current: {currentAngle.toFixed(2)}deg</span>
                  </div>

                  <div className="grid grid-cols-2 gap-2 mb-2">
                    <label className="text-xs text-gray-600">
                      Scale
                      <input
                        type="number"
                        value={entry.scale}
                        onChange={(e) => updateInput(index, { scale: e.target.value })}
                        className="mt-1 w-full border rounded px-2 py-1 text-sm"
                        step="0.0001"
                      />
                    </label>
                    <label className="text-xs text-gray-600">
                      Offset (deg)
                      <input
                        type="number"
                        value={entry.offset}
                        onChange={(e) => updateInput(index, { offset: e.target.value })}
                        className="mt-1 w-full border rounded px-2 py-1 text-sm"
                        step="0.01"
                      />
                    </label>
                  </div>

                  <div className="flex flex-wrap gap-2 mb-3">
                    <button
                      onClick={() => {
                        const scale = parseNumber(entry.scale, 1);
                        const offset = parseNumber(entry.offset, 0);
                        setCalibration(index + 1, scale, offset);
                      }}
                      disabled={!isConnected}
                      className="px-3 py-1 text-sm bg-indigo-600 text-white rounded hover:bg-indigo-700 disabled:bg-gray-400"
                    >
                      Apply
                    </button>
                    <button
                      onClick={() => updateInput(index, { zeroAngle: currentAngle.toFixed(2) })}
                      className="px-3 py-1 text-sm bg-gray-200 text-gray-800 rounded hover:bg-gray-300"
                    >
                      Use Current
                    </button>
                  </div>

                  <div className="grid grid-cols-2 gap-2 mb-2">
                    <label className="text-xs text-gray-600">
                      Logical Angle Here (deg)
                      <input
                        type="number"
                        value={entry.zeroAngle}
                        onChange={(e) => updateInput(index, { zeroAngle: e.target.value })}
                        className="mt-1 w-full border rounded px-2 py-1 text-sm"
                        step="0.01"
                      />
                    </label>
                    <button
                      onClick={() => {
                        const logicalDeg = parseNumber(entry.zeroAngle, currentAngle);
                        zeroCalibration(index + 1, logicalDeg);
                      }}
                      disabled={!isConnected}
                      className="self-end px-3 py-1 text-sm bg-yellow-600 text-white rounded hover:bg-yellow-700 disabled:bg-gray-400"
                    >
                      Zero Here
                    </button>
                  </div>

                  <div className="grid grid-cols-3 gap-2">
                    <label className="text-xs text-gray-600">
                      Jog Delta (deg)
                      <input
                        type="number"
                        value={entry.jogDelta}
                        onChange={(e) => updateInput(index, { jogDelta: e.target.value })}
                        className="mt-1 w-full border rounded px-2 py-1 text-sm"
                        step="0.1"
                      />
                    </label>
                    <label className="text-xs text-gray-600">
                      Jog Speed (deg/s)
                      <input
                        type="number"
                        value={entry.jogSpeed}
                        onChange={(e) => {
                          updateInput(index, { jogSpeed: e.target.value });
                          const parsed = parseNumber(e.target.value, manualSpeed);
                          if (Number.isFinite(parsed)) setManualSpeed(parsed);
                        }}
                        className="mt-1 w-full border rounded px-2 py-1 text-sm"
                        step="1"
                      />
                    </label>
                    <div className="flex items-end gap-2">
                      <button
                        onClick={() => {
                          const delta = parseNumber(entry.jogDelta, 0);
                          const speed = parseNumber(entry.jogSpeed, manualSpeed);
                          jogRelative(index + 1, -Math.abs(delta), speed);
                        }}
                        disabled={!isConnected}
                        className="px-3 py-1 text-sm bg-gray-600 text-white rounded hover:bg-gray-700 disabled:bg-gray-400"
                      >
                        Jog -
                      </button>
                      <button
                        onClick={() => {
                          const delta = parseNumber(entry.jogDelta, 0);
                          const speed = parseNumber(entry.jogSpeed, manualSpeed);
                          jogRelative(index + 1, Math.abs(delta), speed);
                        }}
                        disabled={!isConnected}
                        className="px-3 py-1 text-sm bg-gray-600 text-white rounded hover:bg-gray-700 disabled:bg-gray-400"
                      >
                        Jog +
                      </button>
                    </div>
                  </div>
                </div>
              );
            })}
          </div>

          <p className="text-xs text-gray-500 mt-4">
            Tip: approach each reference angle from the same direction to minimize backlash.
          </p>
        </div>
      )}
    </div>
  );
};
