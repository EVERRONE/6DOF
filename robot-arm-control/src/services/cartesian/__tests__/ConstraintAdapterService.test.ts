import { FirmwareConfig } from '../../../types/robot';
import { ConstraintAdapterService } from '../ConstraintAdapterService';

const createFirmwareConfig = (): FirmwareConfig => ({
  version: 1,
  capabilities: {
    trajectoryQueue: true,
    trajectoryMaxPoints: 256,
    trajectoryPointFormat: 'hermite_v1'
  },
  joints: [
    { min: -170, max: 170, stepsPerDeg: 1, invertDir: false, hasEndstop: true, homeTowardMin: true, homeLogicalDeg: 0, postHomeOffsetDeg: 0, urdfOffsetDeg: 0, cal: { scale: 1, offset: 0 } },
    { min: -90, max: 120, stepsPerDeg: 1, invertDir: false, hasEndstop: true, homeTowardMin: true, homeLogicalDeg: 0, postHomeOffsetDeg: 0, urdfOffsetDeg: 2, cal: { scale: 1, offset: 0 } },
    { min: -100, max: 100, stepsPerDeg: 1, invertDir: false, hasEndstop: true, homeTowardMin: true, homeLogicalDeg: 0, postHomeOffsetDeg: 0, urdfOffsetDeg: -3, cal: { scale: 1, offset: 0 } },
    { min: -180, max: 180, stepsPerDeg: 1, invertDir: false, hasEndstop: true, homeTowardMin: true, homeLogicalDeg: 0, postHomeOffsetDeg: 0, urdfOffsetDeg: 1, cal: { scale: 1, offset: 0 } },
    { min: -120, max: 90, stepsPerDeg: 1, invertDir: false, hasEndstop: true, homeTowardMin: true, homeLogicalDeg: 0, postHomeOffsetDeg: 0, urdfOffsetDeg: 10, cal: { scale: 1, offset: 0 } },
    { min: -150, max: 150, stepsPerDeg: 1, invertDir: false, hasEndstop: true, homeTowardMin: true, homeLogicalDeg: 0, postHomeOffsetDeg: 0, urdfOffsetDeg: -5, cal: { scale: 1, offset: 0 } }
  ],
  pulseWidthUs: 2,
  dirSetupUs: 2,
  endstopDebounceMs: 3
});

describe('ConstraintAdapterService', () => {
  test('builds normalized runtime constraints from firmware config', () => {
    const contract = ConstraintAdapterService.fromFirmwareConfig(createFirmwareConfig());
    expect(contract).not.toBeNull();
    if (!contract) return;

    expect(contract.limitsSource).toBe('firmware');
    expect(contract.angleFrame).toBe('urdf');
    expect(contract.joints).toHaveLength(6);
    expect(contract.jointLimitsRad.min).toHaveLength(6);
    expect(contract.jointLimitsRad.max).toHaveLength(6);
  });

  test('handles inverted URDF directions when computing URDF-frame bounds', () => {
    const contract = ConstraintAdapterService.fromFirmwareConfig(createFirmwareConfig());
    expect(contract).not.toBeNull();
    if (!contract) return;

    // J5 uses URDF direction -1 in angle mapping defaults.
    const j5 = contract.joints[4];
    expect(j5.logicalMinDeg).toBe(-120);
    expect(j5.logicalMaxDeg).toBe(90);
    expect(j5.urdfMinDeg).toBeCloseTo(-80, 6);  // min(logicalMax * -1 + offset, logicalMin * -1 + offset)
    expect(j5.urdfMaxDeg).toBeCloseTo(130, 6);
  });
});
