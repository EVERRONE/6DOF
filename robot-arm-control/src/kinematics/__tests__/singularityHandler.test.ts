import { SingularityHandler } from '../SingularityHandler';

describe('SingularityHandler', () => {
  test('flags wrist singularity near J5 = 0 deg', () => {
    const flags = SingularityHandler.detectFlags([0, 0, 0, 10, 0.1, 20], 1e-4, 1000);
    expect(flags).toContain('wrist_pitch_singularity');
  });

  test('applies deterministic wrist bypass near singularity', () => {
    const out = SingularityHandler.applyWristBypass([0, 0, 0, 30, 0, 40]);
    expect(out[3]).toBeCloseTo(70, 6);
    expect(out[5]).toBeCloseTo(0, 6);
  });

  test('stabilizes J6 drift while preserving J4+J6 sum near wrist singularity', () => {
    const prev = [0, 0, 0, 10, 0.2, 5];
    const proposal = [0, 0, 0, 2, 0.05, 18];
    const out = SingularityHandler.stabilizeWristStep(prev, proposal, 0.3);
    expect(out[3] + out[5]).toBeCloseTo(proposal[3] + proposal[5], 8);
    expect(Math.abs(out[5] - prev[5])).toBeLessThanOrEqual(0.300001);
  });

  test('stabilizes J1 drift when J1 XYZ sensitivity is low', () => {
    const prev = [20, 0, 0, 0, 0, 0];
    const proposal = [35, 0, 0, 0, 0, 0];
    const out = SingularityHandler.stabilizeShoulderStep(prev, proposal, 0.0002, 0.5);
    expect(out[0]).toBeCloseTo(20.5, 6);
  });

  test('does not clamp J1 when sensitivity is healthy', () => {
    const prev = [20, 0, 0, 0, 0, 0];
    const proposal = [35, 0, 0, 0, 0, 0];
    const out = SingularityHandler.stabilizeShoulderStep(prev, proposal, 0.01, 0.5);
    expect(out[0]).toBeCloseTo(35, 6);
  });
});
