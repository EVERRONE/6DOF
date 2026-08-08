import { resolveDirectionVector, applyDirectionalOffset } from '../frames';

describe('resolveDirectionVector — base frame', () => {
  // The world frame is X forward, Y left, Z up. These four assertions are the
  // whole contract; if one of them flips, the arm moves the wrong way.
  it('maps right to -Y', () => {
    expect(resolveDirectionVector('right', 'base')).toEqual({ x: 0, y: -1, z: 0 });
  });

  it('maps left to +Y', () => {
    expect(resolveDirectionVector('left', 'base')).toEqual({ x: 0, y: 1, z: 0 });
  });

  it('maps forward to +X', () => {
    expect(resolveDirectionVector('forward', 'base')).toEqual({ x: 1, y: 0, z: 0 });
  });

  it('maps up to +Z', () => {
    expect(resolveDirectionVector('up', 'base')).toEqual({ x: 0, y: 0, z: 1 });
  });

  it('makes opposite directions exact negations', () => {
    for (const [a, b] of [
      ['right', 'left'],
      ['forward', 'backward'],
      ['up', 'down']
    ] as const) {
      const va = resolveDirectionVector(a, 'base');
      const vb = resolveDirectionVector(b, 'base');
      expect(vb).toEqual({ x: -va.x || 0, y: -va.y || 0, z: -va.z || 0 });
    }
  });
});

describe('resolveDirectionVector — view frame', () => {
  it('is identical to the base frame at yaw 0', () => {
    for (const dir of ['right', 'left', 'forward', 'backward', 'up', 'down'] as const) {
      expect(resolveDirectionVector(dir, 'view', 0)).toEqual(resolveDirectionVector(dir, 'base'));
    }
  });

  it('rotates the basis with the operator: at yaw 90 the operator faces +Y', () => {
    expect(resolveDirectionVector('forward', 'view', 90)).toEqual({ x: 0, y: 1, z: 0 });
    // Facing +Y, the operator's right hand points along +X.
    expect(resolveDirectionVector('right', 'view', 90)).toEqual({ x: 1, y: 0, z: 0 });
  });

  it('flips right and left when the operator stands opposite the base', () => {
    expect(resolveDirectionVector('right', 'view', 180)).toEqual({ x: 0, y: 1, z: 0 });
    expect(resolveDirectionVector('left', 'view', 180)).toEqual({ x: 0, y: -1, z: 0 });
  });

  it('leaves the vertical axis untouched at any yaw', () => {
    for (const yaw of [0, 37, 90, 180, -125]) {
      expect(resolveDirectionVector('up', 'view', yaw)).toEqual({ x: 0, y: 0, z: 1 });
    }
  });

  it('keeps every direction a unit vector', () => {
    for (const yaw of [0, 30, 45, 137, -90]) {
      for (const dir of ['right', 'left', 'forward', 'backward'] as const) {
        const v = resolveDirectionVector(dir, 'view', yaw);
        expect(Math.hypot(v.x, v.y, v.z)).toBeCloseTo(1, 9);
      }
    }
  });
});

describe('applyDirectionalOffset', () => {
  const start = { x: 0.2, y: 0.05, z: 0.15 };

  it('converts millimetres to metres when offsetting', () => {
    const { target } = applyDirectionalOffset(start, 'right', 5, 'base');
    expect(target.x).toBeCloseTo(0.2, 9);
    expect(target.y).toBeCloseTo(0.045, 9); // 5 mm toward -Y
    expect(target.z).toBeCloseTo(0.15, 9);
  });

  it('reports the delta in millimetres for the response echo', () => {
    const { deltaMm } = applyDirectionalOffset(start, 'right', 5, 'base');
    expect(deltaMm).toEqual({ x: 0, y: -5, z: 0 });
  });

  it('does not mutate the input position', () => {
    const before = { ...start };
    applyDirectionalOffset(start, 'up', 25, 'base');
    expect(start).toEqual(before);
  });

  it('respects the view frame when computing the target', () => {
    const { deltaMm } = applyDirectionalOffset(start, 'right', 10, 'view', 90);
    expect(deltaMm).toEqual({ x: 10, y: 0, z: 0 });
  });
});
