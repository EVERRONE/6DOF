// Snapping the tool square with a frame's axes.
//
// The set of square attitudes is generated, not typed, so the first thing worth
// checking is that what came out is 24 genuine rotations and not 48 including
// mirror images. After that the question is whether "nearest" means what an
// operator would expect it to mean.

import {
  AXIS_ALIGNED_ROTATIONS,
  MAX_ALIGNMENT_ERROR_DEG,
  alignmentErrorDeg,
  nearestAxisAligned
} from './axisAlign';
import { matrixToRpy, multiply3, rotationAboutAxis, rpyToMatrix, transpose3 } from './linalg';

const DEG = Math.PI / 180;

/** Angle between two rotations, degrees - independent of the module's own. */
function angleDeg(A: number[][], B: number[][]): number {
  const M = multiply3(transpose3(A), B);
  const trace = M[0][0] + M[1][1] + M[2][2];
  return (Math.acos(Math.max(-1, Math.min(1, (trace - 1) / 2))) * 180) / Math.PI;
}

function det(R: number[][]): number {
  return (
    R[0][0] * (R[1][1] * R[2][2] - R[1][2] * R[2][1]) -
    R[0][1] * (R[1][0] * R[2][2] - R[1][2] * R[2][0]) +
    R[0][2] * (R[1][0] * R[2][1] - R[1][1] * R[2][0])
  );
}

/** Uniformly distributed rotation, so coverage is tested over the whole sphere. */
function randomRotation(rand: () => number): number[][] {
  const u1 = rand(), u2 = rand(), u3 = rand();
  const x = Math.sqrt(1 - u1) * Math.sin(2 * Math.PI * u2);
  const y = Math.sqrt(1 - u1) * Math.cos(2 * Math.PI * u2);
  const z = Math.sqrt(u1) * Math.sin(2 * Math.PI * u3);
  const w = Math.sqrt(u1) * Math.cos(2 * Math.PI * u3);
  return [
    [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
    [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
    [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]
  ];
}

/** Deterministic, so a failure is reproducible. */
function seeded(seed: number): () => number {
  let s = seed;
  return () => {
    s = (s * 1103515245 + 12345) % 2147483648;
    return s / 2147483648;
  };
}

describe('the set of square attitudes', () => {
  it('is 24 rotations, not 48 with the mirror images left in', () => {
    expect(AXIS_ALIGNED_ROTATIONS).toHaveLength(24);
    for (const R of AXIS_ALIGNED_ROTATIONS) {
      // A reflection also sends axes to axes, and no arm can adopt one.
      expect(det(R)).toBeCloseTo(1, 12);
    }
  });

  it('holds each one once', () => {
    const seen = new Set(AXIS_ALIGNED_ROTATIONS.map(R => JSON.stringify(R)));
    expect(seen.size).toBe(24);
  });

  it('sends every axis onto an axis', () => {
    for (const R of AXIS_ALIGNED_ROTATIONS) {
      for (let k = 0; k < 3; k++) {
        const column = [R[0][k], R[1][k], R[2][k]].map(Math.abs).sort();
        expect(column).toEqual([0, 0, 1]);
      }
    }
  });
});

describe('finding the nearest one', () => {
  it('leaves an attitude that is already square alone', () => {
    for (const R of AXIS_ALIGNED_ROTATIONS) {
      const found = nearestAxisAligned(R);
      expect(found.errorDeg).toBeCloseTo(0, 9);
      expect(found.rotation).toEqual(R);
    }
  });

  it('reports the error an operator would measure with a gauge', () => {
    // Two degrees off about X, from square. The number on the panel should be
    // the two degrees, not some Euler-angle proxy for it.
    const R = multiply3(rotationAboutAxis({ x: 1, y: 0, z: 0 }, 2 * DEG), rpyToMatrix({ roll: 0, pitch: 0, yaw: 0 }));
    expect(alignmentErrorDeg(R)).toBeCloseTo(2, 6);
  });

  it('measures a tilt about a slanted axis just as well', () => {
    // Euler-angle rounding gets this wrong; a geodesic distance does not.
    const axis = { x: 1, y: 1, z: 1 };
    const R = rotationAboutAxis(axis, 3.5 * DEG);
    expect(alignmentErrorDeg(R)).toBeCloseTo(3.5, 6);
  });

  it('really finds the nearest, not merely a near one', () => {
    const rand = seeded(7);
    for (let i = 0; i < 500; i++) {
      const R = randomRotation(rand);
      const found = nearestAxisAligned(R);
      const brute = Math.min(...AXIS_ALIGNED_ROTATIONS.map(C => angleDeg(C, R)));
      expect(found.errorDeg).toBeCloseTo(brute, 9);
      expect(angleDeg(found.rotation, R)).toBeCloseTo(brute, 9);
    }
  });

  it('is never further away than the documented ceiling', () => {
    // What bounds the surprise: however crooked the arm is, straightening costs
    // no more wrist travel than this.
    const rand = seeded(11);
    let worst = 0;
    for (let i = 0; i < 4000; i++) {
      worst = Math.max(worst, alignmentErrorDeg(randomRotation(rand)));
    }
    expect(worst).toBeLessThanOrEqual(MAX_ALIGNMENT_ERROR_DEG);
    // And the ceiling is not padded to the point of being useless.
    expect(worst).toBeGreaterThan(50);
  });

  it('snaps a nearly-upside-down tool to upside down rather than to upright', () => {
    // 175 degrees about X. The nearest square attitude is the flipped one, and a
    // solver that reached for the identity would swing the wrist the long way.
    const R = rotationAboutAxis({ x: 1, y: 0, z: 0 }, 175 * DEG);
    const found = nearestAxisAligned(R);
    expect(found.errorDeg).toBeCloseTo(5, 6);
    expect(found.axes).toEqual(['+X', '-Y', '-Z']);
  });
});

describe('saying where the tool will end up', () => {
  it('labels the axes the way the frame sees them', () => {
    // A quarter turn about Z: tool X now lies along the frame's +Y.
    const found = nearestAxisAligned(rotationAboutAxis({ x: 0, y: 0, z: 1 }, 90 * DEG));
    expect(found.errorDeg).toBeCloseTo(0, 9);
    expect(found.axes).toEqual(['+Y', '-X', '+Z']);
  });

  it('is not the same as upright - square has 24 answers', () => {
    // Worth pinning, because "straighten" sounds like it should mean one thing.
    const labels = new Set(AXIS_ALIGNED_ROTATIONS.map(R => nearestAxisAligned(R).axes.join(' ')));
    expect(labels.size).toBe(24);
  });
});

describe('surviving the trip through roll/pitch/yaw', () => {
  it('reproduces every square attitude exactly, gimbal lock included', () => {
    // The solver takes a pose as RPY, so the matrix has to go through
    // matrixToRpy and come back unchanged. Eight of these sit at pitch = +/-90,
    // where roll and yaw stop being separable and the conversion takes a
    // different branch - the one place this could quietly lose a right angle.
    for (const R of AXIS_ALIGNED_ROTATIONS) {
      const round = rpyToMatrix(matrixToRpy(R));
      expect(angleDeg(round, R)).toBeLessThan(1e-9);
    }
  });

  it('covers the gimbal-lock branch rather than assuming it is unreachable', () => {
    const pitched = AXIS_ALIGNED_ROTATIONS.filter(
      R => Math.abs(Math.abs(matrixToRpy(R).pitch) - Math.PI / 2) < 1e-9
    );
    expect(pitched).toHaveLength(8);
  });
});
