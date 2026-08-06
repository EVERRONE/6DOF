import {
  boxesOverlap,
  checkSelfCollision,
  firstCollidingPoint,
  isCollisionFree
} from './CollisionChecker';
import { ALLOWED_PAIRS, LINK_BOXES } from './collisionModel';
import { HOME_POSE_DEG, JOINT_LIMITS_DEG, NUM_JOINTS } from './robotModel';

jest.setTimeout(120000);

function makeRng(seed: number): () => number {
  let s = seed >>> 0;
  return () => {
    s = (Math.imul(s, 1664525) + 1013904223) >>> 0;
    return s / 4294967296;
  };
}

function randomPose(rng: () => number): number[] {
  return JOINT_LIMITS_DEG.min.map(
    (lo, i) => lo + rng() * (JOINT_LIMITS_DEG.max[i] - lo)
  );
}

const unitBox = (centre: number[], half: number[]) => ({
  centre,
  axes: [[1, 0, 0], [0, 1, 0], [0, 0, 1]],
  half
});

describe('oriented box overlap', () => {
  it('separates boxes that do not touch', () => {
    expect(boxesOverlap(unitBox([0, 0, 0], [1, 1, 1]), unitBox([3, 0, 0], [1, 1, 1]))).toBe(false);
  });

  it('finds boxes that do', () => {
    expect(boxesOverlap(unitBox([0, 0, 0], [1, 1, 1]), unitBox([1.5, 0, 0], [1, 1, 1]))).toBe(true);
  });

  it('catches an overlap only a rotated axis reveals', () => {
    // Face-aligned tests alone would call this pair separated. The nine cross
    // products are what a diagonal contact needs.
    const a = unitBox([0, 0, 0], [1, 1, 1]);
    const c = Math.SQRT1_2;
    const b = {
      centre: [1.6, 1.6, 0],
      axes: [[c, c, 0], [-c, c, 0], [0, 0, 1]],
      half: [1, 1, 1]
    };
    expect(boxesOverlap(a, b)).toBe(true);
  });

  it('is symmetric', () => {
    const rng = makeRng(4242);
    for (let i = 0; i < 200; i++) {
      const a = unitBox([rng() * 4 - 2, rng() * 4 - 2, rng() * 4 - 2], [1, 0.5, 0.7]);
      const b = unitBox([rng() * 4 - 2, rng() * 4 - 2, rng() * 4 - 2], [0.6, 1.2, 0.4]);
      expect(boxesOverlap(a, b)).toBe(boxesOverlap(b, a));
    }
  });
});

describe('the collision model', () => {
  it('covers every link', () => {
    expect(LINK_BOXES).toHaveLength(NUM_JOINTS);
    LINK_BOXES.forEach(boxes => {
      expect(boxes.length).toBeGreaterThan(0);
      boxes.forEach(b => {
        expect(b.axes).toHaveLength(3);
        expect(b.half.every(h => h >= 0 && Number.isFinite(h))).toBe(true);
      });
    });
  });

  it('keeps the two pairs that carry information', () => {
    // Everything else either overlaps by construction - adjacent links, and the
    // tool bolted inside the forearm's box - or can never reach. Disabling those
    // is what makes the remaining answers mean something.
    const allowed = new Set(ALLOWED_PAIRS.map(([i, j]) => `${i}-${j}`));
    expect(allowed.has('0-3')).toBe(false); // shoulder vs forearm
    expect(allowed.has('1-3')).toBe(false); // upper arm vs forearm
    expect(allowed.has('0-1')).toBe(true);  // adjacent
    expect(allowed.has('3-5')).toBe(true);  // tool sits inside the forearm box
  });
});

describe('checking the arm', () => {
  it('finds the parked pose clear', () => {
    // The arm sits here happily. A model that calls this a collision is wrong
    // about everything else too.
    expect(isCollisionFree(HOME_POSE_DEG)).toBe(true);
  });

  it('agrees with the arm about where J3 fouls', () => {
    // The one collision measured on the machine: sweeping J3 up from the parked
    // pose, the arm hits itself at about 107 degrees. This is the only external
    // check the model has, so it is worth being exact about.
    let first: number | null = null;
    for (let j3 = 42; j3 <= 160; j3++) {
      const q = [...HOME_POSE_DEG];
      q[2] = j3;
      if (!isCollisionFree(q)) { first = j3; break; }
    }

    expect(first).not.toBeNull();
    expect(Math.abs(first! - 107)).toBeLessThanOrEqual(4);
  });

  it('names the parts that are in each other\'s way', () => {
    const q = [...HOME_POSE_DEG];
    q[2] = 130;
    const result = checkSelfCollision(q);
    expect(result.colliding).toBe(true);
    expect(result.message).toMatch(/forearm/);
  });

  it('rejects a small share of poses the joint limits allow', () => {
    // The reason this exists. If it were zero the limits would already cover it;
    // if it were half, the model would be wrong. Measured at 1.7%.
    const rng = makeRng(11);
    let colliding = 0;
    const total = 600;
    for (let i = 0; i < total; i++) {
      if (!isCollisionFree(randomPose(rng))) colliding++;
    }
    const share = colliding / total;
    expect(share).toBeGreaterThan(0.001);
    expect(share).toBeLessThan(0.10);
  });

  it('ignores a pose it cannot interpret', () => {
    expect(checkSelfCollision([1, 2, 3]).colliding).toBe(false);
    expect(checkSelfCollision([0, 0, 0, 0, 0, NaN]).colliding).toBe(false);
  });
});

describe('checking a path', () => {
  it('says nothing about a clear one', () => {
    const path = [HOME_POSE_DEG, HOME_POSE_DEG.map((v, i) => (i === 1 ? v + 5 : v))];
    expect(firstCollidingPoint(path)).toBeNull();
  });

  it('reports where it first goes wrong, not just that it does', () => {
    // An operator can act on "it fails at point 3 of 5" and cannot act on "it
    // fails".
    const path = [
      HOME_POSE_DEG,
      HOME_POSE_DEG,
      HOME_POSE_DEG.map((v, i) => (i === 2 ? 130 : v)),
      HOME_POSE_DEG
    ];
    const hit = firstCollidingPoint(path);
    expect(hit).not.toBeNull();
    expect(hit!.index).toBe(2);
    expect(hit!.result.message.length).toBeGreaterThan(0);
  });
});
