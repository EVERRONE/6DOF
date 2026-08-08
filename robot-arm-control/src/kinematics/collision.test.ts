import {
  boxesOverlap,
  checkSelfCollision,
  firstCollidingPoint,
  isCollisionFree
} from './CollisionChecker';
import { ALLOWED_PAIRS, LINK_BOXES } from './collisionModel';
import { HOME_POSE_DEG, JOINT_LIMITS_DEG, NUM_JOINTS, resetToolFrame, setToolFrame } from './robotModel';
import { clearToolShape, flushOnFlange, setToolShape } from './toolGeometry';

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

// ---------------------------------------------------------------------------

describe('a tool on the flange', () => {
  // The pose used throughout: clear with a bare flange, and folded enough that
  // a 150 mm tool reaches back into the shoulder. Found by search over the joint
  // limits, then written down - a pose regenerated per run would make a failure
  // impossible to reproduce.
  const FOLDED = [-71, 19, 103, 6, 178, -280];
  const STAGE = { x: 0.08, y: 0.08, z: 0.15 };

  afterEach(() => clearToolShape());

  it('changes nothing while the arm is running bare', () => {
    clearToolShape();
    expect(checkSelfCollision([...HOME_POSE_DEG]).colliding).toBe(false);
    expect(checkSelfCollision(FOLDED).colliding).toBe(false);
  });

  it('catches the pose that drives it into the arm', () => {
    // The whole point. Without this the checker clears this pose and the tool
    // goes into the shoulder at speed.
    expect(setToolShape(flushOnFlange(STAGE)).ok).toBe(true);

    const hit = checkSelfCollision(FOLDED);
    expect(hit.colliding).toBe(true);
    expect(hit.message).toMatch(/tool/);
  });

  it('leaves the poses the arm actually works in alone', () => {
    // A guard that refuses the rest pose is a guard nobody will leave switched
    // on, so this has to hold for tools well past what this arm will carry.
    for (const size of [
      { x: 0.04, y: 0.04, z: 0.06 },
      { x: 0.08, y: 0.08, z: 0.15 },
      { x: 0.12, y: 0.12, z: 0.25 }
    ]) {
      expect(setToolShape(flushOnFlange(size)).ok).toBe(true);
      expect(checkSelfCollision([...HOME_POSE_DEG]).colliding).toBe(false);
    }
  });

  it('costs reach in proportion to its size, rather than all at once', () => {
    // Sampled across the joint limits: a bare arm loses 1.8% of poses to
    // self-collision, and a tool takes more of the joint space the bigger it is.
    // A number that jumped to most of the workspace would mean a box overlapping
    // by construction rather than a tool that genuinely does not fit.
    const refused = (size: { x: number; y: number; z: number } | null) => {
      if (size) setToolShape(flushOnFlange(size));
      else clearToolShape();
      const rng = makeRng(4242);
      let bad = 0;
      for (let n = 0; n < 1500; n++) if (checkSelfCollision(randomPose(rng)).colliding) bad++;
      return (100 * bad) / 1500;
    };

    const bare = refused(null);
    const small = refused({ x: 0.04, y: 0.04, z: 0.06 });
    const large = refused({ x: 0.12, y: 0.12, z: 0.25 });

    expect(bare).toBeLessThan(4);
    expect(small).toBeGreaterThan(bare);
    expect(large).toBeGreaterThan(small);
    expect(large).toBeLessThan(35);
  });

  it('hangs off the flange, not off the TCP', () => {
    // The two are different things and confusing them is the trap this whole
    // feature sits next to. A measured tool frame moves where the TCP is; the
    // box is bolted metal and does not move with it. If this ever fails, a
    // 150 mm tool starts being checked 300 mm out after somebody calibrates.
    setToolShape(flushOnFlange(STAGE));
    const before = checkSelfCollision(FOLDED);

    setToolFrame({ xyz: { x: 0, y: 0, z: 0.15 } });
    try {
      const after = checkSelfCollision(FOLDED);
      expect(after.colliding).toBe(before.colliding);
      expect(after.message).toBe(before.message);
    } finally {
      resetToolFrame();
    }
  });

  it('will not take a size that is not a size', () => {
    expect(setToolShape(flushOnFlange({ x: 0.05, y: 0, z: 0.1 })).ok).toBe(false);
    expect(setToolShape(flushOnFlange({ x: 0.05, y: -0.02, z: 0.1 })).ok).toBe(false);
    expect(setToolShape({ size: { x: 0.05, y: 0.05, z: 0.1 }, centre: { x: 0, y: 0, z: NaN } }).ok)
      .toBe(false);
  });

  it('reads a metre-long tool as a units mistake', () => {
    // Somebody types the millimetres into the metres field. A 1500 mm tool
    // refuses every pose, and the operator concludes the checker is broken
    // rather than that the entry is.
    const bad = setToolShape(flushOnFlange({ x: 1.5, y: 1.5, z: 1.5 }));
    expect(bad.ok).toBe(false);
    if (!bad.ok) expect(bad.reason).toMatch(/millimetres/i);
  });

  it('sits flush on the flange when asked to', () => {
    // The flange face is z = 0 and +Z points away from the arm, so a tool
    // sitting on it has its centre half its length out. Worth pinning because
    // getting it wrong buries the box inside the wrist, where it is checked
    // against nothing.
    const shape = flushOnFlange({ x: 0.04, y: 0.04, z: 0.1 });
    expect(shape.centre.z).toBeCloseTo(0.05, 9);
    expect(shape.centre.x).toBe(0);
    expect(shape.centre.y).toBe(0);
  });

  it('does not start blaming the forearm', () => {
    // The forearm's boxes enclose the wrist mount, so they overlap anything on
    // the flange by construction - 86% of poses in the first 25 mm, with or
    // without a tool. That pair stays off, and this is what says so: a tool must
    // not make ordinary poses report the forearm.
    setToolShape(flushOnFlange(STAGE));
    const rng = makeRng(99);
    for (let n = 0; n < 400; n++) {
      const result = checkSelfCollision(randomPose(rng));
      expect(result.message).not.toMatch(/forearm against tool|tool against forearm/);
    }
  });
});
