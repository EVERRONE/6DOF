import { ForwardKinematics } from './ForwardKinematics';

type Vec3 = { x: number; y: number; z: number };

type Capsule = {
  a: Vec3;
  b: Vec3;
  radius: number;
  name: string;
};

export interface CollisionCheckResult {
  collisionFree: boolean;
  collisionPairs: string[];
  minimumClearanceM: number;
}

const sub = (a: Vec3, b: Vec3): Vec3 => ({ x: a.x - b.x, y: a.y - b.y, z: a.z - b.z });
const add = (a: Vec3, b: Vec3): Vec3 => ({ x: a.x + b.x, y: a.y + b.y, z: a.z + b.z });
const scale = (v: Vec3, s: number): Vec3 => ({ x: v.x * s, y: v.y * s, z: v.z * s });
const dot = (a: Vec3, b: Vec3): number => a.x * b.x + a.y * b.y + a.z * b.z;
const norm = (v: Vec3): number => Math.hypot(v.x, v.y, v.z);

const toVec = (t: number[][]): Vec3 => ({ x: t[0][3], y: t[1][3], z: t[2][3] });

const segmentDistance = (p1: Vec3, q1: Vec3, p2: Vec3, q2: Vec3): number => {
  const d1 = sub(q1, p1);
  const d2 = sub(q2, p2);
  const r = sub(p1, p2);
  const a = dot(d1, d1);
  const e = dot(d2, d2);
  const f = dot(d2, r);
  const eps = 1e-9;

  let s = 0;
  let t = 0;

  if (a <= eps && e <= eps) {
    return norm(sub(p1, p2));
  }
  if (a <= eps) {
    t = Math.min(1, Math.max(0, f / e));
  } else {
    const c = dot(d1, r);
    if (e <= eps) {
      s = Math.min(1, Math.max(0, -c / a));
    } else {
      const b = dot(d1, d2);
      const denom = a * e - b * b;
      if (Math.abs(denom) > eps) {
        s = Math.min(1, Math.max(0, (b * f - c * e) / denom));
      }
      t = (b * s + f) / e;
      if (t < 0) {
        t = 0;
        s = Math.min(1, Math.max(0, -c / a));
      } else if (t > 1) {
        t = 1;
        s = Math.min(1, Math.max(0, (b - c) / a));
      }
    }
  }

  const c1 = add(p1, scale(d1, s));
  const c2 = add(p2, scale(d2, t));
  return norm(sub(c1, c2));
};

const shouldCheckPair = (i: number, j: number): boolean => {
  // Adjacent links share joints and always overlap.
  if (Math.abs(i - j) <= 1) return false;
  // Base-to-EE and upper-arm-to-EE cannot physically collide on this geometry.
  if ((i === 0 && j === 5) || (i === 1 && j === 5)) return false;
  return true;
};

export class CollisionModel {
  static checkSelfCollision(jointAnglesDeg: number[]): CollisionCheckResult {
    const fk = ForwardKinematics.solve(jointAnglesDeg);
    if (!fk.success || fk.jointTransforms.length !== 6) {
      return {
        collisionFree: false,
        collisionPairs: ['fk_failure'],
        minimumClearanceM: 0
      };
    }

    const points = fk.jointTransforms.map((t) => toVec(t));
    const base: Vec3 = { x: 0, y: 0, z: 0 };
    const chainPoints = [base, ...points];

    const capsules: Capsule[] = [];
    for (let i = 0; i < chainPoints.length - 1; i++) {
      capsules.push({
        a: chainPoints[i],
        b: chainPoints[i + 1],
        // Capsule radii: 30mm for base/shoulder (i<2), 22mm for remaining links.
        // Conservative approximation — actual STL geometry varies per link.
        radius: i < 2 ? 0.03 : 0.022,
        name: `L${i}`
      });
    }

    const collisions: string[] = [];
    let minimumClearanceM = Number.POSITIVE_INFINITY;

    for (let i = 0; i < capsules.length; i++) {
      for (let j = i + 1; j < capsules.length; j++) {
        if (!shouldCheckPair(i, j)) continue;
        const dist = segmentDistance(capsules[i].a, capsules[i].b, capsules[j].a, capsules[j].b);
        const clearance = dist - capsules[i].radius - capsules[j].radius;
        minimumClearanceM = Math.min(minimumClearanceM, clearance);
        if (clearance < 0) {
          collisions.push(`${capsules[i].name}-${capsules[j].name}`);
        }
      }
    }

    return {
      collisionFree: collisions.length === 0,
      collisionPairs: collisions,
      minimumClearanceM
    };
  }
}
