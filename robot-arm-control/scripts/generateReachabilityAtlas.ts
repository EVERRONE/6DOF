import * as fs from 'fs';
import * as path from 'path';
import { ForwardKinematics } from '../src/kinematics/ForwardKinematics';
import { getJointLimits, radiansToDegrees } from '../src/kinematics/DHParameters';

type Cell = {
  support: number;
  seed: number[];
  branchId: string;
};

type Atlas = {
  version: string;
  voxelSizeM: number;
  bounds: {
    min: { x: number; y: number; z: number };
    max: { x: number; y: number; z: number };
  };
  sampleCount: number;
  generatedAt: string;
  cells: Record<string, Cell>;
};

const VOXEL = 0.02;
const SAMPLE_COUNT = 60000;

const outputPath = path.resolve(__dirname, '../src/kinematics/reachabilityAtlas.generated.json');

const limits = getJointLimits();
const minDeg = radiansToDegrees(limits.min);
const maxDeg = radiansToDegrees(limits.max);

const atlas: Atlas = {
  version: 'atlas_v1',
  voxelSizeM: VOXEL,
  bounds: {
    min: { x: -0.32, y: -0.32, z: 0.0 },
    max: { x: 0.32, y: 0.32, z: 0.42 }
  },
  sampleCount: 0,
  generatedAt: new Date().toISOString(),
  cells: {}
};

const keyFor = (x: number, y: number, z: number): string => {
  const ix = Math.floor((x - atlas.bounds.min.x) / VOXEL);
  const iy = Math.floor((y - atlas.bounds.min.y) / VOXEL);
  const iz = Math.floor((z - atlas.bounds.min.z) / VOXEL);
  return `${ix}:${iy}:${iz}`;
};

const inBounds = (x: number, y: number, z: number): boolean => (
  x >= atlas.bounds.min.x && x <= atlas.bounds.max.x &&
  y >= atlas.bounds.min.y && y <= atlas.bounds.max.y &&
  z >= atlas.bounds.min.z && z <= atlas.bounds.max.z
);

for (let i = 0; i < SAMPLE_COUNT; i++) {
  const q = minDeg.map((minValue, idx) => minValue + (maxDeg[idx] - minValue) * Math.random());
  const fk = ForwardKinematics.solve(q);
  if (!fk.success) continue;

  const { x, y, z } = fk.endEffectorPose.position;
  if (!inBounds(x, y, z)) continue;
  const key = keyFor(x, y, z);
  const cell = atlas.cells[key];

  if (!cell) {
    atlas.cells[key] = {
      support: 1,
      seed: q,
      branchId: q[2] >= 0 ? 'atlas_elbow_down' : 'atlas_elbow_up'
    };
  } else {
    cell.support += 1;
    if (cell.support % 5 === 0) {
      cell.seed = q;
      cell.branchId = q[2] >= 0 ? 'atlas_elbow_down' : 'atlas_elbow_up';
    }
  }
}

atlas.sampleCount = SAMPLE_COUNT;
atlas.generatedAt = new Date().toISOString();

fs.writeFileSync(outputPath, JSON.stringify(atlas, null, 2), 'utf8');
console.log(`Reachability atlas written to ${outputPath}`);
console.log(`Cells: ${Object.keys(atlas.cells).length}, samples: ${atlas.sampleCount}`);

