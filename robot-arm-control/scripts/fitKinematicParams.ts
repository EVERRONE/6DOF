import * as fs from 'fs';
import * as path from 'path';
import { ForwardKinematics } from '../src/kinematics/ForwardKinematics';

type Sample = {
  jointsDeg: number[];
  measured: { x: number; y: number; z: number };
};

type FitResult = {
  tcpOffsetM: { x: number; y: number; z: number };
  rmseM: number;
  sampleCount: number;
  generatedAt: string;
};

const inputPath = process.argv[2]
  ? path.resolve(process.cwd(), process.argv[2])
  : path.resolve(__dirname, './kinematic_samples.json');

const outputPath = process.argv[3]
  ? path.resolve(process.cwd(), process.argv[3])
  : path.resolve(__dirname, './kinematic_fit_result.json');

if (!fs.existsSync(inputPath)) {
  console.error(`Input sample file not found: ${inputPath}`);
  console.error('Expected JSON array of { jointsDeg:[6], measured:{x,y,z} }');
  process.exit(1);
}

const samples = JSON.parse(fs.readFileSync(inputPath, 'utf8')) as Sample[];
if (!Array.isArray(samples) || samples.length === 0) {
  console.error('Sample file is empty or invalid');
  process.exit(1);
}

let sumDx = 0;
let sumDy = 0;
let sumDz = 0;
let used = 0;

for (const sample of samples) {
  if (!Array.isArray(sample.jointsDeg) || sample.jointsDeg.length !== 6) continue;
  const fk = ForwardKinematics.solve(sample.jointsDeg);
  if (!fk.success) continue;
  sumDx += sample.measured.x - fk.endEffectorPose.position.x;
  sumDy += sample.measured.y - fk.endEffectorPose.position.y;
  sumDz += sample.measured.z - fk.endEffectorPose.position.z;
  used += 1;
}

if (used === 0) {
  console.error('No valid samples for fitting');
  process.exit(1);
}

const tcpOffset = {
  x: sumDx / used,
  y: sumDy / used,
  z: sumDz / used
};

let sqErr = 0;
for (const sample of samples) {
  if (!Array.isArray(sample.jointsDeg) || sample.jointsDeg.length !== 6) continue;
  const fk = ForwardKinematics.solve(sample.jointsDeg);
  if (!fk.success) continue;
  const ex = sample.measured.x - (fk.endEffectorPose.position.x + tcpOffset.x);
  const ey = sample.measured.y - (fk.endEffectorPose.position.y + tcpOffset.y);
  const ez = sample.measured.z - (fk.endEffectorPose.position.z + tcpOffset.z);
  sqErr += ex * ex + ey * ey + ez * ez;
}

const rmse = Math.sqrt(sqErr / used);
const result: FitResult = {
  tcpOffsetM: tcpOffset,
  rmseM: rmse,
  sampleCount: used,
  generatedAt: new Date().toISOString()
};

fs.writeFileSync(outputPath, JSON.stringify(result, null, 2), 'utf8');
console.log(`Kinematic fit written to ${outputPath}`);
console.log(`Samples: ${result.sampleCount}, RMSE: ${(result.rmseM * 1000).toFixed(2)} mm`);

