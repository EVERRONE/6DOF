import atlasData from './reachabilityAtlas.generated.json';
import { Vector3 } from './types';

export interface ReachabilityCell {
  support: number;
  seed: number[];
  branchId: string;
}

export interface ReachabilityAtlas {
  version: string;
  voxelSizeM: number;
  bounds: {
    min: Vector3;
    max: Vector3;
  };
  sampleCount: number;
  generatedAt: string;
  cells: Record<string, ReachabilityCell>;
}

export interface ReachabilityProbe {
  likelyReachable: boolean;
  inAtlasBounds: boolean;
  reason: string;
  boundaryDistanceM: number;
  suggestedSeed?: number[];
  branchId?: string;
  /** Closest point within AABB bounds, NOT the true workspace surface. */
  closestReachablePoint?: Vector3;
}

const atlas = atlasData as ReachabilityAtlas;

const keyFor = (
  position: Vector3,
  atlasModel: ReachabilityAtlas
): string => {
  const ix = Math.floor((position.x - atlasModel.bounds.min.x) / atlasModel.voxelSizeM);
  const iy = Math.floor((position.y - atlasModel.bounds.min.y) / atlasModel.voxelSizeM);
  const iz = Math.floor((position.z - atlasModel.bounds.min.z) / atlasModel.voxelSizeM);
  return `${ix}:${iy}:${iz}`;
};

const inBounds = (position: Vector3, atlasModel: ReachabilityAtlas): boolean => (
  position.x >= atlasModel.bounds.min.x &&
  position.x <= atlasModel.bounds.max.x &&
  position.y >= atlasModel.bounds.min.y &&
  position.y <= atlasModel.bounds.max.y &&
  position.z >= atlasModel.bounds.min.z &&
  position.z <= atlasModel.bounds.max.z
);

const clampToBounds = (position: Vector3, atlasModel: ReachabilityAtlas): Vector3 => ({
  x: Math.min(atlasModel.bounds.max.x, Math.max(atlasModel.bounds.min.x, position.x)),
  y: Math.min(atlasModel.bounds.max.y, Math.max(atlasModel.bounds.min.y, position.y)),
  z: Math.min(atlasModel.bounds.max.z, Math.max(atlasModel.bounds.min.z, position.z))
});

const boundaryDistance = (position: Vector3, atlasModel: ReachabilityAtlas): number => {
  const distances = [
    position.x - atlasModel.bounds.min.x,
    atlasModel.bounds.max.x - position.x,
    position.y - atlasModel.bounds.min.y,
    atlasModel.bounds.max.y - position.y,
    position.z - atlasModel.bounds.min.z,
    atlasModel.bounds.max.z - position.z
  ];
  return Math.min(...distances);
};

const lookupWithNeighbors = (
  position: Vector3,
  atlasModel: ReachabilityAtlas
): ReachabilityCell | null => {
  const baseKey = keyFor(position, atlasModel);
  const direct = atlasModel.cells[baseKey];
  if (direct) return direct;

  const [ixStr, iyStr, izStr] = baseKey.split(':');
  const ix = Number.parseInt(ixStr, 10);
  const iy = Number.parseInt(iyStr, 10);
  const iz = Number.parseInt(izStr, 10);

  for (let dx = -1; dx <= 1; dx++) {
    for (let dy = -1; dy <= 1; dy++) {
      for (let dz = -1; dz <= 1; dz++) {
        const key = `${ix + dx}:${iy + dy}:${iz + dz}`;
        const cell = atlasModel.cells[key];
        if (cell) return cell;
      }
    }
  }

  return null;
};

export const getDefaultReachabilityAtlas = (): ReachabilityAtlas => atlas;

export const isReachabilityAtlasReady = (): boolean => (
  Boolean(atlas && atlas.voxelSizeM > 0 && atlas.bounds && atlas.cells)
);

export const classifyReachability = (
  position: Vector3,
  atlasModel: ReachabilityAtlas = atlas
): ReachabilityProbe => {
  const clamped = clampToBounds(position, atlasModel);
  const boundaryDistanceM = boundaryDistance(clamped, atlasModel);

  if (!inBounds(position, atlasModel)) {
    return {
      likelyReachable: false,
      inAtlasBounds: false,
      reason: 'outside_atlas_bounds',
      boundaryDistanceM: -Math.hypot(
        position.x - clamped.x,
        position.y - clamped.y,
        position.z - clamped.z
      ),
      closestReachablePoint: clamped
    };
  }

  const cell = lookupWithNeighbors(position, atlasModel);
  if (!cell) {
    return {
      likelyReachable: true,
      inAtlasBounds: true,
      reason: 'inside_bounds_no_local_cell',
      boundaryDistanceM
    };
  }

  return {
    likelyReachable: true,
    inAtlasBounds: true,
    reason: 'atlas_cell_found',
    boundaryDistanceM,
    suggestedSeed: [...cell.seed],
    branchId: cell.branchId
  };
};
