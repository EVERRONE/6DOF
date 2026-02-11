// STL Loader utility for loading robot meshes
import * as THREE from 'three';
import { STLLoader as ThreeSTLLoader } from 'three/examples/jsm/loaders/STLLoader';

/**
 * Load STL file and return geometry
 */
export async function loadSTL(path: string): Promise<THREE.BufferGeometry> {
  return new Promise((resolve, reject) => {
    const loader = new ThreeSTLLoader();

    loader.load(
      path,
      (geometry) => {
        // Center the geometry
        geometry.computeBoundingBox();
        resolve(geometry);
      },
      undefined,
      (error) => {
        reject(new Error(`Failed to load STL: ${path} - ${error}`));
      }
    );
  });
}

/**
 * Create mesh from STL geometry with default material
 */
export function createMeshFromGeometry(
  geometry: THREE.BufferGeometry,
  color: number = 0x808080
): THREE.Mesh {
  const material = new THREE.MeshStandardMaterial({
    color: color,
    metalness: 0.3,
    roughness: 0.6,
    flatShading: false
  });

  const mesh = new THREE.Mesh(geometry, material);
  mesh.castShadow = true;
  mesh.receiveShadow = true;

  return mesh;
}

/**
 * Load multiple STL files
 */
export async function loadMultipleSTLs(
  paths: string[]
): Promise<THREE.BufferGeometry[]> {
  return Promise.all(paths.map(path => loadSTL(path)));
}

/**
 * Robot mesh file mapping
 * Maps link names to STL file paths
 */
export const ROBOT_MESH_FILES: { [linkName: string]: string } = {
  'linkB': '/stl_meshes/Baseplate.001.stl',
  'link0': '/stl_meshes/BaseWall_2_v2.stl',
  'link1': '/stl_meshes/Arm1v2.stl',
  'link2': '/stl_meshes/Arm2Mountv2.stl',
  'link3': '/stl_meshes/Rotation_arm_V3.stl',
  'link4': '/stl_meshes/J6_housing.stl',
  'link5': '/stl_meshes/Cylinder.stl'
};

/**
 * Load all robot meshes
 */
export async function loadAllRobotMeshes(): Promise<Map<string, THREE.BufferGeometry>> {
  const meshMap = new Map<string, THREE.BufferGeometry>();

  for (const [linkName, path] of Object.entries(ROBOT_MESH_FILES)) {
    try {
      const geometry = await loadSTL(path);
      meshMap.set(linkName, geometry);
      console.log(`Loaded mesh for ${linkName}`);
    } catch (error) {
      console.error(`Failed to load mesh for ${linkName}:`, error);
    }
  }

  return meshMap;
}

/**
 * Get default mesh color for each link
 */
export function getLinkColor(linkName: string): number {
  const colorMap: { [key: string]: number } = {
    'linkB': 0x404040,    // Dark gray (base)
    'link0': 0x2196F3,    // Blue
    'link1': 0xFF9800,    // Orange
    'link2': 0x4CAF50,    // Green
    'link3': 0x9C27B0,    // Purple
    'link4': 0xF44336,    // Red
    'link5': 0xFFEB3B     // Yellow (end-effector)
  };

  return colorMap[linkName] || 0x808080;
}
