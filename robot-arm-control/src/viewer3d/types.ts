// 3D Viewer type definitions
import * as THREE from 'three';

/**
 * Robot link mesh information
 */
export interface LinkMeshInfo {
  linkName: string;
  stlFileName: string;
  position: THREE.Vector3;
  rotation: THREE.Euler;
  scale: THREE.Vector3;
}

/**
 * Robot joint hierarchy node
 */
export interface JointNode {
  jointIndex: number;      // 0-5 for J1-J6
  jointName: string;        // e.g., "link0_joint"
  linkName: string;         // e.g., "link0"
  mesh: THREE.Mesh | null;
  group: THREE.Group;       // Joint's local coordinate frame
  children: JointNode[];
}

/**
 * 3D Robot model
 */
export interface Robot3DModel {
  root: THREE.Group;        // Root of entire robot
  joints: JointNode[];      // Ordered list of joints [J1-J6]
  endEffectorMarker: THREE.Object3D | null;
}

/**
 * Workspace visualization options
 */
export interface WorkspaceOptions {
  visible: boolean;
  xRange: [number, number];  // meters
  yRange: [number, number];
  zRange: [number, number];
  color: string;
  opacity: number;
}

/**
 * Camera preset positions
 */
export enum CameraPreset {
  FRONT = 'front',
  SIDE = 'side',
  TOP = 'top',
  ISOMETRIC = 'isometric'
}
