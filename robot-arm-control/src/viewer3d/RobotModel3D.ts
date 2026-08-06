// 3D Robot Model Builder.
//
// The joint hierarchy is built from ROBOT_JOINTS, the same chain the kinematics
// core solves against, so what you see is what FK and IK compute.
import * as THREE from 'three';
import { JointNode, Robot3DModel } from './types';
import { loadAllRobotMeshes, createMeshFromGeometry, getLinkColor } from './STLLoader';
import { ROBOT_JOINTS, logicalToUrdfRad } from '../kinematics/robotModel';

/**
 * Build 3D robot model from STL meshes
 */
export class RobotModel3DBuilder {
  private meshes: Map<string, THREE.BufferGeometry>;
  private model: Robot3DModel | null = null;

  constructor() {
    this.meshes = new Map();
  }

  /**
   * Load all meshes
   */
  async loadMeshes(): Promise<void> {
    this.meshes = await loadAllRobotMeshes();
  }

  /**
   * Build robot model hierarchy
   */
  buildModel(): Robot3DModel {
    const root = new THREE.Group();
    root.name = 'RobotRoot';

    // Create base (linkB) - fixed
    const baseGroup = new THREE.Group();
    baseGroup.name = 'linkB';

    const baseGeometry = this.meshes.get('linkB');
    if (baseGeometry) {
      const baseMesh = createMeshFromGeometry(baseGeometry, getLinkColor('linkB'));
      baseMesh.name = 'linkB_mesh';

      // Apply URDF visual transform (URDF rpy = extrinsic XYZ = Three.js 'ZYX')
      baseMesh.rotation.set(Math.PI / 2, 0, 0, 'ZYX');
      baseMesh.position.set(0, 0, 0.08);

      baseGroup.add(baseMesh);
    }

    root.add(baseGroup);

    // Create joints hierarchy
    const joints: JointNode[] = [];
    let parentGroup = baseGroup;

    // Link names in order
    const linkNames = ['link0', 'link1', 'link2', 'link3', 'link4', 'link5'];

    for (let i = 0; i < 6; i++) {
      const jointInfo = ROBOT_JOINTS[i];
      const linkName = linkNames[i];

      // Create joint group (represents joint's coordinate frame)
      const jointGroup = new THREE.Group();
      jointGroup.name = `${jointInfo.name}_frame`;

      // Apply joint origin transform from URDF
      const origin = jointInfo.origin;
      jointGroup.position.set(origin.xyz.x, origin.xyz.y, origin.xyz.z);
      // URDF rpy = extrinsic XYZ = Three.js Euler order 'ZYX'
      jointGroup.rotation.set(origin.rpy.roll, origin.rpy.pitch, origin.rpy.yaw, 'ZYX');

      // Create link mesh
      let mesh: THREE.Mesh | null = null;
      const geometry = this.meshes.get(linkName);

      if (geometry) {
        mesh = createMeshFromGeometry(geometry, getLinkColor(linkName));
        mesh.name = `${linkName}_mesh`;

        // Apply visual offset from URDF (link-specific transforms)
        this.applyVisualTransform(mesh, linkName);

        jointGroup.add(mesh);
      }

      // Attach joint to parent
      parentGroup.add(jointGroup);

      // Create joint node
      const jointNode: JointNode = {
        jointIndex: i,
        jointName: jointInfo.name,
        linkName: linkName,
        mesh: mesh,
        group: jointGroup,
        children: []
      };

      joints.push(jointNode);

      // Next joint attaches to this joint's group
      parentGroup = jointGroup;
    }

    // Create end-effector marker
    const endEffectorMarker = this.createEndEffectorMarker();
    joints[5].group.add(endEffectorMarker);

    this.model = {
      root,
      joints,
      endEffectorMarker
    };

    return this.model;
  }

  /**
   * Apply visual transform from URDF for each link
   */
  private applyVisualTransform(mesh: THREE.Mesh, linkName: string): void {
    // These transforms come from the <visual> tags in URDF
    // They position the mesh relative to the link's coordinate frame

    // URDF rpy = extrinsic XYZ = Three.js Euler order 'ZYX'
    switch (linkName) {
      case 'link0': // BaseWall_2_v2
        mesh.rotation.set(Math.PI / 2, 0, Math.PI / 2, 'ZYX');
        mesh.position.set(-0.0375, 0.02, 0.05595);
        break;

      case 'link1': // Arm1v2
        mesh.rotation.set(0, 0, Math.PI, 'ZYX');
        mesh.position.set(0, 0, 0.016);
        break;

      case 'link2': // Arm2Mountv2
        mesh.rotation.set(-1.83292, -Math.PI / 2, 1.83292, 'ZYX');
        mesh.position.set(0, 0, 0.012);
        break;

      case 'link3': // Rotation arm V3
        mesh.rotation.set(0, 0, Math.PI / 2, 'ZYX');
        mesh.position.set(0, 0, 0);
        break;

      case 'link4': // J6 housing
        mesh.rotation.set(-1.30867, -Math.PI / 2, -1.83292, 'ZYX');
        mesh.position.set(-0.02, 0, 0.00994);
        break;

      case 'link5': // Cylinder (end-effector)
        mesh.rotation.set(Math.PI, 0, Math.PI, 'ZYX');
        mesh.position.set(0, 0, -0.00677);
        mesh.scale.set(0.002, 0.002, 0.0025);
        break;
    }
  }

  /**
   * Create end-effector coordinate frame marker
   */
  private createEndEffectorMarker(): THREE.Object3D {
    const group = new THREE.Group();
    group.name = 'EndEffectorMarker';

    const axesHelper = new THREE.AxesHelper(0.05); // 50mm axes
    group.add(axesHelper);

    return group;
  }

  /**
   * Update robot pose from joint angles
   */
  updatePose(jointAngles: number[]): void {
    if (!this.model) return;

    // These arrive as firmware angles - what the arm reports and the panels
    // show - and the chain below is the URDF's, so they need converting. Drawing
    // them straight, as this used to, is what put the arm flat on the floor in
    // the viewer while the real one stood upright. Same conversion as
    // ForwardKinematics uses, so the viewer and the solver cannot disagree.
    const anglesRad = jointAngles.map(a => (a * Math.PI) / 180);
    const anglesUrdf = logicalToUrdfRad(anglesRad);

    // For each joint, apply rotation
    for (let i = 0; i < 6; i++) {
      const joint = this.model.joints[i];
      const angle = anglesUrdf[i];

      // All joints rotate around Z-axis (from URDF axis="0 0 1")
      const origin = ROBOT_JOINTS[i].origin;

      // URDF rpy = extrinsic XYZ = Three.js Euler order 'ZYX'
      const urdfRotation = new THREE.Euler(
        origin.rpy.roll,
        origin.rpy.pitch,
        origin.rpy.yaw,
        'ZYX'
      );

      const urdfQuat = new THREE.Quaternion().setFromEuler(urdfRotation);

      // Joint angle rotation around local Z-axis (after origin transform)
      const jointQuat = new THREE.Quaternion().setFromAxisAngle(
        new THREE.Vector3(0, 0, 1),
        angle
      );

      // Combine: R_origin * R_z(θ) — origin rotation first, then joint in local frame
      const finalQuat = new THREE.Quaternion().multiplyQuaternions(urdfQuat, jointQuat);

      joint.group.setRotationFromQuaternion(finalQuat);
    }
  }

  /**
   * Get current end-effector position (from 3D model)
   */
  getEndEffectorPosition(): THREE.Vector3 {
    if (!this.model || !this.model.endEffectorMarker) {
      return new THREE.Vector3(0, 0, 0);
    }

    const worldPos = new THREE.Vector3();
    this.model.endEffectorMarker.getWorldPosition(worldPos);
    return worldPos;
  }

  /**
   * Get model
   */
  getModel(): Robot3DModel | null {
    return this.model;
  }
}

/**
 * Create workspace boundary box
 */
export function createWorkspaceBoundary(
  xRange: [number, number],
  yRange: [number, number],
  zRange: [number, number],
  color: number = 0x00ff00,
  opacity: number = 0.1
): THREE.Object3D {
  const group = new THREE.Group();
  group.name = 'WorkspaceBoundary';

  const width = xRange[1] - xRange[0];
  const height = zRange[1] - zRange[0];
  const depth = yRange[1] - yRange[0];

  const centerX = (xRange[0] + xRange[1]) / 2;
  const centerY = (yRange[0] + yRange[1]) / 2;
  const centerZ = (zRange[0] + zRange[1]) / 2;

  // Create wireframe box
  const geometry = new THREE.BoxGeometry(width, depth, height);
  const edges = new THREE.EdgesGeometry(geometry);
  const lineMaterial = new THREE.LineBasicMaterial({ color: color, linewidth: 2 });
  const wireframe = new THREE.LineSegments(edges, lineMaterial);

  wireframe.position.set(centerX, centerY, centerZ);
  group.add(wireframe);

  // Semi-transparent box
  const boxMaterial = new THREE.MeshBasicMaterial({
    color: color,
    transparent: true,
    opacity: opacity,
    side: THREE.DoubleSide
  });

  const box = new THREE.Mesh(geometry, boxMaterial);
  box.position.set(centerX, centerY, centerZ);
  group.add(box);

  return group;
}

/**
 * Create target position marker
 */
export function createTargetMarker(color: number = 0xff0000): THREE.Object3D {
  const group = new THREE.Group();
  group.name = 'TargetMarker';

  // Sphere
  const geometry = new THREE.SphereGeometry(0.015, 16, 16); // 15mm radius
  const material = new THREE.MeshBasicMaterial({
    color: color,
    transparent: true,
    opacity: 0.8
  });

  const sphere = new THREE.Mesh(geometry, material);
  group.add(sphere);

  // Coordinate frame
  const axes = new THREE.AxesHelper(0.03); // 30mm axes
  group.add(axes);

  group.visible = false; // Hidden by default

  return group;
}

/**
 * Create ground plane
 */
export function createGroundPlane(size: number = 1.0): THREE.Mesh {
  const geometry = new THREE.PlaneGeometry(size, size);
  const material = new THREE.MeshStandardMaterial({
    color: 0x333333,
    roughness: 0.8,
    metalness: 0.2
  });

  const plane = new THREE.Mesh(geometry, material);
  // The scene is Z-up, matching the URDF the arm is built from. PlaneGeometry is
  // already in the XY plane, so it needs no rotation here - it used to be turned
  // -90 degrees about X for a Y-up world, which left the floor standing on edge
  // relative to a robot whose own up axis is Z.
  plane.position.z = -0.01; // Slightly below the base
  plane.receiveShadow = true;

  return plane;
}

/**
 * Create grid helper
 */
export function createGrid(size: number = 1.0, divisions: number = 20): THREE.GridHelper {
  const grid = new THREE.GridHelper(size, divisions, 0x444444, 0x222222);
  // GridHelper is built in the XZ plane for a Y-up world. This scene is Z-up,
  // like the URDF, so bring it into XY - otherwise the floor is a wall and the
  // arm appears to lie on its side against it.
  grid.rotation.x = Math.PI / 2;
  grid.position.set(0, 0, 0);
  return grid;
}
