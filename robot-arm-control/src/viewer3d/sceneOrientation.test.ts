// The scene is Z-up, because everything placed in it is in robot coordinates:
// the arm is built from the URDF, and the target marker, trajectory line and
// waypoints are all fed positions straight out of the kinematics.
//
// Three.js defaults to Y-up, and its helpers are built for that. Mixing the two
// drew the arm lying on its side against a floor that was standing on edge.

import * as THREE from 'three';
import { createGrid, createGroundPlane } from './RobotModel3D';

/** World-space direction a plane's local +Z faces after its transform. */
function normalOf(object: THREE.Object3D): THREE.Vector3 {
  object.updateMatrixWorld(true);
  return new THREE.Vector3(0, 0, 1)
    .applyQuaternion(object.getWorldQuaternion(new THREE.Quaternion()))
    .normalize();
}

describe('3D scene orientation', () => {
  it('lays the ground plane flat, facing up', () => {
    const plane = createGroundPlane(2.0);
    const n = normalOf(plane);

    // Up is +Z. A Y-up ground plane would come out facing +Y.
    expect(Math.abs(n.z)).toBeCloseTo(1, 9);
    expect(Math.abs(n.y)).toBeCloseTo(0, 9);
  });

  it('puts the ground just below the base, not beside it', () => {
    const plane = createGroundPlane(2.0);
    expect(plane.position.z).toBeLessThan(0);
    expect(plane.position.y).toBe(0);
  });

  it('lays the grid in the horizontal plane', () => {
    const grid = createGrid(2.0, 40);
    grid.updateMatrixWorld(true);

    // GridHelper's lines lie in its local XZ plane, so its local +Y is the
    // plane normal. In a Z-up world that has to end up along Z.
    const normal = new THREE.Vector3(0, 1, 0)
      .applyQuaternion(grid.getWorldQuaternion(new THREE.Quaternion()))
      .normalize();

    expect(Math.abs(normal.z)).toBeCloseTo(1, 9);
    expect(Math.abs(normal.y)).toBeCloseTo(0, 9);
  });

  it('keeps the grid at the height of the base', () => {
    const grid = createGrid(2.0, 40);
    expect(grid.position.z).toBe(0);
  });
});
