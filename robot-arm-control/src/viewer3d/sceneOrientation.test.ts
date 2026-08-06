// The scene is Z-up, because everything placed in it is in robot coordinates:
// the arm is built from the URDF, and the target marker, trajectory line and
// waypoints are all fed positions straight out of the kinematics.
//
// Three.js defaults to Y-up, and its helpers are built for that. Mixing the two
// drew the arm lying on its side against a floor that was standing on edge.

import * as THREE from 'three';
import { createGrid, createGroundPlane, createWorkspaceBoundary } from './RobotModel3D';
import { workspaceBounds } from '../kinematics/InverseKinematics';
import { ForwardKinematics } from '../kinematics/ForwardKinematics';
import { HOME_POSE_DEG } from '../kinematics/robotModel';

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

describe('workspace boundary', () => {
  // It used to be a fixed 600 x 600 x 400 mm box centred on the origin, which
  // described no robot: this arm reaches from -316 to -23 mm in X, so most of
  // its work area fell outside the box while the box enclosed a large volume
  // the arm cannot enter. A boundary that is not the arm's boundary is worse
  // than none - it invites targets that cannot be reached.

  it('is built from the arm rather than from a constant', () => {
    const b = workspaceBounds(9);
    const box = createWorkspaceBoundary(
      [b.min.x, b.max.x],
      [b.min.y, b.max.y],
      [b.min.z, b.max.z]
    );

    const size = new THREE.Box3().setFromObject(box).getSize(new THREE.Vector3());
    expect(size.x).toBeCloseTo(b.max.x - b.min.x, 6);
    expect(size.y).toBeCloseTo(b.max.y - b.min.y, 6);
    expect(size.z).toBeCloseTo(b.max.z - b.min.z, 6);
  });

  it('is not the constant it used to be', () => {
    // The regression this guards: a fixed 600 x 600 x 400 mm box centred on the
    // origin. It has to differ from that in some dimension, whatever the joint
    // limits currently are.
    const b = workspaceBounds(9);
    const wasHardcoded =
      Math.abs(b.min.x + 0.3) < 1e-9 && Math.abs(b.max.x - 0.3) < 1e-9 &&
      Math.abs(b.min.y + 0.3) < 1e-9 && Math.abs(b.max.y - 0.3) < 1e-9 &&
      Math.abs(b.min.z) < 1e-9 && Math.abs(b.max.z - 0.4) < 1e-9;
    expect(wasHardcoded).toBe(false);

    // The base sits at z = 0.08 and the arm reaches up, so the floor of the
    // reachable set cannot be at or below the ground plane.
    expect(b.min.z).toBeGreaterThan(0);
  });

  it('encloses the pose the arm parks in', () => {
    const b = workspaceBounds(9);
    const p = ForwardKinematics.position(HOME_POSE_DEG);
    expect(p.x).toBeGreaterThanOrEqual(b.min.x - 1e-9);
    expect(p.x).toBeLessThanOrEqual(b.max.x + 1e-9);
    expect(p.z).toBeGreaterThanOrEqual(b.min.z - 1e-9);
    expect(p.z).toBeLessThanOrEqual(b.max.z + 1e-9);
  });
});
