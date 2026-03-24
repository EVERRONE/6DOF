# Collision-Based Motion Freedom Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** Replace conservative firmware joint limits with collision-based constraints to give the robot arm significantly more movement freedom.

**Architecture:** Widen firmware JOINT_MIN/JOINT_MAX in config.h so the Teensy no longer blocks valid motion, enable the existing capsule-based CollisionModel in the IK solver by default (opt-out instead of opt-in), and replace the misleading rectangular workspace box in the 3D viewer with a J1 ±60° sector shape.

**Tech Stack:** C++ (Teensy firmware), TypeScript/React (web app), Three.js (3D viewer)

**Spec:** `docs/superpowers/specs/2026-03-24-collision-based-motion-freedom-design.md`

---

## File Map

| File | Change |
|------|--------|
| `firmware/config.h` | Widen `JOINT_MIN`/`JOINT_MAX` for J1–J5 |
| `robot-arm-control/src/store/robotStore.ts` | Line 206: flag opt-out only (guard unchanged) |
| `robot-arm-control/src/viewer3d/RobotModel3D.ts` | Add `createWorkspaceSector()` function |
| `robot-arm-control/src/components/RobotViewer3D.tsx` | Replace `WorkspaceBoundary` with sector component |

**Unchanged (do not touch):**
- `firmware/config.h` fields: `HOME_LOGICAL_DEG`, `URDF_OFFSET_DEG`, `HOME_TOWARD_MIN`, `HAS_ENDSTOP`
- `CollisionModel.ts` — capsule radii stay at 30mm/22mm
- `ConstraintAdapterService.ts` — reads firmware config at runtime, picks up new limits automatically
- `HybridIKSolver.ts`, `InverseKinematics.ts` — collision check already wired in

---

## Task 1: Widen firmware joint limits

**Files:**
- Modify: `firmware/config.h:36-37`

### Endstop homing convention — read before changing limits

- **J2, J4, J5**: endstop at logical 0°, valid travel is **negative** → extend MIN only, keep MAX at 0
- **J3**: endstop at logical 0°, valid travel is **positive** → extend MAX only, keep MIN at 0
- **J1, J6**: no endstops → extend both directions freely

Setting MAX > 0 for J2/J4/J5 or MIN < 0 for J3 would command motion past the endstop switch.

### Why URDF_OFFSET_DEG does NOT change

`URDF_OFFSET_DEG` compensates for where the URDF neutral pose is relative to logical zero (the endstop). Widening travel limits does not move logical zero — the endstop still homes to 0° and `URDF_OFFSET_DEG` still correctly maps logical 0° to the URDF neutral pose. The firmware will broadcast the new wider limits in the `CFG` message; `ConstraintAdapterService.fromFirmwareConfig()` will automatically compute the new URDF range by applying the unchanged offsets and directions to the new wider logical limits. No manual change to `URDF_OFFSET_DEG` or `angleMapping.ts` is needed.

Example for J2 (direction=1, offset=60°):
- Old: logical [-60°, 0°] → URDF [0°, 60°]
- New: logical [-120°, 0°] → URDF [-60°, 60°]

The IK solver will now search the expanded URDF range. This is correct behavior.

### Steps

- [ ] **Step 1: Verify physical clearance for J1 ±60°**

J1 has no endstop. Current range is -40°/+30° (70° total). New range is ±60° (120° total). Before changing firmware, manually push J1 (motors disabled) to approximately ±60° to confirm no mechanical binding or cable damage. If the arm cannot safely reach ±60° in one direction, reduce the limit to the safe physical maximum.

- [ ] **Step 2: Update JOINT_MIN and JOINT_MAX in config.h**

Replace lines 36–37:
```cpp
const float JOINT_MIN[6] = {-60, -120, 0, -355, -355, -360};
const float JOINT_MAX[6] = {60, 0, 120, 0, 0, 360};
```

- [ ] **Step 3: Confirm the following lines in config.h are NOT modified**

```cpp
const float HOME_LOGICAL_DEG[6] = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };
const float URDF_OFFSET_DEG[6] = { 0.0, 60.0, 0.0, 274.0, 280.0, 0.0 };
```

- [ ] **Step 4: Flash firmware and verify CFG message**

Flash `firmware/firmware.ino` to Teensy 4.1. Connect via Chrome/Edge. In the web app, query `CFG?` and confirm the response contains the new limits (e.g., J2 min = -120, J3 max = 120).

- [ ] **Step 5: Verify ConstraintAdapterService picks up new URDF range**

In the browser console or by adding a temporary `console.log` in `ConstraintAdapterService.fromFirmwareConfig()`, confirm J2 URDF min is now approximately -60° (was 0°). Remove any temporary logging.

- [ ] **Step 6: Jog test at wider range**

After `H ALL`:
1. Jog J2 to -90° — arm should move further than before
2. Jog J3 to +100° — arm should accept the target
3. Run `H ALL` again — confirm homing still sets logical zero correctly

- [ ] **Step 7: Commit**

```bash
git add firmware/config.h
git commit -m "feat(firmware): widen joint limits J1 ±60°, J2/J3 ±120°, J4/J5 ±355°"
```

---

## Task 2: Enable collision detection by default

**Files:**
- Modify: `robot-arm-control/src/store/robotStore.ts:206` (flag initializer only)

### Design decision: guard stays, only flag initializer changes

The guard at line ~1278:
```ts
if (!FEATURE_IK_COLLISION_CHECK_V1) {
  set({ collisionCheckEnabled: false });
  return;
}
```
This guard enforces that when the operator explicitly opts out (`REACT_APP_IK_COLLISION_CHECK_V1=0`), the UI toggle cannot re-enable collision checking at runtime. This is the correct behavior for a hard opt-out: the env var is an operator decision, not a default. The guard stays unchanged.

With the new default-on flag (`!== '0'`):
- Default (env absent): `FEATURE_IK_COLLISION_CHECK_V1 = true` → guard never fires → initial state `true` → UI toggle works freely
- Opt-out (`=0`): `FEATURE_IK_COLLISION_CHECK_V1 = false` → guard forces `false` on any toggle call → operator cannot accidentally re-enable via UI

This is the correct semantics. Only line 206 changes.

### Steps

- [ ] **Step 1: Change line 206 to opt-out**

```ts
// Before:
const FEATURE_IK_COLLISION_CHECK_V1 = process.env.REACT_APP_IK_COLLISION_CHECK_V1 === '1';

// After:
const FEATURE_IK_COLLISION_CHECK_V1 = process.env.REACT_APP_IK_COLLISION_CHECK_V1 !== '0';
```

- [ ] **Step 2: Verify line 751 still reads**

```ts
collisionCheckEnabled: FEATURE_IK_COLLISION_CHECK_V1,
```
Initial store state will now be `true` by default — correct.

- [ ] **Step 3: Write regression test**

Create `robot-arm-control/src/store/__tests__/robotStore.collisionFlag.test.ts`:

```ts
describe('FEATURE_IK_COLLISION_CHECK_V1 flag evaluation', () => {
  const evaluate = (envVal: string | undefined): boolean =>
    envVal !== '0';

  it('is true when env var is absent', () => {
    expect(evaluate(undefined)).toBe(true);
  });

  it('is true when env var is set to 1', () => {
    expect(evaluate('1')).toBe(true);
  });

  it('is false when env var is set to 0 (opt-out)', () => {
    expect(evaluate('0')).toBe(false);
  });
});
```

Note: the test evaluates the flag expression in isolation (`envVal !== '0'`) to avoid CRA module initialization ordering issues. This tests the exact boolean logic used in `robotStore.ts:206`.

- [ ] **Step 4: Run tests**

```bash
cd robot-arm-control
npm test -- --watchAll=false --runInBand
```
Expected: all previous tests pass, plus the 3 new tests above pass.

- [ ] **Step 5: Commit**

```bash
git add robot-arm-control/src/store/robotStore.ts \
        robot-arm-control/src/store/__tests__/robotStore.collisionFlag.test.ts
git commit -m "feat(ik): enable collision detection by default (opt-out instead of opt-in)"
```

---

## Task 3: Add sector workspace visualization

**Files:**
- Modify: `robot-arm-control/src/viewer3d/RobotModel3D.ts` (add `createWorkspaceSector` after line 276)
- Modify: `robot-arm-control/src/components/RobotViewer3D.tsx` (update import + replace `WorkspaceBoundary`)

### Coordinate frame

Three.js coordinate system in this viewer: **Z is up** (confirmed by `axesHelper` at origin and `[0,0,1]` camera up vector in `RobotViewer3D.tsx`). J1 is the base rotation joint; it rotates around the vertical (Z) axis. A sector showing the J1 ±60° zone sweeps in the **XY plane** (horizontal) and extrudes upward along Z.

`THREE.ExtrudeGeometry` takes a 2D `THREE.Shape` in the XY plane and extrudes along its local +Z. With Z-up and J1 rotating about Z, this produces a vertical pie-slice standing above the floor — correct.

**Acceptance criterion for visual verify:** viewed from above (top-down camera in the 3D viewer), the green sector should appear as a pie slice spanning 120° (±60° from the +X forward axis). Viewed from the side, it should appear as a vertical green wedge.

### Step 3a — Add createWorkspaceSector to RobotModel3D.ts

- [ ] **Step 1: Add the function immediately after the closing brace of `createWorkspaceBoundary` (after line 276)**

```ts
/**
 * Create a sector (pie slice) workspace boundary representing J1 ±halfAngleDeg freedom.
 * J1 rotates about the Z axis (vertical/up in this viewer). The sector sweeps
 * in the XY plane and extrudes along +Z (upward).
 */
export function createWorkspaceSector(
  halfAngleDeg: number = 60,
  radiusM: number = 0.4,
  zMin: number = 0,
  zMax: number = 0.4,
  color: number = 0x00ff00,
  opacity: number = 0.05
): THREE.Object3D {
  const group = new THREE.Group();
  group.name = 'WorkspaceSector';

  const segments = 32;
  const halfAngleRad = (halfAngleDeg * Math.PI) / 180;
  const height = zMax - zMin;

  // Build a 2D sector shape in the XY plane.
  // Angle 0 = +X axis (forward direction). Sweep from -halfAngle to +halfAngle.
  const shape = new THREE.Shape();
  shape.moveTo(0, 0);
  for (let i = 0; i <= segments; i++) {
    const angle = -halfAngleRad + (2 * halfAngleRad * i) / segments;
    shape.lineTo(Math.cos(angle) * radiusM, Math.sin(angle) * radiusM);
  }
  shape.lineTo(0, 0);

  const extrudeSettings: THREE.ExtrudeGeometryOptions = {
    depth: height,
    bevelEnabled: false
  };

  const geometry = new THREE.ExtrudeGeometry(shape, extrudeSettings);

  // Semi-transparent fill
  const fillMaterial = new THREE.MeshBasicMaterial({
    color,
    transparent: true,
    opacity,
    side: THREE.DoubleSide
  });
  const fillMesh = new THREE.Mesh(geometry, fillMaterial);
  fillMesh.position.z = zMin;
  group.add(fillMesh);

  // Wireframe outline
  const edges = new THREE.EdgesGeometry(geometry);
  const edgeMaterial = new THREE.LineBasicMaterial({ color, linewidth: 1 });
  const wireframe = new THREE.LineSegments(edges, edgeMaterial);
  wireframe.position.z = zMin;
  group.add(wireframe);

  return group;
}
```

### Step 3b — Update RobotViewer3D.tsx

- [ ] **Step 2: Update the import**

```ts
// Before:
import {
  RobotModel3DBuilder,
  createWorkspaceBoundary,
  createTargetMarker,
  createGroundPlane,
  createGrid
} from '../viewer3d/RobotModel3D';

// After:
import {
  RobotModel3DBuilder,
  createWorkspaceSector,
  createTargetMarker,
  createGroundPlane,
  createGrid
} from '../viewer3d/RobotModel3D';
```

- [ ] **Step 3: Replace the WorkspaceBoundary component (lines ~87–116)**

```tsx
/**
 * Workspace Sector Visualization — J1 ±60° zone
 */
const WorkspaceBoundary: React.FC<{ visible: boolean }> = ({ visible }) => {
  const boundaryRef = useRef<THREE.Object3D>(null);

  useEffect(() => {
    const sector = createWorkspaceSector(
      60,       // J1 ±60° half-angle
      0.4,      // 400mm radius (estimate — verify against FK at new joint limits)
      0,        // zMin
      0.4,      // zMax = 400mm
      0x00ff00, // Green
      0.05      // Low opacity
    );

    boundaryRef.current = sector;

    return () => {
      sector.traverse((obj) => {
        if (obj instanceof THREE.Mesh) {
          obj.geometry.dispose();
          if (obj.material instanceof THREE.Material) {
            obj.material.dispose();
          }
        }
      });
    };
  }, []);

  if (!boundaryRef.current) return null;

  return <primitive object={boundaryRef.current} visible={visible} />;
};
```

- [ ] **Step 4: Visually verify in the browser**

```bash
cd robot-arm-control
npm start
```
Open 3D viewer in Chrome/Edge. Toggle "Show Workspace".

**Acceptance criteria:**
- Viewed from above: green pie slice spanning 120° centered on the +X forward axis
- Viewed from the side: vertical green wedge from floor (z=0) to ~400mm height
- NOT a rectangular box

If the sector appears as a flat disc or is oriented wrong, verify the Z-up coordinate frame and check that `fillMesh.position.z = zMin` correctly places the base at z=0.

- [ ] **Step 5: Run tests**

```bash
npm test -- --watchAll=false --runInBand
```
Expected: all tests pass.

- [ ] **Step 6: Commit**

```bash
git add robot-arm-control/src/viewer3d/RobotModel3D.ts \
        robot-arm-control/src/components/RobotViewer3D.tsx
git commit -m "feat(viewer): replace workspace box with J1 ±60° sector visualization"
```

---

## Task 4: Final verification

- [ ] **Step 1: Run full test suite**

```bash
cd robot-arm-control
npm test -- --watchAll=false --runInBand
```
Expected: 21+ suites, all pass.

- [ ] **Step 2: Run production build**

```bash
npm run build
```
Expected: build succeeds. The `@mediapipe/tasks-vision` source map warning is pre-existing and acceptable.

- [ ] **Step 3: Hardware test checklist (with Teensy connected)**

Run each item in order. Stop and investigate if any item fails.

1. Flash updated firmware (`firmware/firmware.ino` in Arduino IDE with Teensyduino)
2. Connect via Chrome/Edge → run `CFG?` → confirm new limits in response (J2 min=-120, J3 max=120)
3. Home the arm: `H ALL` → confirm `HOMED` / `HOMEPOSE_REACHED` for J2–J5
4. Jog J1 to -60°, then +60° at low speed (5°/s) → confirm no cable damage or binding
5. Jog J2 to -90° (past old -60° limit) → confirm arm moves smoothly without firmware rejection
6. Jog J3 to +100° (past old +70° limit) → confirm arm moves smoothly
7. Run `H ALL` again → confirm homing sets logical zero correctly after wide-range jog
8. Send a Cartesian target that previously failed due to joint limits → confirm IK now finds a solution
9. Attempt a Cartesian pose where the arm would self-collide → confirm IK rejects it with a collision error in the web app UI
10. Press emergency stop during motion in the wider range → confirm immediate halt

- [ ] **Step 4: Commit any last fixes**

```bash
git add -p
git commit -m "fix: post-integration corrections from hardware test"
```
