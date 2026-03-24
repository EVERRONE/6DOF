# Design: Collision-Based Motion Freedom

**Date:** 2026-03-24
**Status:** Approved
**Goal:** Increase robot arm movement freedom by replacing conservative joint limits with collision-based constraints.

---

## Problem

The current firmware joint limits are conservative and restrict the IK solver's search space. J2 (shoulder) and J3 (elbow) are the primary bottlenecks:
- J1: only 70° total travel (-40° to +30°) — cable constraint, but range too narrow
- J2: only 60° total (-60° to 0°) — shoulder, severely limits reach
- J3: only 70° total (0° to +70°) — elbow, severely limits reach

J4 and J5 already have wide travel (-274°/-280°) and are not the limiting factor.

The robot has no 360° free rotation due to cables, but that is only a J1 concern. J2–J5 limits should be widened so the IK solver can explore the full collision-free space.

---

## Requirements

1. **J1 limited to ±60°** — cable routing prevents full rotation; this is the only angular constraint that must remain.
2. **J2–J5 firmware limits widened** — firmware becomes a generous last-resort stop, not the primary motion constraint.
3. **Collision detection always on** — `CollisionModel.ts` (capsule-based self-collision) becomes the primary constraint for J2–J5 motion freedom.
4. **Endstops unchanged** — hardware stops on J2–J5 always remain active regardless of software state.
5. **Workspace visualization updated** — 3D viewer reflects the actual J1 ±60° sector.

---

## Architecture

### What changes

| Layer | File | Change |
|-------|------|--------|
| Firmware | `firmware/config.h` | Widen `JOINT_MIN`/`JOINT_MAX` for J1–J5 |
| Web app — state | `robot-arm-control/src/store/robotStore.ts` | Collision flag default to `true` (two locations) |
| Web app — viewer | `robot-arm-control/src/components/RobotViewer3D.tsx` | Sector workspace visualization |

### What does NOT change

- `HOME_LOGICAL_DEG` — must remain at 0° for all joints; defines the logical zero assigned at the endstop
- `URDF_OFFSET_DEG` — must remain unchanged (J2: 60°, J4: 274°, J5: 280°); these compensate for URDF neutral vs logical zero and are derived from the homing position, not the travel limits
- `ConstraintAdapterService.ts` — reads firmware limits at runtime, automatically picks up new values
- `angleMapping.ts` — no change; URDF offsets and directions unchanged
- Homing logic (`HomingController.cpp`)
- Endstop hardware behavior
- Serial protocol
- IK algorithm (`HybridIKSolver.ts`, `InverseKinematics.ts`)
- `CollisionModel.ts` capsule geometry

---

## Firmware Changes (`config.h`)

### Endstop homing convention

Before reading the table: J2/J4/J5 home toward their **maximum** stop (`HOME_TOWARD_MIN = false`). Logical zero is assigned at the endstop (high mechanical side). Valid travel is **negative** (away from endstop). J3 homes toward its **minimum** stop (`HOME_TOWARD_MIN = true`); valid travel is **positive**.

Widening must extend in the valid travel direction only. Setting MAX > 0 for J2/J4/J5 or MIN < 0 for J3 would attempt to drive past the endstop and is not physically possible.

### Updated limits

| Joint | Current MIN | Current MAX | New MIN | New MAX | Widening direction |
|-------|-------------|-------------|---------|---------|-------------------|
| J1 | -40° | +30° | **-60°** | **+60°** | Both sides (no endstop) |
| J2 | -60° | 0° | **-120°** | **0°** | Negative only (endstop at 0°) |
| J3 | 0° | +70° | **0°** | **+120°** | Positive only (endstop at 0°) |
| J4 | -274° | 0° | **-355°** | **0°** | Negative only (endstop at 0°) |
| J5 | -280° | 0° | **-355°** | **0°** | Negative only (endstop at 0°) |
| J6 | -360° | +360° | unchanged | unchanged | Already full rotation |

**Important:** J2–J5 values are starting points. The user must verify them against the physical mechanical range before flashing. Values must not exceed the point where the arm would crash in the endstop-free direction.

---

## Web App: Collision Default On

**File:** `robot-arm-control/src/store/robotStore.ts`

Two locations must be updated together:

**1. Line ~206 — feature flag initializer** (opt-out instead of opt-in):
```ts
// Before:
const FEATURE_IK_COLLISION_CHECK_V1 = process.env.REACT_APP_IK_COLLISION_CHECK_V1 === '1';

// After:
const FEATURE_IK_COLLISION_CHECK_V1 = process.env.REACT_APP_IK_COLLISION_CHECK_V1 !== '0';
```

**2. Line ~1278 — `setCollisionCheckEnabled` guard** (same change):
```ts
// Before:
setCollisionCheckEnabled: (enabled) => {
  if (!FEATURE_IK_COLLISION_CHECK_V1) {
    set({ collisionCheckEnabled: false });
    return;
  }
  ...

// After: guard remains but now FEATURE_IK_COLLISION_CHECK_V1 is true by default,
// so the guard will not force-disable collision. No logic change needed beyond the
// flag initializer fix above — both files read the same const.
```

Both locations reference the same `FEATURE_IK_COLLISION_CHECK_V1` const, so fixing the initializer alone is sufficient. No changes to `HybridIKSolver.ts` or `InverseKinematics.ts`.

**Capsule radii** in `CollisionModel.ts` remain: 30 mm (base/shoulder links), 22 mm (remaining links). Note: with wider joint limits, the arm can now reach poses that the capsule geometry may not have been tested at. Supervised testing at the motion extremes is required to verify the capsule model provides adequate coverage.

---

## Web App: Workspace Visualization

**File:** `robot-arm-control/src/components/RobotViewer3D.tsx`

The `WorkspaceBoundary` component currently renders a rectangular box `[-0.3, 0.3] × [-0.3, 0.3] × [0, 0.4]` which misleadingly suggests 360° freedom.

Replace with a **sector mesh** representing the J1 ±60° front zone:
- Angular sweep: -60° to +60° around the Z axis
- Radial range: 0 to ~400 mm *(estimate — verify against FK at new joint limits)*
- Height range: 0 to ~400 mm
- Color: green, low opacity (consistent with current style)

The 400 mm radial figure is an estimate based on the current workspace box size. After implementation, run FK at the extreme joint configurations to verify actual reach and adjust if needed.

This is purely informational — the sector does not enforce any constraint. Constraint enforcement happens in the IK solver.

---

## Data Flow (unchanged)

```
firmware CFG message
  → SerialManager.ts parses it
  → robotStore stores as firmwareConfig
  → ConstraintAdapterService.fromFirmwareConfig() maps to URDF limits (picks up new wider limits automatically)
  → IK solver uses wider URDF limits + collision check (now always on)
  → valid joint angles sent to firmware via TQ
  → firmware enforces its own (now wider) limits as last resort
  → endstops always override
```

---

## Risks and Mitigations

| Risk | Mitigation |
|------|------------|
| Arm mechanically crashes in J2/J3 negative direction (no endstop there) | User verifies new firmware limits against physical range before flashing |
| Capsule model may not cover new extreme poses accurately | Conservative 22–30 mm radii provide margin; supervised testing at motion extremes required |
| Web app collision check is planning-only, not real-time | Firmware limits + endstops remain as hardware safety net |
| J1 driven past ±60° | Firmware enforces J1 ±60° as hard limit |
| HOME_LOGICAL_DEG or URDF_OFFSET_DEG accidentally changed | Explicitly out of scope — these are frame anchors, not travel limits |

---

## Out of Scope

- Real-time collision avoidance during trajectory execution
- Ground plane / table collision detection
- Updating capsule geometry to match actual STL meshes
- Waypoint execution path improvements
- Changes to `HOME_LOGICAL_DEG` or `URDF_OFFSET_DEG`
