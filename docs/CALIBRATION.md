# 6DOF Robot Arm - Calibration Guide

**Goal:** Align firmware, UI, and kinematics so logical (URDF) angles match the physical hardware.

**Target Accuracy (Balanced):** ~+/-2 deg per joint, ~+/-5 mm Cartesian.

---

## Prerequisites

- Robot is assembled, endstops wired for J2-J5.
- Motors enabled and homing works.
- A simple angle gauge or digital inclinometer.
- Calipers or ruler for Cartesian validation.
- Review `docs/HARDWARE_AUDIT.md` for current limits and homing configuration.
- For Cartesian control in UI: robot must be connected, motors enabled, and J2-J5 homed.
- After connect/home, wait for Cartesian frame sync (or press `Current -> Target`) before first Cartesian move.

---

## Key Concepts

**Logical angle** = the angle used in UI and kinematics (URDF).

**Raw angle** = the angle implied by step counts and microstepping.

Mapping per joint:
```
raw_deg = steps / stepsPerDeg
logical_deg = raw_deg * scale + offset
raw_deg = (logical_deg - offset) / scale
```

Calibration adjusts `scale` and `offset` so logical angles match the hardware.
URDF offsets keep FK/IK aligned without changing the URDF model.
Cartesian coordinates in UI/3D/FK use the URDF chain frame (same frame everywhere).

### Operational Home Pose vs Calibration

- **Calibration (`CAL`)** changes logical/raw mapping (`scale`, `offset`) for angle accuracy.
- **Operational Home Pose (`HP`)** defines where the robot moves after `H ALL` (working/start pose).
- **Trajectory Queue (`TQ`)** controls how Cartesian motion is executed (smoothness/path quality), not calibration.
- Calibration does **not** change TCP geometry, URDF link transforms, or IK objective function.
- Endstop zero reference is still preserved for J2-J5.
- Do not use `HP` to fix measurement errors. Use `CAL` for that.

### Workspace and Solver Notes (Not Calibration)

- Workspace reachability decisions are made by IK + trajectory retries, not by `CAL`.
- A target is only marked `invalid_target` after staged retries fail above strict residual gate.
- If `pose_lock` fails but `position_only` works, this is an orientation constraint issue, not a calibration issue.
- If you see queue budget errors, reduce move distance/speed; do not change calibration to fix this.
- Planning latency is controlled by IK/trajectory pipeline settings (stage1/stage2 planning), not by calibration values.
- IK V3 runtime toggles (`Analytic Primary`, `Branch Lock`, `Resolved-Rate`, `Collision Checks`) change solver behavior and smoothness only; they do not change calibration mapping.
- If Cartesian behavior changes after toggling IK runtime options, keep `CAL` values unchanged and validate solver settings first.

### URDF Default Anchoring via HP

In the web app, FK/IK/3D use **effective URDF offsets**:

```
For J2..J5: effectiveUrdfOffsetDeg = -homePose.jointsDeg
For J1/J6: effectiveUrdfOffsetDeg = cfg.joints[j].urdfOffsetDeg

urdfDir = [1, 1, 1, 1, -1, -1]    # J5/J6 inverted
urdfDeg = logicalDeg * urdfDir + effectiveUrdfOffsetDeg
logicalDeg = (urdfDeg - effectiveUrdfOffsetDeg) / urdfDir
```

This makes URDF `0 deg` for `J2..J5` align with the configured operational home pose.

---

## Step-by-Step Calibration

### 1) Home (J2-J5)

- Home the robot (H 2345 or H ALL).
- Verify endstop repeatability by homing twice and checking that the joint returns to the same physical pose.
- After homing, the joint logical angle is set to 0 deg at the endstop and the sliders reset to 0.
- With current config, J2/J4/J5 move away from endstop toward negative angles, while J3 moves away toward positive angles.

### 2) Zero Offsets

For each joint:

1. Jog to a known reference pose (use `JR` or manual control sliders).
2. Measure the physical angle with an angle gauge.
3. In Calibration panel, enter that measurement into **Logical Angle Here**.
4. Click **Zero Here**.

This sets the offset so the current raw position matches the logical angle you entered.

For joints without endstops (J1, J6), align to mechanical marks or a fixed reference before zeroing.

### 3) Scale Calibration (Optional but Recommended)

For each joint:

1. Move to a second pose far from the first (e.g. +60 deg).
2. Measure the physical angle.
3. Compute scale:
   - `scale = (logical2 - logical1) / (raw2 - raw1)`
4. Enter **Scale** and **Offset** and click **Apply**.

### 4) Save Calibration

- Click **Save EEPROM** to persist calibration on the Teensy.

---

## Validation

### Joint Validation

- Command each joint to two points inside its configured range (for example +/-30 deg equivalent within limits) and verify measured angles are within +/-2 deg.

### Cartesian Validation

1. Pick 5-8 positions in workspace.
2. Move robot to each pose.
3. Compare real XYZ with UI FK readout.
4. Target <=5 mm error.

---

## Tips

- Always approach reference angles from the same direction to reduce backlash effects.
- If a joint drifts, re-home and re-zero that joint.
- If Cartesian error is large in one area, check for URDF mismatch or mechanical play.

---

## Commands Reference

- `CFG?` -> Request firmware configuration
- `CAL SET Jn <scale> <offset>` -> Set calibration
- `CAL ZERO Jn <logical_deg>` -> Set offset based on current pose
- `CAL SAVE` / `CAL LOAD` / `CAL RESET` -> EEPROM actions
- `JR Jn <delta_deg> <speed>` -> Jog a joint relative to current angle
- `HP?` -> Query operational home pose config
- `HP EN <0|1>` -> Enable/disable post-home move after `H ALL`
- `HP SET` / `HP SETALL` / `HP SPD` -> Configure operational home targets and speed
- `HP SAVE` / `HP LOAD` / `HP RESET` -> Persist or restore operational home pose config
- `MQ?` -> Query motion-kernel diagnostics (smoothness/jitter counters)
- `npm run model:fit -- <samples.json>` -> Optional TCP/link parameter fit (separate from `CAL`)
