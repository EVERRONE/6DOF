# Hardware Alignment Audit

This table captures current configuration values to align firmware, UI, and URDF. Use it as the calibration checklist.

## Joint Summary

| Joint | Firmware Limits (deg) | URDF Limits (rad / deg) | Endstop | Home Toward Min | Home Logical (deg) | Post Home Offset (deg) | URDF Offset (deg) | Invert Dir |
|------|------------------------|-------------------------|---------|-----------------|--------------------|------------------------|-------------------|------------|
| J1 | -40 .. 30 | -2.8 .. 2.8 / ~-160.3 .. 160.3 | No | N/A | 0 | 0 | 0 | false |
| J2 | -60 .. 0 | -1.3 .. 1.3 / ~-74.5 .. 74.5 | Yes | false | 0 | 0 | 60 | true |
| J3 | 0 .. 70 | -2.1 .. 2.1 / ~-120.3 .. 120.3 | Yes | true | 0 | 0 | 0 | true |
| J4 | -274 .. 0 | -2.5 .. 2.5 / ~-143.2 .. 143.2 | Yes | false | 0 | 0 | 274 | false |
| J5 | -280 .. 0 | -2.5 .. 2.5 / ~-143.2 .. 143.2 | Yes | false | 0 | 0 | 280 | false |
| J6 | -360 .. 360 | continuous | No | N/A | 0 | 0 | 0 | false |

## Notes

- Firmware and UI now consume logical angles; calibration compensates for scale and offset.
- URDF limits are symmetrical, while firmware currently enforces asymmetric ranges (e.g., J2 -60..0).
- Invert direction is currently true for J2/J3 in firmware. Verify positive jog moves away from endstops.
- Homing is configured for J2-J5 only and now leaves joints on the endstop at logical 0.
- Operational home pose (post-home stage) defaults:
  - `enabled=false`
  - `applyAfterHAll=true`
  - `speedDegS=10`
  - `jointsDeg=[0,0,0,0,0,0]` (J2..J5 used)
- UI effective URDF offsets are derived as:
  - `J2..J5: effectiveOffset = -homePose.jointsDeg`
  - `J1/J6: effectiveOffset = joint.urdfOffsetDeg`
- UI URDF direction multipliers:
  - `J1..J4: +1`
  - `J5/J6: -1`
- Verify stepper driver logic-level compatibility with 3.3V outputs; add level shifting if required.
