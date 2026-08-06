# Hardware Reference

Physical build of the arm, and how it maps onto `firmware/config.h`.

Extracted from the original project notes. Where those notes and the firmware
disagree, both values are given and the conflict is called out rather than
quietly resolved — see [Unresolved conflicts](#unresolved-conflicts).

---

## Controller and drivers

| | |
|---|---|
| MCU | Teensy 4.1, 600 MHz, 3.3 V logic, USB serial |
| Drivers | 6× TMC2209, no UART, on 2× CNC Shield V3 |
| Microstepping | 1/16 (the TMC2209 interpolates internally to 1/256 via MicroPlyer) |
| Motor supply | 24 V |
| Logic supply | 3.3 V from the Teensy |
| Hardware E-stop | Physical switch in series with the 24 V supply |

> **TMC2209 microstepping is set by MS1/MS2, and its default is 1/8, not 1/16.**
> That is the opposite of the A4988/DRV8825 a CNC Shield is designed for, where no
> jumpers means full step. If the shield jumpers are left off, every axis moves
> exactly twice as far as commanded and drives into its hard stops. Both MS1 and
> MS2 must be high for 1/16. Verify before homing: command 10° and measure.

The hardware E-stop cuts motor power but not the Teensy, so the firmware keeps
its step count while the motors are dead. After a hardware E-stop the recorded
position is meaningless — re-home.

---

## Motors and reduction

| Joint | Motor | Rated current | Gear pairing | Reduction | µsteps/° |
|-------|-------|---------------|--------------|-----------|----------|
| J1 | Wantai 1.8° | 2.6 A | 16:100 | 6.25 : 1 | 55.556 |
| J2 | 17HS13-0404S-PG5 | 0.4 A | 16:80 × 5:1 planetary | 25 : 1 | 222.222 |
| J3 | 17HS16-2004S1 | 2.0 A | 16:100 | 6.25 : 1 | 55.556 |
| J4 | 14HS08-0404S | 0.4 A | 16:60 | 3.75 : 1 | 33.333 |
| J5 | 14HR05-0504S | 0.5 A | not recorded | 2 : 1 | 17.778 |
| J6 | 8HS11-0204S | 0.2 A | direct | 1 : 1 | 8.889 |

The µsteps/° column is `USTEPS_PER_DEG` in `firmware/config.h`. Every value is
consistent with its documented gear pairing at 1/16 microstepping on a 200
step/rev motor:

```
µsteps per motor revolution = 200 × 16 = 3200
µsteps per degree at 1:1    = 3200 / 360 = 8.889
reduction                   = USTEPS_PER_DEG / 8.889
```

which gives 6.25, 25, 6.25, 3.75, 2.0 and 1.0 — matching the table. J5's gear
pairing was never written down; its 2:1 is implied by the calibration value.

> These numbers are **derived, not measured**. They are self-consistent, which
> rules out a transcription error, but not a wrong assumption about a gear. A
> 20T→120T pair reads 6.0 where a 20T→125T reads 6.25, and the two look nearly
> identical. Measure each axis before trusting a long move: see the bring-up
> sequence in [firmware/README.md](../firmware/README.md).

---

## Pin map (Teensy 4.1)

| Joint | STEP | DIR | Endstop |
|-------|------|-----|---------|
| J1 | 2 | 5 | — |
| J2 | 3 | 6 | 30 |
| J3 | 4 | 7 | 31 |
| J4 | 22 | 23 | 32 |
| J5 | 24 | 25 | 33 |
| J6 | 26 | 27 | — |

Driver enable: pin 8 for J1–J3, pin 9 for J4–J6. Both active LOW.

Endstops are Omron D2F-L microswitches, wired C to the Teensy pin and NO to GND,
with `INPUT_PULLUP`, so a closed switch reads LOW. J1 and J6 have no switch and
cannot be homed automatically.

---

## Travel limits

| Joint | Min | Max | Range |
|-------|-----|-----|-------|
| J1 | −40° | +30° | 70° |
| J2 | 0° | +60° | 60° |
| J3 | 0° | +70° | 70° |
| J4 | 0° | +274° | 274° |
| J5 | 0° | +280° | 280° |
| J6 | −360° | +360° | 720° |

These are `JOINT_MIN` / `JOINT_MAX` in `firmware/config.h`, **mirrored** in the
web app at `robot-arm-control/src/kinematics/robotModel.ts`. A test there asserts
the exact values, so the two cannot drift apart silently. Change both together.

The reachable workspace that follows from these limits is small and strongly
off-centre, because J1 travels only 70° and J2 only 60°:

| Axis | Min | Max |
|------|-----|-----|
| X | −203 mm | +70 mm |
| Y | −115 mm | +138 mm |
| Z | +146 mm | +384 mm |

That is the axis-aligned outer bound, not the reachable set — a point inside the
box can still be out of reach.

---

## Homing

| Joint | Datum angle | Parks at |
|-------|-------------|----------|
| J2 | 0° | 5° |
| J3 | 0° | 55° |
| J4 | 0° | 129° |
| J5 | 0° | 220° |

`HOME_POSITION` and `POST_HOME_ANGLES` in `firmware/config.h`. Homing runs fast
seek → back off → slow second approach → park, all acceleration limited, and each
joint's datum is set independently of the others.

The seek is bounded per joint by `HOMING_MAX_TRAVEL`, sized as the joint's own
range plus 10°. A joint cannot need more than its own travel to reach its switch,
and bounding it means a wrong seek direction produces a clean error instead of
grinding against a hard stop.

---

## Unresolved conflicts

The original project notes and `firmware/config.h` disagree on two arrays. Both
control which way a motor turns, so getting either wrong drives a joint into a
hard stop. **Neither can be settled without the arm.**

### Direction inversion — RESOLVED by measurement

Each joint was jogged in the positive direction on the arm and watched against
its endstop. Every switched axis carries its switch at the **minimum** end of
travel, so a positive command must move the joint *away* from it.

| Joint | Old `config.h` | Project notes | Observed | Now |
|-------|---------------|---------------|----------|-----|
| J1 | false | false | turns correctly | false |
| J2 | true | false | `+` moved **toward** the switch | **false** |
| J3 | true | false | `+` moved away — correct | **true** |
| J4 | false | true | `+` moved **toward** the switch | **true** |
| J5 | false | true | `+` moved **toward** the switch | **true** |
| J6 | false | false | turns correctly | false |

The notes were right about J2, J4 and J5 and wrong about J3. `config.h` was wrong
about J2, J4 and J5 and right about J3. Neither source was reliable on its own,
which is why this had to come off the arm.

### Homing direction — RESOLVED

All four switched axes home toward the minimum. `HOME_POSITION` is 0 for every
one of them, `JOINT_MIN` is 0, and the resting poses in `POST_HOME_ANGLES` are
all positive, so a joint parks by moving up and away from its switch. The
project notes had this right.

| Joint | Old `config.h` | Now |
|-------|---------------|-----|
| J2 | false | **true** |
| J3 | true | true |
| J4 | false | **true** |
| J5 | false | **true** |

A wrong value here sends the joint away from its switch. The firmware reports
`Endstop not found in travel range, check HOME_TOWARD_MIN direction` after the
joint's own range of travel, so the failure is bounded and diagnostic — but it
still moves the joint the wrong way first.

### Driver enable pin

The notes say one enable pin, 8, common to all drivers. `config.h` drives pin 8
for J1–J3 and pin 9 for J4–J6. If pin 9 is not actually wired, the second
shield's drivers follow whatever its EN pin is pulled to, which on a CNC Shield
usually means permanently enabled — the motors would hold torque even when the
firmware believes they are off.

### How to settle all three

Work one joint at a time, with the arm free to move:

1. `E 1`, then a small move: `J 0 5 0 0 0 0 5`. Note which way J2 turns. Repeat
   per joint. Fix `INVERT_DIR` so that a positive command moves the joint in the
   positive direction of the limits table above.
2. With direction confirmed, `H 2` and watch. If it moves away from the switch,
   flip `HOME_TOWARD_MIN` for that joint. Only run `H ALL` once every joint has
   been homed individually.
3. `E 0` and check by hand whether all six motors actually lose holding torque.
   If J4–J6 stay energised, pin 9 is not wired and the notes are right.

Steps 2 and 5 of [BRINGUP.md](BRINGUP.md) walk through this in the right order.
Update this file and `firmware/config.h` together with what you find, and delete
the conflicting column.

---

## Related

- [BRINGUP.md](BRINGUP.md) — ordered checklist for the first session with the arm
- [firmware/README.md](../firmware/README.md) — firmware architecture, tuning
- [KINEMATICS.md](KINEMATICS.md) — the kinematic chain and the solvers
- [SERIAL_PROTOCOL.md](SERIAL_PROTOCOL.md) — the host link
- [../URDF.md](../URDF.md) — link geometry, the source for the kinematic chain
