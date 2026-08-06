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

| Joint | Motor | Rated current | Documented pairing | Reduction | µsteps/° |
|-------|-------|---------------|--------------------|-----------|----------|
| J1 | Wantai 1.8° | 2.6 A | 16:100 | 6.250 : 1 | 55.556 |
| J2 | 17HS13-0404S-PG5 | 0.4 A | 16:80 × 5:1 planetary | 25.000 : 1 | 222.222 |
| J3 | 17HS16-2004S1 | 2.0 A | 16:100 → really 16:90 | **5.625 : 1** | **50.000** |
| J4 | 14HS08-0404S | 0.4 A | 16:60 → wrong | **2.909 : 1** | **25.862** |
| J5 | 14HR05-0504S | 0.5 A | never recorded | **3.390 : 1** | **30.132** |
| J6 | 8HS11-0204S | 0.2 A | direct | 1.000 : 1 | 8.889 |

The µsteps/° column is `USTEPS_PER_DEG` in `firmware/config.h`. Every value is
consistent with its documented gear pairing at 1/16 microstepping on a 200
step/rev motor:

```
µsteps per motor revolution = 200 × 16 = 3200
µsteps per degree at 1:1    = 3200 / 360 = 8.889
reduction                   = USTEPS_PER_DEG / 8.889
```

The bold values are **measured**, folded in from an earlier calibration of this
same arm. The rest were derived from the documented pairings, and three of them
were wrong — exactly the failure the derivation was warned about: a 20T→120T pair
reads 6.0 where a 20T→125T reads 6.25, and the two look nearly identical on the
machine.

J5 is the one that mattered. Its pairing was never written down, so 2:1 was a
guess, and it is out by 70%.

Cross-checked independently: applying the calibration scales to the angles the
firmware reports at the parked pose reproduces the operational home pose stored
by the earlier project to within 0.0° on J4, 1.0° on J5, 1.1° on J3 and 2.0° on
J2 — two records that never shared a number agreeing on the same pose.

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
| J5 | 0° | **+165°** | 165° |
| J6 | −360° | +360° | 720° |

These are `JOINT_MIN` / `JOINT_MAX` in `firmware/config.h`, **mirrored** in the
web app at `robot-arm-control/src/kinematics/robotModel.ts`. A test there asserts
the exact values, so the two cannot drift apart silently. Change both together.

The limits were recorded in the same scale as the old, wrong step scaling, so
correcting the calibration moved where each number sits physically — the hard
stop does not move, the number describing it does. The honest conversion is
`limit × scale`, which narrows J5 from 280° to 165° and would widen J3 to 77.8°
and J4 to 353°. J5 is narrowed; J3 and J4 keep their old, tighter figures,
because it is not known whether those came from measurement or from the same
notes as the wrong ratios, and a limit that is too tight only costs travel while
one that is too generous drives into the stop. Open J3 and J4 up once their real
travel has been measured.

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
| J2 | 0° | 15° |
| J3 | 0° | 41.1° |
| J4 | 0° | 165° |
| J5 | 0° | 131° |

The parked pose was measured: the arm was jogged into the wanted rest position
and these are the angles reported there. It is also the anchor for the 3D view —
URDF zero is this pose, so the model and the machine agree there by construction.

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

### Driver enable pin — RESOLVED

`config.h` is right: pin 8 drives J1–J3 and pin 9 drives J4–J6, and pin 9 is
genuinely wired. Verified on the arm by sending `E 0` and turning J4 and J5 by
hand — both released, so the second shield's drivers do follow the firmware.
The old notes, which described a single common enable on pin 8, were wrong.

This also rules out permanent energisation as a cause of the heat seen on J4 and
J5: those drivers do switch off when asked.

All three above were settled on the arm. Steps 2 and 5 of
[BRINGUP.md](BRINGUP.md) describe the procedure used.

---

## Still open

### J4 and J5 run hot

Both motors get very hot in normal use. Ruled out so far:

- **Not permanent energisation.** `E 0` releases both, so the split enable works.
- Vref was reported as set correctly, but the actual voltages have not been
  measured, so "correctly" is unverified.

The motors on this arm span 0.2 A to 2.6 A, and the two that run hot are the two
smallest of the loaded axes:

| Axis | Motor | Rated |
|------|-------|-------|
| J4 | 14HS08-0404S | 0.4 A |
| J5 | 14HR05-0504S | 0.5 A |

A single Vref applied across all six drivers would over-drive these by several
times. Both are also small NEMA14 frames with little mass to shed heat, so they
reach a given temperature faster than the NEMA17s on J1 and J3 at equal load.

**To settle it, measure:**

1. Vref at the test point on the J4 and J5 drivers, in volts, and compare against
   the rated currents above using the formula for these specific driver modules
   (it depends on the sense resistor, 0.11 Ω or 0.15 Ω).
2. Whether they heat up while *enabled and stationary*: `E 1`, move nothing, wait
   two minutes, feel. Hot at standstill points at holding current — either Vref,
   or standstill current reduction being disabled. The TMC2209 configures that
   from `PDN_UART` in standalone mode, and on a CNC Shield that pin's state
   varies by driver module.

Both axes are wrist joints with the lightest loads on the arm (3.75:1 and 2:1)
and now run at 20 and 30 deg/s, so there is likely room to reduce their current
without affecting motion. Reduce and watch for lost steps.

For reference, a stepper at rated current normally reaches 60–80 °C, which feels
alarming but is not a fault.

### URDF angles vs firmware angles — anchor RESOLVED, signs NOT

The two count from different places, which is why the 3D view drew the arm
collapsed on the floor while the real one stood upright. `robotModel.ts` now
maps between them:

```
urdf = URDF_DIRECTION * (logical - HOME_POSE_DEG)
URDF_DIRECTION = [+1, -1, +1, -1, +1, -1]
```

**The anchor is settled.** URDF zero is the pose the arm parks in after homing.
At all-zero URDF angles this chain puts J3 at 296 mm with the upper arm vertical
and the forearm level at 311 mm, reaching out 202 mm — which is the pose the arm
is set to rest in. An earlier calibration of this machine arrived at the same
anchoring independently. Two tests hold it.

**The per-joint signs are not settled.** They were inherited from that earlier
work, which recorded J5 and J6 as running opposite to the URDF, combined with
its joints counting negatively away from their endstops where these count
positively — which flips J2 and J4 as well. That reasoning is sound but it is
reasoning, not measurement.

No test in the suite can catch a wrong sign. FK and the viewer both read
`URDF_DIRECTION`, so they agree with each other whatever it says: setting it to
all `+1` leaves all 115 tests green. Only the machine can settle it.

**To settle it,** with the arm homed and the 3D view open, jog each joint on its
own by 20 degrees or so and watch both:

- [ ] J1 — model turns the same way as the arm
- [ ] J2 — same
- [ ] J3 — same
- [ ] J4 — same
- [ ] J5 — same
- [ ] J6 — same

Any joint where the model goes the opposite way has the wrong sign: flip that
entry in `URDF_DIRECTION` and in the test that records it. Do this before
trusting Cartesian moves, because a wrong sign there sends the solver's
correction the wrong way on that joint.

---

## Related

- [BRINGUP.md](BRINGUP.md) — ordered checklist for the first session with the arm
- [firmware/README.md](../firmware/README.md) — firmware architecture, tuning
- [KINEMATICS.md](KINEMATICS.md) — the kinematic chain and the solvers
- [SERIAL_PROTOCOL.md](SERIAL_PROTOCOL.md) — the host link
- [../URDF.md](../URDF.md) — link geometry, the source for the kinematic chain
