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

| Joint | Min | Max | Range | URDF | Confirmed on the arm |
|-------|-----|-----|-------|------|----------------------|
| J1 | −90° | +90° | 180° | ±160° | not yet — held back for cables |
| J2 | +2° | +86° | 84° | 89.5° | not yet |
| J3 | +2° | **+104°** | 102° | 161.4° | **self-collision at ~107°** |
| J4 | +2° | **+332°** | 330° | 308.2° | yes — more than the design claimed |
| J5 | +2° | **+222°** | 220° | 274.2° | yes — less |
| J6 | −360° | +360° | 720° | placeholder | n/a |

Three were checked by approaching from the inside and moved. J4 has more travel
than the design claimed and J5 has less, which is the ordinary outcome of a
built machine differing from its drawing.

**J3 is different in kind.** It does not reach a mechanical stop: the arm hits
*itself* at about 107°, well inside the design range. A single joint limit is a
poor way to hold that, because where the arm fouls depends on where J2 and the
wrist are.

The app now checks self-collision directly — see
[Self-collision](#self-collision) below — so J3's 104° is a backstop rather than
the only protection. It stays because the collision model is 2° optimistic
against the one measurement available.

---

## Self-collision

Joint limits and collisions guard different failures. A limit is the end of a
joint's own travel; past it is metal on metal, and an open-loop stepper driven
there loses steps in silence. A collision is the arm reaching a pose where two
parts occupy the same space while every joint sits comfortably inside its range,
which no per-joint number can express.

**1.7% of poses drawn uniformly from inside the joint limits are
self-colliding.** Small, but not something a limit can catch.

Each link carries three oriented boxes, fitted to its STL mesh along the part's
own longest axis, in `robot-arm-control/src/kinematics/collisionModel.ts`.
Boxes rather than spheres: these are printed plates and brackets, and a sphere
around a 210 mm plate 30 mm thick is fat in the two directions that matter.
Against the one collision known from the arm — J3 fouling at about 107° — the
sphere model predicted 83° and the box model predicts 109°.

Twelve of the fifteen link pairs are disabled, and it matters that they are:

- **Overlapping by construction.** Adjacent links always do, and link3's box
  reaches the wrist mount at z=103 mm where link4 and the tool are bolted on.
  Checking them would report a collision in every pose.
- **Never touching.** They cannot reach each other anywhere in the joint range,
  so checking them can only cost time.

That leaves shoulder-vs-forearm and upper-arm-vs-forearm carrying the
information. The set is built by sampling 3000 poses, the same way MoveIt's setup
assistant does it.

> **A 2 mm clearance is added when checking, and the pair list is built from the
> true geometry.** Inflating first would make more pairs overlap at the parked
> pose, disabling the very pairs the margin exists to protect — the model then
> reports its first J3 collision at 64° instead of 109°.

> **The tool link is a placeholder**: a 4 mm stub sized for a bare flange, which
> is why its pair with the forearm is disabled. Once a real tool is fitted that
> becomes the pair that matters most — a pen sticking out is what will hit things
> — and the model has to be regenerated around its real geometry.
>
> Measuring a tool frame does not do this. The tool frame moves the TCP, so FK,
> IK and the viewer follow it; the collision boxes are geometry and know nothing
> about it. A tool can be fitted, measured and driven around while the checker
> still believes the arm ends at the flange.

Regenerate the model if the meshes or the visual transforms in
`RobotModel3D.applyVisualTransform` change.

These come from the mechanical design in [URDF.md](../URDF.md), mapped into
firmware angles through `urdf = URDF_DIRECTION × (logical − POST_HOME_ANGLES)`,
with three degrees taken off each upper bound and the lower bound set two
degrees off the endstop. A software limit belongs inside the mechanical stop:
reaching a software limit should be ordinary, reaching a stop a fault.

The previous values came from the project notes — the same source this bring-up
caught being wrong about direction on four of six axes and homing direction on
three of four. J2 was limited to 60° where the design allows 89.5°.

These are `JOINT_MIN` / `JOINT_MAX` in `firmware/config.h`, **mirrored** in the
web app at `robot-arm-control/src/kinematics/robotModel.ts`. A test there asserts
the exact values, so the two cannot drift apart silently. Change both together.

Superseded by the design values above. The interim step of rescaling the old
limits by the calibration factors — which narrowed J5 to 165° — was working from
the project notes, and the URDF puts J5 at 274°, close to the 280° the notes
carried. That suggests the notes' figure was in true degrees after all and the
rescale was wrong; taking the limits from the design settles it either way.

**Verify from the inside, never by probing.** Jog to a limit and check clearance
remains. A joint that reaches its limit with room to spare has a conservative
limit; one that fouls first means the URDF is optimistic and it wants tightening.
Driving into a hard stop to find it is how an open-loop arm loses steps silently,
which is the failure this bring-up started with.

The reachable workspace that follows, at 13 samples per joint:

| Axis | Min | Max |
|------|-----|-----|
| X | −341 mm | +27 mm |
| Y | −341 mm | +341 mm |
| Z | +23 mm | +437 mm |

103,693 cm³, against 38,816 cm³ under the notes' limits and 16,605 cm³ before the
URDF mapping was applied to the kinematics at all.

That is the axis-aligned outer bound, not the reachable set — a point inside the
box can still be out of reach, and the IK result is the authority. The 3D view
draws this box, computed from the same model rather than from a constant.

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

Both axes are wrist joints with the lightest loads on the arm (2.909:1 and
3.390:1), so there is likely room to reduce their current without affecting
motion. Reduce and watch for lost steps.

For reference, a stepper at rated current normally reaches 60–80 °C, which feels
alarming but is not a fault.

**This got more urgent with the tuning rounds.** J4 and J5 went from 20 and
30 deg/s to 180 and 200, and from 60 and 75 deg/s² to 600 and 700 — nine times
the speed and ten times the acceleration, on the arm's two smallest and hottest
motors. A stepper's available torque falls as it heats, and both rounds were run
cold, so these are the values least likely to survive a long run. The margin
above them was never established, because both axes hit the tuning ceiling
rather than a noise, twice. If either starts losing steps after twenty minutes
of running, this is the first place to look, not the acceleration.

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
