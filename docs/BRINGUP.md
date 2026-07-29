# Bring-up Checklist

Everything that could not be verified without the arm, in the order to do it.

Work through this once, top to bottom, before trusting the arm with a real path.
Each step says what to expect, so a wrong result is recognisable rather than
something you find out by the sound.

**Have a hand on the power switch throughout.** Steps 1 to 4 can drive a joint the
wrong way, and the software cannot know which way is right until you tell it.

---

## 0. Before power

- [ ] Mechanically inspect every joint. Turn each one by hand through its full
      travel with the drivers off, feeling for a spot that catches or grinds.
      The old firmware repeatedly drove joints into their hard stops, so
      stripped printed teeth or a loosened pulley are real possibilities.
- [ ] Check every grub screw on the motor shafts. A loose pulley gives a single
      sharp knock on direction reversal and is the most common cause of it.
- [ ] Confirm the driver enable wiring: pin 8 for J1–J3, pin 9 for J4–J6. The
      project notes described one common pin, so this may not match the build.

## 1. Microstepping — do this first

The TMC2209 sets microstepping from MS1/MS2, and its default with **no jumpers is
1/8, not 1/16**. That is the opposite of the A4988/DRV8825 a CNC Shield is
designed for. `USTEPS_PER_DEG` assumes 1/16, so if the jumpers are off every axis
moves exactly twice as far as commanded — straight into the hard stops.

- [ ] Both MS1 and MS2 high on all six drivers, for 1/16.
- [ ] Verify by measurement: `E 1`, then `J 0 0 10 0 0 0 5`, and measure J3 with a
      protractor.

| Measured | Meaning |
|----------|---------|
| ~10° | Correct |
| ~20° | Still on 1/8 — fix the jumpers, or halve `USTEPS_PER_DEG` |
| anything else | Gear ratio is wrong; see step 3 |

## 2. Direction — `INVERT_DIR`

`firmware/config.h` and the project notes disagree on **four of six axes**. See
[HARDWARE.md](HARDWARE.md#unresolved-conflicts) for the table.

For each joint in turn, command a small positive move and note which way it goes.
A positive command must move the joint toward the **positive** end of its range as
listed in [HARDWARE.md](HARDWARE.md#travel-limits).

```
E 1
J 5 5 5 5 5 5 5        # +5 deg on every joint, slowly
```

- [ ] J1 — positive goes toward +30°
- [ ] J2 — positive goes toward +60°
- [ ] J3 — positive goes toward +70°
- [ ] J4 — positive goes toward +274°
- [ ] J5 — positive goes toward +280°
- [ ] J6 — positive goes toward +360°

Flip the offending entries in `INVERT_DIR`, re-upload, and re-check. **Do not go
on to homing until all six are right** — homing direction is meaningless while a
motor turns the wrong way.

## 3. Calibration — `USTEPS_PER_DEG`

The values in `config.h` are derived from documented gear ratios, never measured.
They are self-consistent, which rules out a typo but not a wrong assumption about
a gear: a 20T→120T pair reads 6.0 where a 20T→125T reads 6.25, and they look
nearly identical.

For each joint: command a large known move, measure what you get, and scale that
axis by `commanded / measured`. A large move makes the error easier to read.

- [ ] J1  - [ ] J2  - [ ] J3  - [ ] J4  - [ ] J5  - [ ] J6

Re-upload and repeat until it converges.

## 4. Endstops

- [ ] Press each switch by hand and watch the `ENDSTOP` report, or the indicators
      in the status bar. J2→pin 30, J3→31, J4→32, J5→33. Closed reads `1`.
- [ ] Measure the margin between each switch and the hard stop behind it. This
      needs to exceed the stopping distance at `HOMING_SPEED`, which is well under
      a degree, but it also determines whether an endstop trip during ordinary
      motion can be caught in time.

## 5. Homing direction — `HOME_TOWARD_MIN`

`config.h` and the notes disagree on **three of four axes**. A wrong value sends
the joint away from its switch.

The seek is bounded by the joint's own range of travel, so a wrong direction now
produces a clean `ERROR Endstop not found in travel range, check HOME_TOWARD_MIN
direction` rather than grinding. It still moves the joint the wrong way first, so
watch it.

**One joint at a time. Do not run `H ALL` yet.**

```
H 2      # then H 3, H 4, H 5
```

Each should: seek toward the switch, stop, back off 2°, approach slowly, then move
to its resting angle and report `HOMED n`.

- [ ] J2 — parks at 5°
- [ ] J3 — parks at 55°
- [ ] J4 — parks at 129°
- [ ] J5 — parks at 220°

- [ ] Then, and only then: `H ALL`. Afterwards the status bar should show J2–J5
      homed and `POSITION UNVERIFIED` gone.
- [ ] Immediately after homing, command the resting pose:
      `J 0 5 55 129 220 0 20`. **Nothing should move more than a fraction of a
      degree.** If a joint travels a long way, the datum bookkeeping is wrong —
      stop and report it, because that was the bug that drove J3 into its hard
      stop.

## 6. Acceleration — the noise

`MAX_JOINT_ACCEL` values are estimates based on the gear reductions. This is the
one parameter that has to be set by ear.

For each joint, run a full-speed move over most of its travel and listen:

| What you hear | What it means |
|---------------|---------------|
| Quiet ramp up and down | Correct |
| Knock or thud at start or stop | `MAX_JOINT_ACCEL` too high for that joint — lower it |
| Grinding at constant speed | `MAX_JOINT_SPEED` past where the driver holds torque — lower it, or raise the driver current |
| Growl at one specific speed only | Motor resonance. J1 and J3 run near 1 rev/s at their maximum, which is the classic NEMA17 mid-band |

- [ ] J1  - [ ] J2  - [ ] J3  - [ ] J4  - [ ] J5  - [ ] J6

J2 carries the whole arm and wants the gentlest ramp. Check `Vref` on each driver
(~0.6–1.0 V depending on the motor) before blaming the acceleration for a joint
that loses steps under load.

## 7. Stops

- [ ] Start a slow move, send `S` mid-move. The arm must stop at once, keep
      holding torque, and the app must show `POSITION UNVERIFIED`.
- [ ] Re-home, then start a move and press cancel in the app. The arm should
      decelerate smoothly and keep its position — no `POSITION UNVERIFIED`.
- [ ] Trip an endstop by hand during an ordinary move. Expect an immediate stop
      and `ERROR Endstop triggered during move on Jn`.
- [ ] `E 0` and check by hand whether all six motors lose holding torque. If
      J4–J6 stay energised, pin 9 is not wired — see step 0.

## 8. Kinematics against the real arm

- [ ] Home, then read the XYZ in the status bar. Type the same XYZ into the
      Cartesian panel and press *Move to Position*. The arm should barely move and
      IK should report a residual well under 1 mm.
- [ ] Jog 20 mm in X, then 20 mm in Z, and measure the actual tool movement. A
      consistent scale error points at `USTEPS_PER_DEG`; a direction error points
      at the URDF; a large offset points at `TOOL_OFFSET`, which is currently zero.
- [ ] Set `TOOL_OFFSET` in `robot-arm-control/src/kinematics/robotModel.ts` once
      the real tool geometry is known.

## 9. A path, end to end

- [ ] Teach four waypoints well inside the workspace, plan, and run at low speed.
      Watch for a pause at each waypoint: there should be none, since the firmware
      carries speed through the junctions.
- [ ] Run the same path with `loopCount` set to 5. The arm should return to the
      same place each cycle. Drift between cycles means lost steps — driver
      current, then acceleration.
- [ ] Pull the USB cable mid-path. The app should notice within about two seconds,
      abandon the path, and say it is reconnecting. Plug it back in: it should
      reopen the port on its own and warn that re-homing is needed.

---

## When you are done

Update these together, and delete the conflicting columns from
[HARDWARE.md](HARDWARE.md#unresolved-conflicts):

- `firmware/config.h` — `INVERT_DIR`, `HOME_TOWARD_MIN`, `USTEPS_PER_DEG`,
  `MAX_JOINT_ACCEL`, `MAX_JOINT_SPEED`
- `robot-arm-control/src/kinematics/robotModel.ts` — `JOINT_MAX_SPEED_DEG_S`,
  `JOINT_MAX_ACCEL_DEG_S2`, and `TOOL_OFFSET`

The mirrored values are asserted by tests in both projects, so if you change one
side only, the suite tells you.
