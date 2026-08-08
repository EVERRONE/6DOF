# Bring-up Checklist

Everything that could not be verified without the arm, in the order to do it.

Work through this once, top to bottom, before trusting the arm with a real path.
Each step says what to expect, so a wrong result is recognisable rather than
something you find out by the sound.

**Have a hand on the power switch throughout.** Steps 1 to 4 can drive a joint the
wrong way, and the software cannot know which way is right until you tell it.

Everything below can be done from the web app, no serial monitor needed — and a
serial monitor could not hold the port at the same time anyway. The **Jog** tab has
per-joint nudge buttons with a selectable step, and the **Console** underneath it
shows everything the arm reports and sends any command literally. `Escape` is the
emergency stop.

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

**Done once, and the values in `config.h` came from it.** Repeat it whenever the
load changes, a motor is replaced, or the supply changes.

Acceleration is the one parameter that has to be set by ear. Use the Tuning tab,
which sets both limits over the `V` command so an attempt costs a slider move
rather than a re-flash. Listen to one axis at a time, in the order J2, J3, J1,
J4, J5, J6 — J2 carries the whole arm, so it is the rate-limiting one.

| What you hear | Where in the move | What it means |
|---------------|-------------------|---------------|
| Quiet ramp up and down | — | Correct |
| Knock or thud | at the start or the stop | `MAX_JOINT_ACCEL` too high for that joint |
| Grinding, constant tone | through the middle | `MAX_JOINT_SPEED` past where the driver holds torque |
| Growl | one speed only, gone above and below | Motor resonance, not a fault |

Raise speed by 25% and acceleration by 50% per round. Raising both together is
safe because the *position* of the noise in the move says which one caused it.
When a round makes a noise, go back one round and take a further 20% off — that
is the margin for a warm motor, a sagging supply and a payload, none of which are
present while tuning.

Re-home after any round that ground. Lost steps are silent and the position
report keeps counting, so tuning the next axis on top of a bad datum is tuning
fiction.

- [x] J1  - [x] J2  - [x] J3  - [x] J4  - [x] J5  - [x] J6

**Result after two rounds**, and the values now in `config.h`:

| | J1 | J2 | J3 | J4 | J5 | J6 |
|---|---|---|---|---|---|---|
| speed °/s | 120 | 90 | 120 | 180 | 200 | 360 |
| accel °/s² | 400 | 300 | 400 | 600 | 700 | 1000 |
| at the motor | 125 RPM | 375 RPM | 113 RPM | 87 RPM | 113 RPM | 60 RPM |

Eight times the bring-up values. Round one ended with five axes on the ceiling
and J5 at 94 °/s; the ceilings were doubled, and round two ended with all six on
the ceiling again.

> **These are tested-silent, not safe-with-margin.** Every axis reached
> `TUNING_MAX_*` in both rounds without ever making a noise, so both rounds
> measured the ceiling rather than the arm, and the 20% back-off from a found
> limit was never taken — nothing was ever found. Held here deliberately: this is
> fast enough for the work, and the ceilings sit 1.5× above so a third round
> needs no re-flash.
>
> What is not covered: a warm motor, a sagging supply, a payload. See the J4/J5
> note in [HARDWARE.md](HARDWARE.md#still-open).

Check `Vref` on each driver (~0.6–1.0 V depending on the motor) before blaming
acceleration for a joint that loses steps under load. J4 and J5 run hot — see
`HARDWARE.md`.

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
      at the URDF; a large fixed offset points at the tool frame.

### Measuring the tool frame

Use the **Tool frame** panel at the top of the Cartesian tab. Millimetres and
degrees; it remembers what you enter and prints the literal to paste into
`robotModel.ts`.

- [ ] **Offset — by touching, not by measuring.** Press *Start* under "Find the
      tip by touching". Put something pointed on the bench — a nail, a sharpened
      screw — and touch its tip with the tool's tip four times, approaching from
      genuinely different directions. Press *Set tool*.

      Typing the offset in instead needs somebody who knows which way frame 6's
      X, Y and Z point, and that is not visible on the machine: it falls out of a
      chain of URDF rotations. Touching asks nothing but the touching.

      Read the residual it reports. That figure is your aim, the arm's
      repeatability and the model's fidelity added together — the first real
      number for how good this arm is.

      Check it afterwards: spin J6 through 180°. The tip should sweep a circle of
      twice the radial offset, and the reported position should follow it. On a
      bare flange the reported position does not move at all, which is the
      quickest way to tell the offset is still zero.
- [ ] **Rotation.** Level the base first, or this measures the table. Command the
      tool to a known attitude, put a digital inclinometer on it, and enter the
      difference. Roll and pitch come straight off the gauge; yaw needs a square
      or a straight edge against a known axis.

      **Measure it in four to six different arm poses, not one.** See below.
- [ ] Paste the printed `DEFAULT_TOOL_FRAME` into
      `robot-arm-control/src/kinematics/robotModel.ts`. Until you do, the value
      lives in one browser's local storage and another machine driving this arm
      has a different idea of where the tool is.

### Telling a crooked tool from a wrong model — NOT YET BUILT

**Planned for a later session. Nothing in the app does this yet; the Tool frame
panel takes numbers but cannot help you find them.**

Calibrating the tool rotation from a single pose is a trap. A tool bolted on
crooked and an error in the kinematic model look *identical* at one pose, so
whatever you measure there gets written into the tool frame — including every bit
of model error that happened to be present. Move somewhere else and it reappears,
now with the tool frame carrying a correction that no longer applies.

The two are distinguishable, and the test is simple:

> Measure the tool's attitude in **several different arm poses**, spread across
> the workspace.
>
> **Error the same everywhere → the tool.** Write it into the tool frame.
> **Error changes with the pose → the model.** The tool frame cannot fix it, and
> putting it there hides it.

What to build when this comes up:

- A calibration panel that commands the tool to one nominal attitude — axis
  aligned, say — at four to six different arm poses in turn.
- A field per pose for the measured roll / pitch / yaw off a digital
  inclinometer.
- It solves for the constant part, which is the tool rotation, and reports the
  **spread** separately.

That spread is the valuable half. It is the first real number for how good the
arm's kinematic model actually is, and nothing measured so far tells us that.

Do the ruler checks in the list above first: a scale or direction error found
with a ruler is much easier to attribute than the same error seen through an
inclinometer.

You will need a levelled base — otherwise this measures the bench — a digital
inclinometer for roll and pitch, and a square for yaw.

---

> Fitting a tool does **not** teach the collision checker about it.
> `collisionModel.ts` carries boxes for the links only, so a long tool can reach
> the arm without anything noticing.

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
  `JOINT_MAX_ACCEL_DEG_S2`, and `DEFAULT_TOOL_FRAME`

The mirrored values are asserted by tests in both projects, so if you change one
side only, the suite tells you.
